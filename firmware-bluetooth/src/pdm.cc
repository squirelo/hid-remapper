#include "pdm.h"
#include "pdm_descriptor.h"
#include "config.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "platform.h"
#include "remapper.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(pdm, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_NODELABEL(pdm0)) && DT_NODE_HAS_STATUS(DT_NODELABEL(pdm0), okay)

#define PDM_VIRTUAL_INTERFACE 0x2000
#define PDM_SAMPLE_RATE_MS 10
#define PDM_BUFFER_SIZE 256
#define PDM_SAMPLE_RATE_HZ 16000
#define MAX_ERROR_COUNT_BEFORE_BACKOFF 5
#define ERROR_BACKOFF_MULTIPLIER 4
#define READ_TIMEOUT 1000

#define LED_ACTIVITY_DURATION_MS 50

// Memory slab for DMIC blocks - balanced for responsiveness
#define MAX_BLOCK_SIZE (sizeof(int16_t) * 128)  // 128 samples = 8ms at 16kHz
#define BLOCK_COUNT 12
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

typedef struct {
    float alpha;
    float beta;
    float threshold;
    float gain;
} pdm_config_t;

static pdm_config_t pdm_config = {
    .alpha = 0.7f,  // Balanced response
    .beta = 0.2f,   // Balanced frequency tracking
    .threshold = 0.005f,  // Lower threshold for more sensitivity
    .gain = 3.0f    // Higher gain for more sensitivity
};

static const struct device* dmic_dev;
static void pdm_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(pdm_work, pdm_work_fn);

static volatile bool is_initialized = false;
static volatile bool is_calibrated = false;
static uint32_t error_count = 0;
static uint32_t consecutive_success_count = 0;

static volatile float audio_level = 0.0f;
static volatile float frequency = 0.0f;
static volatile uint8_t volume = 0;

static float rms_filter = 0.0f;
static float peak_filter = 0.0f;
static float frequency_filter = 0.0f;

extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

static float calculate_rms(const int16_t* buffer, size_t size) {
    float sum = 0.0f;
    int16_t max_val = 0;
    
    for (size_t i = 0; i < size; i++) {
        float sample = (float)buffer[i] / 32768.0f;
        sum += sample * sample;
        if (abs(buffer[i]) > max_val) {
            max_val = abs(buffer[i]);
        }
    }
    
    // Use both RMS and peak for more responsive detection
    float rms = sqrtf(sum / size);
    float peak = (float)max_val / 32768.0f;
    
    return fmaxf(rms, peak * 0.7f);  // Combine RMS and peak
}

static float calculate_peak(const int16_t* buffer, size_t size) {
    float peak = 0.0f;
    for (size_t i = 0; i < size; i++) {
        float sample = fabsf((float)buffer[i] / 32768.0f);
        if (sample > peak) {
            peak = sample;
        }
    }
    return peak;
}

static float estimate_frequency(const int16_t* buffer, size_t size) {
    // Simple zero-crossing frequency estimation with noise filtering
    int crossings = 0;
    bool was_positive = false;
    int16_t threshold = 10;  // Lower noise threshold for more sensitivity
    
    for (size_t i = 0; i < size; i++) {
        bool is_positive = buffer[i] >= threshold;
        if (is_positive != was_positive && fabsf(buffer[i]) > threshold) {
            crossings++;
        }
        was_positive = is_positive;
    }
    
    // Convert crossings to frequency estimate
    float time_duration = (float)size / PDM_SAMPLE_RATE_HZ;
    float frequency_estimate = (float)crossings / (2.0f * time_duration);
    
    // Clamp to reasonable range
    return fmaxf(0.0f, fminf(8000.0f, frequency_estimate));
}

static void update_filters(float rms, float peak, float freq) {
    rms_filter = pdm_config.alpha * rms_filter + (1.0f - pdm_config.alpha) * rms;
    peak_filter = pdm_config.alpha * peak_filter + (1.0f - pdm_config.alpha) * peak;
    frequency_filter = pdm_config.beta * frequency_filter + (1.0f - pdm_config.beta) * freq;
}

static int16_t scale_audio_level(float level, float min_level, float max_level) {
    level = fmaxf(min_level, fminf(max_level, level));
    float normalized = (level - min_level) / (max_level - min_level);
    int scaled = (int)((normalized - 0.5f) * 65535.0f);
    return (int16_t)fmaxf(-32768.0f, fminf(32767.0f, (float)scaled));
}

static uint16_t scale_frequency(float freq, float max_freq) {
    freq = fmaxf(0.0f, fminf(max_freq, freq));
    float normalized = freq / max_freq;
    int scaled = (int)(normalized * 65535.0f);
    return (uint16_t)fmaxf(0.0f, fminf(65535.0f, (float)scaled));
}

static uint8_t scale_volume(float volume_level, float max_volume) {
    volume_level = fmaxf(0.0f, fminf(max_volume, volume_level));
    float normalized = volume_level / max_volume;
    int scaled = (int)(normalized * 255.0f);
    return (uint8_t)fmaxf(0.0f, fminf(255.0f, (float)scaled));
}

static void pdm_work_fn(struct k_work* work) {
    if (!dmic_dev || !is_initialized) {
        error_count++;
        // Blink red LED to indicate error
        gpio_pin_toggle_dt(&led0);
        if (error_count > MAX_ERROR_COUNT_BEFORE_BACKOFF) {
            k_work_reschedule(&pdm_work, K_MSEC(PDM_SAMPLE_RATE_MS * ERROR_BACKOFF_MULTIPLIER));
            error_count = 0;
        } else {
            k_work_reschedule(&pdm_work, K_MSEC(PDM_SAMPLE_RATE_MS));
        }
        return;
    }

    // Read DMIC data
    void* buffer;
    size_t size;
    int ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
    if (ret < 0) {
        error_count++;
        consecutive_success_count = 0;
        gpio_pin_set_dt(&led0, false);
        
        // Force reschedule even on errors to prevent stall
        uint32_t delay_ms = (error_count > MAX_ERROR_COUNT_BEFORE_BACKOFF) ? 
                           PDM_SAMPLE_RATE_MS * ERROR_BACKOFF_MULTIPLIER : 
                           PDM_SAMPLE_RATE_MS;
        k_work_reschedule(&pdm_work, K_MSEC(delay_ms));
        return;
    }
    
    // Ensure we got valid data
    if (!buffer || size == 0) {
        error_count++;
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&pdm_work, K_MSEC(PDM_SAMPLE_RATE_MS));
        return;
    }
    
    error_count = 0;
    consecutive_success_count++;
    
    // Process audio data
    int16_t* audio_buffer = (int16_t*)buffer;
    size_t samples = size / sizeof(int16_t);
    float rms = calculate_rms(audio_buffer, samples);
    float peak = calculate_peak(audio_buffer, samples);
    float freq = estimate_frequency(audio_buffer, samples);
    
    update_filters(rms, peak, freq);
    
    // Apply gain (but don't zero out low levels)
    float processed_rms = rms_filter * pdm_config.gain;
    float processed_peak = peak_filter * pdm_config.gain;
    float processed_freq = frequency_filter;
    
    // Update global variables
    audio_level = processed_rms;
    frequency = processed_freq;
    volume = scale_volume(processed_peak, 1.0f);
    
    // Create report
    int16_t audio_level_scaled = scale_audio_level(processed_rms, 0.0f, 1.0f);
    uint16_t frequency_scaled = scale_frequency(processed_freq, 8000.0f); // Max 8kHz
    
    pdm_report_t pdm_report = {
        .audio_level = audio_level_scaled,
        .frequency = frequency_scaled,
        .volume = volume
    };
    
    // Always send report, even if audio level is low
    handle_received_report((uint8_t*)&pdm_report, (int)sizeof(pdm_report_t), PDM_VIRTUAL_INTERFACE);

    // LED activity indication (same as IMU)
    k_work_cancel_delayable(&activity_led_off_work);
    gpio_pin_set_dt(&led0, true);
    k_work_reschedule(&activity_led_off_work, K_MSEC(LED_ACTIVITY_DURATION_MS));
    
    // Free the buffer after processing (with safety check)
    if (buffer) {
        k_mem_slab_free(&mem_slab, &buffer);
    }
    
    // Always reschedule to prevent stall
    k_work_reschedule(&pdm_work, K_MSEC(PDM_SAMPLE_RATE_MS));
}

bool pdm_init() {
    LOG_INF("PDM: Starting initialization");
    #if DT_NODE_EXISTS(DT_NODELABEL(pdm0)) && DT_NODE_HAS_STATUS(DT_NODELABEL(pdm0), okay)
        LOG_INF("PDM: PDM0 node exists and is enabled");
        dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0));
        
        if (!device_is_ready(dmic_dev)) {
            LOG_ERR("PDM: DMIC device not ready");
            return false;
        }
        LOG_INF("PDM: DMIC device is ready");
    #else
        LOG_ERR("PDM: PDM0 node not found or not enabled");
        return false;
    #endif
    
    // Configure DMIC
    struct pcm_stream_cfg stream = {
        .pcm_width = 16,
        .mem_slab = &mem_slab,
    };
    
    struct dmic_cfg cfg = {
        .io = {
            .min_pdm_clk_freq = 1000000,
            .max_pdm_clk_freq = 3500000,
            .min_pdm_clk_dc = 40,
            .max_pdm_clk_dc = 60,
        },
        .streams = &stream,
        .channel = {
            .req_num_streams = 1,
        },
    };
    
    cfg.channel.req_num_chan = 1;
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
    cfg.streams[0].pcm_rate = PDM_SAMPLE_RATE_HZ;
    cfg.streams[0].block_size = MAX_BLOCK_SIZE;
    
    // Set balanced clock for reliable audio
    cfg.io.min_pdm_clk_freq = 1500000;  // 1.5MHz minimum
    cfg.io.max_pdm_clk_freq = 3000000;  // 3MHz maximum
    
    int ret = dmic_configure(dmic_dev, &cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure DMIC: %d", ret);
        return false;
    }
    
    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("Failed to start DMIC: %d", ret);
        return false;
    }
    
    is_initialized = true;
    is_calibrated = true;
    
    // Initialize filters
    rms_filter = 0.0f;
    peak_filter = 0.0f;
    frequency_filter = 0.0f;
    
    // Parse descriptor and register device
    parse_descriptor(0x0F0E, 0x00C2, pdm_hid_report_desc, PDM_HID_REPORT_DESC_SIZE, PDM_VIRTUAL_INTERFACE, 0);
    device_connected_callback(PDM_VIRTUAL_INTERFACE, 0x0F0E, 0x00C2, 0);
    
    their_descriptor_updated = true;
    
    k_work_schedule(&pdm_work, K_MSEC(PDM_SAMPLE_RATE_MS));
    
    return true;
}

void pdm_recalibrate() {
    if (is_calibrated) {
        // Reset filters
        rms_filter = 0.0f;
        peak_filter = 0.0f;
        frequency_filter = 0.0f;
        
        // Reset audio levels
        audio_level = 0.0f;
        frequency = 0.0f;
        volume = 0;
    }
}

void pdm_stop() {
    if (is_initialized && dmic_dev) {
        dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
        is_initialized = false;
        k_work_cancel_delayable(&pdm_work);
    }
}

#else

bool pdm_init() {
    return true;
}

void pdm_recalibrate() {
}

void pdm_stop() {
}

#endif 