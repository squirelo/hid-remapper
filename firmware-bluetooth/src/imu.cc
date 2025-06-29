#include "imu.h"
#include "imu_descriptor.h"
#include "config.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "platform.h"
#include "remapper.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_DBG);

#define CHK(X) ({ int err = X; if (err != 0) { LOG_ERR("%s returned %d (%s:%d)", #X, err, __FILE__, __LINE__); } err == 0; })

// Check if IMU device exists in device tree
#if DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c))

#define IMU_SAMPLE_RATE_MS 50
#define IMU_VIRTUAL_INTERFACE 0x0000  // Virtual interface ID for 6-axis IMU

// 6-axis IMU support (accelerometer + gyroscope)
static const struct device* imu_dev;
static void imu_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(imu_work, imu_work_fn);

// External LED references (assuming these are available globally)
extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

// Add IMU trigger support
#ifdef CONFIG_LSM6DSL_TRIGGER
static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    /* Schedule work to handle sensor reading in main thread context */
    k_work_submit((struct k_work*)&imu_work);
}
#endif

/* Called every 50 ms (20 Hz) to read IMU (accel + gyro) and queue/process it */
static void imu_work_fn(struct k_work* work) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    if (!imu_dev) {
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }

    /* Fetch accelerometer data */
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_WRN("Accel fetch fail");
        gpio_pin_set_dt(&led0, false);  // Turn off LED on failure
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    /* Get accelerometer channel values */
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0) {
        LOG_WRN("Accel channel fail");
        gpio_pin_set_dt(&led0, false);  // Turn off LED on failure
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    /* Fetch gyroscope data */
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_WRN("Gyro fetch fail");
        gpio_pin_set_dt(&led0, false);  // Turn off LED on failure
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    /* Get gyroscope channel values */
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]) < 0) {
        LOG_WRN("Gyro channel fail");
        gpio_pin_set_dt(&led0, false);  // Turn off LED on failure
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    /* Convert accelerometer to double values */
    double x_ms2 = sensor_value_to_double(&accel[0]);
    double y_ms2 = sensor_value_to_double(&accel[1]);
    double z_ms2 = sensor_value_to_double(&accel[2]);
    
    /* Convert gyroscope to double values (rad/s) */
    double gx_rads = sensor_value_to_double(&gyro[0]);
    double gy_rads = sensor_value_to_double(&gyro[1]);
    double gz_rads = sensor_value_to_double(&gyro[2]);
    
    /* Log raw values occasionally for debugging */
    static int debug_counter = 0;
    if (++debug_counter >= 40) {  // Log every 2 seconds (40 * 50ms)
        LOG_DBG("Accel raw: X=%.3f Y=%.3f Z=%.3f m/s²", x_ms2, y_ms2, z_ms2);
        LOG_DBG("Gyro raw: X=%.3f Y=%.3f Z=%.3f rad/s", gx_rads, gy_rads, gz_rads);
        debug_counter = 0;
    }
    
    /* Convert accelerometer from m/s² to g units (1g ≈ 9.81 m/s²) */
    double x_g = x_ms2 / 9.81;
    double y_g = y_ms2 / 9.81;
    double z_g = z_ms2 / 9.81;
    
    /* Convert gyroscope from rad/s to dps (degrees per second) */
    double gx_dps = gx_rads * 180.0 / 3.14159;
    double gy_dps = gy_rads * 180.0 / 3.14159;
    double gz_dps = gz_rads * 180.0 / 3.14159;
    
    /* Scale accelerometer to 8-bit range (0-255) with 128 as center (0g) */
    /* Assuming typical ±2g range, scale accordingly */
    uint8_t x_scaled = (uint8_t)(128 + (x_g * 64));  // ±2g maps to 0-255
    uint8_t y_scaled = (uint8_t)(128 + (y_g * 64));
    uint8_t z_scaled = (uint8_t)(128 + (z_g * 64));
    
    /* Scale gyroscope to 8-bit range (0-255) with 128 as center (0 dps) */
    /* Assuming typical ±250 dps range, scale accordingly */
    uint8_t gx_scaled = (uint8_t)(128 + (gx_dps * 128 / 250));  // ±250 dps maps to 0-255
    uint8_t gy_scaled = (uint8_t)(128 + (gy_dps * 128 / 250));
    uint8_t gz_scaled = (uint8_t)(128 + (gz_dps * 128 / 250));
    
    /* Clamp accelerometer values to valid range */
    if (x_g > 2.0) x_scaled = 255; else if (x_g < -2.0) x_scaled = 0;
    if (y_g > 2.0) y_scaled = 255; else if (y_g < -2.0) y_scaled = 0;
    if (z_g > 2.0) z_scaled = 255; else if (z_g < -2.0) z_scaled = 0;
    
    /* Clamp gyroscope values to valid range */
    if (gx_dps > 250.0) gx_scaled = 255; else if (gx_dps < -250.0) gx_scaled = 0;
    if (gy_dps > 250.0) gy_scaled = 255; else if (gy_dps < -250.0) gy_scaled = 0;
    if (gz_dps > 250.0) gz_scaled = 255; else if (gz_dps < -250.0) gz_scaled = 0;
    
    /* Log scaled values occasionally for debugging */
    if (debug_counter == 20) {  // Log in the middle of the cycle
        LOG_DBG("Accel scaled: X=%d Y=%d Z=%d", x_scaled, y_scaled, z_scaled);
        LOG_DBG("Gyro scaled: X=%d Y=%d Z=%d", gx_scaled, gy_scaled, gz_scaled);
    }
    
    // Create 6-axis IMU report using the new custom descriptor format
    imu_report_t imu_report = { 
        .accel_x = x_scaled,
        .accel_y = y_scaled,
        .accel_z = z_scaled,
        .gyro_x = gx_scaled,
        .gyro_y = gy_scaled,
        .gyro_z = gz_scaled
    };
    
    /* Send the 6-axis IMU report through the standard remapping system */
    handle_received_report((uint8_t*)&imu_report, sizeof(imu_report), IMU_VIRTUAL_INTERFACE);

    /* Blink LED to indicate IMU data is working (same pattern as Bluetooth activity) */
    k_work_cancel_delayable(&activity_led_off_work);  // Cancel any pending LED off work
    gpio_pin_set_dt(&led0, true);                     // Turn LED on immediately
    k_work_reschedule(&activity_led_off_work, K_MSEC(50));  // Schedule LED off after 50ms

#ifndef CONFIG_LSM6DSL_TRIGGER
    /* Only reschedule if not using triggers */
    k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
#endif
}

bool imu_init() {
    imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));
    
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return false;
    }

    LOG_INF("6-axis IMU device found and ready");

    /* Set sampling frequency to 104 Hz (similar to working example) */
    struct sensor_value odr_attr;
    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for accelerometer");
        return false;
    }

    /* Set gyroscope sampling frequency to 104 Hz */
    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for gyroscope");
        return false;
    }

    /* Set accelerometer full scale to ±2g */
    struct sensor_value accel_scale_attr;
    accel_scale_attr.val1 = 2;  /* ±2g */
    accel_scale_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &accel_scale_attr) < 0) {
        LOG_WRN("Cannot set full scale for accelerometer (continuing anyway)");
    }

    /* Set gyroscope full scale to ±250 dps */
    struct sensor_value gyro_scale_attr;
    gyro_scale_attr.val1 = 250;  /* ±250 dps */
    gyro_scale_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &gyro_scale_attr) < 0) {
        LOG_WRN("Cannot set full scale for gyroscope (continuing anyway)");
    }

    /* Take an initial sample to verify sensor is working */
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_ERR("Initial accelerometer sample fetch failed");
        return false;
    }
    
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_ERR("Initial gyroscope sample fetch failed");
        return false;
    }

    /* Create virtual device for 6-axis IMU using custom sensor descriptor */
    parse_descriptor(0x0F0D, 0x00C1, imu_hid_report_desc, IMU_HID_REPORT_DESC_SIZE, IMU_VIRTUAL_INTERFACE, 0);
    device_connected_callback(IMU_VIRTUAL_INTERFACE, 0x0F0D, 0x00C1, 0);
    
    /* Force descriptor update */
    their_descriptor_updated = true;

#ifdef CONFIG_LSM6DSL_TRIGGER
    /* Set up data ready trigger */
    struct sensor_trigger trig;
    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    if (sensor_trigger_set(imu_dev, &trig, imu_trigger_handler) < 0) {
        LOG_WRN("Could not set sensor trigger, falling back to polling");
        /* Fall back to polling mode */
        k_work_schedule(&imu_work, K_MSEC(1000));
    } else {
        LOG_INF("IMU trigger mode enabled");
    }
#else
    /* Use polling mode */
    LOG_INF("IMU polling mode enabled");
    k_work_schedule(&imu_work, K_MSEC(1000));
#endif

    return true;
}

#else // !DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c))

bool imu_init() {
    LOG_INF("IMU device not available on this board - skipping IMU initialization");
    return true;  // Return true so the application doesn't fail
}

#endif // DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c)) 