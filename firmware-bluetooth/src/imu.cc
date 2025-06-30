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
#include <math.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_DBG);

#if DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c))

#define IMU_SAMPLE_RATE_MS 16
#define IMU_VIRTUAL_INTERFACE 0x1000
#define PI 3.14159265359
#define FILTER_ALPHA 0.96
#define CALIBRATION_SAMPLES 200
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
#define GYRO_DEADZONE 0.02

static const struct device* imu_dev;
static void imu_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(imu_work, imu_work_fn);

static double filtered_pitch = 0.0;
static double filtered_roll = 0.0;
static double accumulated_yaw = 0.0;
static int64_t last_timestamp = 0;

static double gyro_bias_x = 0.0;
static double gyro_bias_y = 0.0;
static double gyro_bias_z = 0.0;
static double accel_bias_x = 0.0;
static double accel_bias_y = 0.0;
static double accel_bias_z = 0.0;
static bool is_calibrated = false;

extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

static uint8_t scale_angle_to_uint8(double angle, double min_angle, double max_angle) {
    if (angle > max_angle) angle = max_angle;
    if (angle < min_angle) angle = min_angle;
    
    double normalized = (angle - min_angle) / (max_angle - min_angle);
    int scaled = (int)(normalized * 255.0);
    
    if (scaled > 255) scaled = 255;
    if (scaled < 0) scaled = 0;
    
    return (uint8_t)scaled;
}

static void normalize_yaw(double* yaw) {
    while (*yaw >= 360.0) *yaw -= 360.0;
    while (*yaw < 0.0) *yaw += 360.0;
}

static void compute_accel_angles(double ax, double ay, double az, double* pitch, double* roll) {
    *pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    *roll = atan2(ay, az) * RAD_TO_DEG;
}

static double apply_deadzone(double value, double deadzone) {
    if (fabs(value) < deadzone) {
        return 0.0;
    }
    return value > 0 ? value - deadzone : value + deadzone;
}

static bool calibrate_sensors(void) {
    struct sensor_value accel[3];
    struct sensor_value angular[3];
    
    double sum_accel_x = 0.0, sum_accel_y = 0.0, sum_accel_z = 0.0;
    double sum_gyro_x = 0.0, sum_gyro_y = 0.0, sum_gyro_z = 0.0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (sensor_sample_fetch(imu_dev) < 0) {
            return false;
        }
        
        if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0 ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0 ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0) {
            return false;
        }
        
        if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &angular[0]) < 0 ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &angular[1]) < 0 ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &angular[2]) < 0) {
            return false;
        }
        
        sum_accel_x += sensor_value_to_double(&accel[0]);
        sum_accel_y += sensor_value_to_double(&accel[1]);
        sum_accel_z += sensor_value_to_double(&accel[2]);
        
        sum_gyro_x += sensor_value_to_double(&angular[0]);
        sum_gyro_y += sensor_value_to_double(&angular[1]);
        sum_gyro_z += sensor_value_to_double(&angular[2]);
        
        k_msleep(5);
    }
    
    accel_bias_x = sum_accel_x / CALIBRATION_SAMPLES;
    accel_bias_y = sum_accel_y / CALIBRATION_SAMPLES;
    accel_bias_z = sum_accel_z / CALIBRATION_SAMPLES - 9.81;
    
    gyro_bias_x = sum_gyro_x / CALIBRATION_SAMPLES;
    gyro_bias_y = sum_gyro_y / CALIBRATION_SAMPLES;
    gyro_bias_z = sum_gyro_z / CALIBRATION_SAMPLES;
    
    return true;
}

static void imu_work_fn(struct k_work* work) {
    struct sensor_value accel[3];
    struct sensor_value angular[3];

    if (!imu_dev) {
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }

    int64_t now = k_uptime_get();
    double dt = last_timestamp ? (now - last_timestamp) / 1000.0 : IMU_SAMPLE_RATE_MS / 1000.0;
    last_timestamp = now;

    if (!is_calibrated) {
        if (calibrate_sensors()) {
            is_calibrated = true;
        } else {
            k_work_reschedule(&imu_work, K_MSEC(1000));
            return;
        }
    }

    if (sensor_sample_fetch(imu_dev) < 0) {
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0) {
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &angular[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &angular[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &angular[2]) < 0) {
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    double accel_x = sensor_value_to_double(&accel[0]) - accel_bias_x;
    double accel_y = sensor_value_to_double(&accel[1]) - accel_bias_y;
    double accel_z = sensor_value_to_double(&accel[2]) - accel_bias_z;
    
    double angular_x = sensor_value_to_double(&angular[0]) - gyro_bias_x;
    double angular_y = sensor_value_to_double(&angular[1]) - gyro_bias_y;
    double angular_z = sensor_value_to_double(&angular[2]) - gyro_bias_z;
    
    angular_x = apply_deadzone(angular_x, GYRO_DEADZONE);
    angular_y = apply_deadzone(angular_y, GYRO_DEADZONE);
    angular_z = apply_deadzone(angular_z, GYRO_DEADZONE);
    
    double accel_pitch, accel_roll;
    compute_accel_angles(accel_x, accel_y, accel_z, &accel_pitch, &accel_roll);
    
    double angular_x_deg_s = angular_x * RAD_TO_DEG;
    double angular_y_deg_s = angular_y * RAD_TO_DEG;
    double angular_z_deg_s = angular_z * RAD_TO_DEG;
    
    filtered_pitch = FILTER_ALPHA * (filtered_pitch + angular_x_deg_s * dt) + (1.0 - FILTER_ALPHA) * accel_pitch;
    filtered_roll = FILTER_ALPHA * (filtered_roll + angular_y_deg_s * dt) + (1.0 - FILTER_ALPHA) * accel_roll;
    
    accumulated_yaw += angular_z_deg_s * dt;
    
    normalize_yaw(&accumulated_yaw);
    
    uint8_t yaw_scaled = scale_angle_to_uint8(accumulated_yaw, 0.0, 360.0);
    uint8_t pitch_scaled = scale_angle_to_uint8(filtered_pitch, -90.0, 90.0);
    uint8_t roll_scaled = scale_angle_to_uint8(filtered_roll, -180.0, 180.0);
    
    imu_report_t imu_report = { 
        .yaw = yaw_scaled,
        .pitch = pitch_scaled,
        .roll = roll_scaled
    };
    
    handle_received_report((uint8_t*)&imu_report, (int)sizeof(imu_report), IMU_VIRTUAL_INTERFACE);

    k_work_cancel_delayable(&activity_led_off_work);
    gpio_pin_set_dt(&led0, true);
    k_work_reschedule(&activity_led_off_work, K_MSEC(50));

    k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
}

bool imu_init() {
    imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));
    
    if (!device_is_ready(imu_dev)) {
        return false;
    }
    
    struct sensor_value odr_attr;
    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        return false;
    }

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        return false;
    }

    struct sensor_value accel_scale_attr;
    accel_scale_attr.val1 = 2;
    accel_scale_attr.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                     SENSOR_ATTR_FULL_SCALE, &accel_scale_attr);

    struct sensor_value angular_scale_attr;
    angular_scale_attr.val1 = 125;
    angular_scale_attr.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                     SENSOR_ATTR_FULL_SCALE, &angular_scale_attr);
    
    if (sensor_sample_fetch(imu_dev) < 0) {
        return false;
    }

    parse_descriptor(0x0F0D, 0x00C1, imu_hid_report_desc, IMU_HID_REPORT_DESC_SIZE, IMU_VIRTUAL_INTERFACE, 0);
    device_connected_callback(IMU_VIRTUAL_INTERFACE, 0x0F0D, 0x00C1, 0);
    
    their_descriptor_updated = true;

    k_work_schedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));

    return true;
}

#else

bool imu_init() {
    return true;
}

#endif 