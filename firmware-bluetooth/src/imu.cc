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



#if DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c))

#define IMU_SAMPLE_RATE_MS 16
#define IMU_VIRTUAL_INTERFACE 0x1000

static const struct device* imu_dev;
static void imu_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(imu_work, imu_work_fn);

extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

static void imu_work_fn(struct k_work* work) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    if (!imu_dev) {
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
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
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]) < 0) {
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    double x_ms2 = sensor_value_to_double(&accel[0]);
    double y_ms2 = sensor_value_to_double(&accel[1]);
    double z_ms2 = sensor_value_to_double(&accel[2]);
    
    double gx_rads = sensor_value_to_double(&gyro[0]);
    double gy_rads = sensor_value_to_double(&gyro[1]);
    double gz_rads = sensor_value_to_double(&gyro[2]);
    
    double x_g = x_ms2 / 9.81;
    double y_g = y_ms2 / 9.81;
    double z_g = z_ms2 / 9.81;
    
    double gx_dps = gx_rads * 180.0 / 3.14159;
    double gy_dps = gy_rads * 180.0 / 3.14159;
    double gz_dps = gz_rads * 180.0 / 3.14159;
    
    uint8_t x_scaled = (uint8_t)(128 + (x_g * 128));
    uint8_t y_scaled = (uint8_t)(128 + (y_g * 128));
    uint8_t z_scaled = (uint8_t)(128 + (z_g * 128));
    
    uint8_t gx_scaled = (uint8_t)(128 + (gx_dps * 128 / 125));
    uint8_t gy_scaled = (uint8_t)(128 + (gy_dps * 128 / 125));
    uint8_t gz_scaled = (uint8_t)(128 + (gz_dps * 128 / 125));
    
    if (x_g > 1.0) x_scaled = 255; else if (x_g < -1.0) x_scaled = 0;
    if (y_g > 1.0) y_scaled = 255; else if (y_g < -1.0) y_scaled = 0;
    if (z_g > 1.0) z_scaled = 255; else if (z_g < -1.0) z_scaled = 0;
    
    if (gx_dps > 125.0) gx_scaled = 255; else if (gx_dps < -125.0) gx_scaled = 0;
    if (gy_dps > 125.0) gy_scaled = 255; else if (gy_dps < -125.0) gy_scaled = 0;
    if (gz_dps > 125.0) gz_scaled = 255; else if (gz_dps < -125.0) gz_scaled = 0;
    
    // Quick fix: don't know why yet, but gyro and accel data need to be inverted
    imu_report_t imu_report = { 
        .accel_x = gx_scaled,
        .accel_y = gy_scaled,
        .accel_z = gz_scaled,
        .gyro_x = x_scaled,
        .gyro_y = y_scaled,
        .gyro_z = z_scaled
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
        LOG_ERR("IMU device not ready");
        return false;
    }
    struct sensor_value odr_attr;
    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for accelerometer");
        return false;
    }

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for gyroscope");
        return false;
    }

    struct sensor_value accel_scale_attr;
    accel_scale_attr.val1 = 2;
    accel_scale_attr.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                     SENSOR_ATTR_FULL_SCALE, &accel_scale_attr);

    struct sensor_value gyro_scale_attr;
    gyro_scale_attr.val1 = 125;
    gyro_scale_attr.val2 = 0;

    sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                     SENSOR_ATTR_FULL_SCALE, &gyro_scale_attr);
    if (sensor_sample_fetch(imu_dev) < 0) {
        LOG_ERR("Initial sensor sample fetch failed");
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