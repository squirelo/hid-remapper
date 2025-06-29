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

#if DT_NODE_EXISTS(DT_NODELABEL(lsm6ds3tr_c))

#define IMU_SAMPLE_RATE_MS 50
#define IMU_VIRTUAL_INTERFACE 0x1000

static const struct device* imu_dev;
static void imu_work_fn(struct k_work* work);
static K_WORK_DELAYABLE_DEFINE(imu_work, imu_work_fn);

extern const struct gpio_dt_spec led0;
extern struct k_work_delayable activity_led_off_work;

extern void handle_received_report(const uint8_t* report, int len, uint16_t interface, uint8_t external_report_id);

static void imu_work_fn(struct k_work* work) {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    if (!imu_dev) {
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }

    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_WRN("Accel fetch fail");
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0) {
        LOG_WRN("Accel channel fail");
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_WRN("Gyro fetch fail");
        gpio_pin_set_dt(&led0, false);
        k_work_reschedule(&imu_work, K_MSEC(IMU_SAMPLE_RATE_MS));
        return;
    }
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]) < 0 ||
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]) < 0) {
        LOG_WRN("Gyro channel fail");
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
    
    uint8_t x_scaled = (uint8_t)(128 + (x_g * 64));
    uint8_t y_scaled = (uint8_t)(128 + (y_g * 64));
    uint8_t z_scaled = (uint8_t)(128 + (z_g * 64));
    
    uint8_t gx_scaled = (uint8_t)(128 + (gx_dps * 128 / 250));
    uint8_t gy_scaled = (uint8_t)(128 + (gy_dps * 128 / 250));
    uint8_t gz_scaled = (uint8_t)(128 + (gz_dps * 128 / 250));
    
    if (x_g > 2.0) x_scaled = 255; else if (x_g < -2.0) x_scaled = 0;
    if (y_g > 2.0) y_scaled = 255; else if (y_g < -2.0) y_scaled = 0;
    if (z_g > 2.0) z_scaled = 255; else if (z_g < -2.0) z_scaled = 0;
    
    if (gx_dps > 250.0) gx_scaled = 255; else if (gx_dps < -250.0) gx_scaled = 0;
    if (gy_dps > 250.0) gy_scaled = 255; else if (gy_dps < -250.0) gy_scaled = 0;
    if (gz_dps > 250.0) gz_scaled = 255; else if (gz_dps < -250.0) gz_scaled = 0;
    imu_report_t imu_report = { 
        .accel_x = x_scaled,
        .accel_y = y_scaled,
        .accel_z = z_scaled,
        .gyro_x = gx_scaled,
        .gyro_y = gy_scaled,
        .gyro_z = gz_scaled
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

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &accel_scale_attr) < 0) {
        LOG_WRN("Cannot set full scale for accelerometer");
    }

    struct sensor_value gyro_scale_attr;
    gyro_scale_attr.val1 = 250;
    gyro_scale_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &gyro_scale_attr) < 0) {
        LOG_WRN("Cannot set full scale for gyroscope");
    }
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_ERR("Initial accelerometer sample fetch failed");
        return false;
    }
    
    if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_ERR("Initial gyroscope sample fetch failed");
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