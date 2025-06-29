/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// LED1 configuration - Device Tree approach (Blue LED)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

// LED0 configuration - Device Tree approach (Red LED)
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Function to blink LED1 on error
static void led1_blink_error(void)
{
	for (int i = 0; i < 10; i++) {  // Blink 10 times
		gpio_pin_toggle_dt(&led1);
		k_sleep(K_MSEC(200));       // 200ms on/off cycle
	}
	gpio_pin_set_dt(&led1, 0);     // Turn off after blinking
}

static int print_samples;
static int lsm6dsl_trig_cnt;

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
static struct sensor_value magn_x_out, magn_y_out, magn_z_out;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
static struct sensor_value press_out, temp_out;
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	static struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	static struct sensor_value press, temp;
#endif
	lsm6dsl_trig_cnt++;

	LOG_DBG("Trigger handler called, count: %d", lsm6dsl_trig_cnt);
	
	// Toggle RED LED (led0) on each sensor data trigger
	gpio_pin_toggle_dt(&led0);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	/* lsm6dsl external magn */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	/* lsm6dsl external press/temp */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
#endif

	if (print_samples) {
		print_samples = 0;

		accel_x_out = accel_x;
		accel_y_out = accel_y;
		accel_z_out = accel_z;

		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		press_out = press;
		temp_out = temp;
#endif

		LOG_DBG("Sensor data updated in trigger handler");
	}

}
#endif

int main(void)
{
	int cnt = 0;
	char out_str[64];
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET(DT_ALIAS(accel0));

	LOG_INF("Starting HID Remapper Bluetooth firmware");

	// Initialize LED1 (Blue LED) - Device Tree approach
	if (!device_is_ready(led1.port)) {
		LOG_ERR("LED1 GPIO device not ready");
		return 0;
	}

	int ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED1 GPIO: %d", ret);
		led1_blink_error();
		return 0;
	}

	LOG_INF("LED1 (Blue) initialized successfully");
	
	// Initialize LED0 (Red LED) - Device Tree approach
	if (!device_is_ready(led0.port)) {
		LOG_ERR("LED0 GPIO device not ready");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED0 GPIO: %d", ret);
		led1_blink_error();
		return 0;
	}

	LOG_INF("LED0 (Red) initialized successfully");
	
	// Turn off red LED initially
	gpio_pin_set_dt(&led0, 0);
	
	// LED Pattern: 2 quick blinks on BLUE LED = Both LEDs initialized
	for (int i = 0; i < 2; i++) {
		gpio_pin_set_dt(&led1, 1);
		k_sleep(K_MSEC(100));
		gpio_pin_set_dt(&led1, 0);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(500));

	if (!device_is_ready(lsm6dsl_dev)) {
		LOG_ERR("LSM6DSL sensor device not ready");
		led1_blink_error(); // Blink LED1 on error
		return 0;
	}

	LOG_INF("LSM6DSL sensor device ready");
	
	// LED Pattern: 3 quick blinks on BLUE LED = Sensor device ready
	for (int i = 0; i < 3; i++) {
		gpio_pin_set_dt(&led1, 1);
		k_sleep(K_MSEC(100));
		gpio_pin_set_dt(&led1, 0);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(500));

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for accelerometer");
		led1_blink_error(); // Blink LED1 on error
		return 0;
	}

	LOG_DBG("Accelerometer sampling frequency set to 104 Hz");

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for gyro");
		led1_blink_error(); // Blink LED1 on error
		return 0;
	}

	LOG_DBG("Gyroscope sampling frequency set to 104 Hz");
	
	// LED Pattern: 4 quick blinks on BLUE LED = Sensor frequency configured
	for (int i = 0; i < 4; i++) {
		gpio_pin_set_dt(&led1, 1);
		k_sleep(K_MSEC(100));
		gpio_pin_set_dt(&led1, 0);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(500));

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		LOG_ERR("Could not set sensor trigger");
		led1_blink_error(); // Blink LED1 on error
		return 0;
	}

	LOG_INF("Sensor trigger configured successfully");
	
	// LED Pattern: 5 quick blinks on BLUE LED = Trigger configured successfully
	for (int i = 0; i < 5; i++) {
		gpio_pin_set_dt(&led1, 1);
		k_sleep(K_MSEC(100));
		gpio_pin_set_dt(&led1, 0);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(500));
#endif

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		LOG_ERR("Sensor sample update error");
		led1_blink_error(); // Blink LED1 on error
		return 0;
	}

	LOG_INF("Initial sensor sample fetch successful");
	LOG_INF("Entering main sensor loop");
	
	// BLUE LED: Solid ON = Everything initialized successfully, entering main loop
	// RED LED: Will toggle on each sensor data interrupt
	gpio_pin_set_dt(&led1, 1);

	while (1) {
		/* Erase previous */
		printk("\0033\014");
		printf("LSM6DSL sensor samples:\n\n");

		/* lsm6dsl accel */
		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
							  sensor_value_to_double(&accel_x_out),
							  sensor_value_to_double(&accel_y_out),
							  sensor_value_to_double(&accel_z_out));
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
							   sensor_value_to_double(&gyro_x_out),
							   sensor_value_to_double(&gyro_y_out),
							   sensor_value_to_double(&gyro_z_out));
		printk("%s\n", out_str);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		/* lsm6dsl external magn */
		sprintf(out_str, "magn x:%f gauss y:%f gauss z:%f gauss",
							   sensor_value_to_double(&magn_x_out),
							   sensor_value_to_double(&magn_y_out),
							   sensor_value_to_double(&magn_z_out));
		printk("%s\n", out_str);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		/* lsm6dsl external press/temp */
		sprintf(out_str, "press: %f kPa - temp: %f deg",
			sensor_value_to_double(&press_out), sensor_value_to_double(&temp_out));
		printk("%s\n", out_str);
#endif

		printk("loop:%d trig_cnt:%d\n\n", ++cnt, lsm6dsl_trig_cnt);

		LOG_DBG("Main loop iteration %d, trigger count: %d", cnt, lsm6dsl_trig_cnt);

		print_samples = 1;
		k_sleep(K_MSEC(2000));
	}
}