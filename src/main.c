/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include "icm_20948.h"
#include "ds18b20_1wire.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>
#include "tds_sensor_adc.h"

//Private defines section
#define SLEEP_TIME_MS 1000
#define I2C_NODE DT_NODELABEL(mysensor)

//DeviceTree variables section
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

//Data variables section
icm_20948_data imu_data;
int16_t adc_buffer;
struct adc_sequence sequence = {
	.buffer = &adc_buffer,
	.buffer_size = sizeof(adc_buffer)
};

int main(void)
{
	
	i2c_bus_start_check(&dev_i2c);

	if (!device_is_ready(uart))
	{
		printk("UART: %c not ready", (int)uart->name);
		return -1;
	}
	icm_20948_init(dev_i2c);
	
	for (;;) {

		k_msleep(SLEEP_TIME_MS);
	}
}

