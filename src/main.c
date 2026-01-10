/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/settings/settings.h>
#include "icm_20948.h"


#define SLEEP_TIME_MS 1000
#define I2C_NODE DT_NODELABEL(mysensor)


uint8_t id = 0;
uint8_t regs[] = {WHO_AM_I_REGISTER};
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

icm_20948_data imu_data;


int main(void)
{
	
	
	if (!device_is_ready(dev_i2c.bus)) {
	printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
	return -1;
	}
	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);

	if(ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
		return -1;
	}

	if (id != CHIP_ID) {
		printk("Invalid chip id! 0x%02X \n", id);
		return -1;
	}

	icm_20948_init(dev_i2c);


	for (;;) {

		icm_20948_read_data(dev_i2c,&imu_data);

		k_msleep(SLEEP_TIME_MS);
	}
}
