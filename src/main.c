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
#include "icm_20948.h"
#include <zephyr/drivers/uart.h>


#define SLEEP_TIME_MS 1000
#define I2C_NODE DT_NODELABEL(mysensor)


uint8_t id = 0;
uint8_t regs[] = {WHO_AM_I_REGISTER};
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};


icm_20948_data imu_data;

uint8_t tx_buffer[8] = {1,2,3,4,5,6,7,8};


int main(void)
{
	int8_t err = uart_configure(uart, &uart_cfg);
	if(err == -ENOSYS)
	{
		printk("Configuration not supported");
	}
	i2c_bus_start_check(&dev_i2c, regs, id);

	if (!device_is_ready(uart))
	{
		printk("UART: %c not ready", (int)uart->name);
		return -1;
	}

	icm_20948_init(dev_i2c);


	for (;;) {

		uart_tx(uart, tx_buffer, sizeof(tx_buffer), 1000);
		//icm_20948_read_data(dev_i2c,&imu_data);


		k_msleep(SLEEP_TIME_MS);
	}
}

