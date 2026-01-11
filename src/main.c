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

int DS18B20_Start(void);
void uart0_rx_cplt_callback(const struct device *dev, struct uart_event *evt, void *user_data);
uint8_t id = 0;
uint8_t regs[] = {WHO_AM_I_REGISTER};
uint8_t RxData[8];
volatile uint8_t isRxed = 0;
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

const struct uart_config uart_cfg_reset = {
		.baudrate = 9600,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

const struct uart_config uart_cfg_data = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};


icm_20948_data imu_data;

uint8_t tx_buffer = 0xF0;


int main(void)
{
	int8_t err = uart_configure(uart, &uart_cfg_reset);
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

	uart_callback_set(uart, uart0_rx_cplt_callback, NULL); //Complete RX callback


	for (;;) {

		/*
		DS18B20_Start();
		DS18B20_Write(0xCC); //Skip ROM
		DS18B20_Write(0x44); //convert t

		DS18B20_Start();
		DS18B20_Write(0xCC); //Skip ROM
		DS18B20_Write(0xBE); //Read Scratchpad 
		
		Enable this part after pluing DS12B20 sensor
		*/ 

		k_msleep(SLEEP_TIME_MS);
	}
}

int DS18B20_Start(void)
{
	uint8_t data = 0xF0; //Reset condition
	uart_configure(uart, &uart_cfg_reset); //9600 data transmission
	uart_tx(uart, &data, 1, 1000);
	if(uart_rx_enable(uart, &data, 1, 1000) != 0)
	{
		return -1; // API not enabled or RX already in progress or other negative errno value in case of failure.
	}
	uart_configure(uart, &uart_cfg_data); //115200 data transmission
	if(data == 0xF0)
	{
		return -2; //no device detected
	}
	return 1;
}
void DS18B20_Write(uint8_t data)
{
	uint8_t buffer[8];
	//preparing buffer for UART transmission, each bit from data variable is put to buffer when each index represents each bit.
	for(uint8_t i = 0; i<8; i++)
	{
		if((data & (1<<i)) != 0)
		{
			buffer[i] = 1; //set HIGH
		}
		else
		{
			buffer[i] = 0; //set LOW 
		}
	}
	uart_tx(uart, buffer, 8, 1000);
}
uint8_t DS18B20_Read(void)
{
	uint8_t buffer[8];
	uint8_t value = 0;
	for(uint8_t i = 0; i<8; i++)
	{
		buffer[i] = 0xFF;
	}
	uart_tx(uart, buffer, 8, 1000);
	uart_rx_enable(uart, RxData, 1, 1000);

	while(isRxed == 0);

	for(uint8_t i = 0; i<8; i++)
	{
		if(RxData[i] == 0xFF)//bit is 1
		{
			value |= (1<<i);
		} 

	}
	isRxed = 0; //disable flag
	return value;
}
void uart0_rx_cplt_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch(evt->type)
	{
		case UART_RX_RDY:
			isRxed = 1;
		break;

		default:
		break;
	}
}