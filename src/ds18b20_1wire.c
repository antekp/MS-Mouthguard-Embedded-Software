#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include "ds18b20_1wire.h"


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

int8_t DS18B20_Start(const struct device *uart)
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
void DS18B20_Write(uint8_t data, const struct device *uart)
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
int8_t UartConfigureResetBaudRate(const struct device *uart)
{
    int8_t err = uart_configure(uart, &uart_cfg_reset);
    if(err == errno)
    {
        printk("Failure");
        return errno;
    }
    if(err == ENOSYS)
    {
        printk("configuration is not supported by device or driver does not support setting configuration in runtime");
        return ENOSYS;
    }
    if(err == ENOTSUP)
    {
        printk("API is not enabled");
        return ENOTSUP;
    }
    return 0;
} 

int8_t UartConfigureDataBaudRate(const struct device *uart)
{
    int8_t err = uart_configure(uart, &uart_cfg_data);
    if(err == errno)
    {
        printk("Failure");
        return errno;
    }
    if(err == ENOSYS)
    {
        printk("configuration is not supported by device or driver does not support setting configuration in runtime");
        return ENOSYS;
    }
    if(err == ENOTSUP)
    {
        printk("API is not enabled");
        return ENOTSUP;
    }
    return 0;
} 
