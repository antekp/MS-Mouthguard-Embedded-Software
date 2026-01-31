#include <zephyr/drivers/uart.h>

int8_t DS18B20_Start(const struct device *uart);
void DS18B20_Write(uint8_t data, const struct device *uart);
int8_t UartConfigureResetBaudRate(const struct device *uart);
int8_t UartConfigureDataBaudRate(const struct device *uart);

