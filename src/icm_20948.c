#include <zephyr/sys/printk.h>
#include "icm_20948.h"



uint8_t id = 0;
uint8_t regs[] = {WHO_AM_I_REGISTER};

void icm_20948_init(const struct i2c_dt_spec dev_i2c)
{

	icm_20948_i2c_write_reg(_b0, dev_i2c, PWR_MGMT_1, 0xC1);  //IMU reset, page 37

	k_msleep(100);

	icm_20948_i2c_write_reg(_b0, dev_i2c, PWR_MGMT_1, 0x01);  //Exit from sleep mode, selecting clock 37, page 37
	icm_20948_i2c_write_reg(_b2, dev_i2c, ODR_ALIGN_EN, 0x01); //Output data rate start time aligment, page 63

	//Gyroscope configuration 
	icm_20948_i2c_write_reg(_b2, dev_i2c, GYRO_SMPLRT_DIV, 0x00); //Sample rate devider set to 0, page 59
	icm_20948_i2c_write_reg(_b2, dev_i2c, GYRO_CONFIG_1, 0x05); //Enable gyro DLPF and set +- 1000 dps 

	//Accelerometer configuration
	icm_20948_i2c_write_reg(_b2, dev_i2c, ACCEL_SMPLRT_DIV_1, 0x00); //Sample rate devider = 0, page 63
	icm_20948_i2c_write_reg(_b2, dev_i2c, ACCEL_SMPLRT_DIV_2, 0x00);
	icm_20948_i2c_write_reg(_b2, dev_i2c, ACCEL_CONFIG, 0x03); //Enable acc DLPF and set +- 4g

	select_register_bank(dev_i2c, _b0);
}

void icm_20948_read_data(const struct i2c_dt_spec dev_i2c, icm_20948_data *data)
{
	uint8_t data_rx[12];

	//Reading 12 bytes from data registers
	int i2c_write_error = i2c_burst_read_dt(&dev_i2c, ACCEL_XOUT_H, data_rx, 12);
	if(i2c_write_error != 0)
	{
		printk("Failed to read to register 0x%02X\r\n", ACCEL_XOUT_H);
	}
	//Encapsulating data
	data->x_accel = ((int16_t)data_rx[0] << 8) + data_rx[1];
	data->y_accel = ((int16_t)data_rx[2] << 8) + data_rx[3];
	data->z_accel = ((int16_t)data_rx[4] << 8) + data_rx[5];

	data->x_gyro  = ((int16_t)data_rx[6] << 8) + data_rx[7];
	data->y_gyro  = ((int16_t)data_rx[8] << 8) + data_rx[9];
	data->z_gyro  = ((int16_t)data_rx[10] << 8) + data_rx[11];

	printk("Accelerometer: %d, %d, %d \r\n", data-> x_accel, data-> y_accel, data -> z_accel);
	printk("Gyroscope: %d, %d, %d \r\n", data-> x_gyro, data-> y_gyro, data -> z_gyro);
}

void icm_20948_i2c_write_reg(user_bank_select bank_number, const struct i2c_dt_spec dev_i2c, uint8_t reg, uint8_t data)
{
	select_register_bank(dev_i2c, bank_number);
	k_msleep(100);
	
	int i2c_write_error = i2c_burst_write_dt(&dev_i2c, reg, &data, 1);
	if(i2c_write_error != 0)
	{
		printk("Failed to write to register 0x%02X\r\n", reg);
	}
}

void select_register_bank(const struct i2c_dt_spec dev_i2c, user_bank_select bank_register_number)
{
	int bank_num_write_error = i2c_burst_write_dt(&dev_i2c, REG_BANK_SEL, &bank_register_number, 1);
	if(bank_num_write_error != 0)
	{
		printk("Failed to write to register 0x%02X\r\n", REG_BANK_SEL);
	}
}
int8_t i2c_bus_start_check(const struct i2c_dt_spec *dev_i2c)
{
	if (!device_is_ready(dev_i2c->bus)) {
	printk("I2C bus %s is not ready!\n\r",dev_i2c->bus->name);
	return -1;
	}
	int ret = i2c_write_read_dt(dev_i2c, regs, 1, &id, 1);

	if(ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
		return -1;
	}

	if (id != CHIP_ID) {
		printk("Invalid chip id! 0x%02X \n", id);
		return -1;
	}
	return 0;
}