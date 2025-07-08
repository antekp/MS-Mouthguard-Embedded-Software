#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define ICM_20948_I2C_ADDRESS  0x69
#define ACCEL_XOUT_H           0x2D
#define WHO_AM_I_REGISTER      0x00
#define CHIP_ID                0xEA
#define REG_BANK_SEL           0x7F
#define PWR_MGMT_1             0x06
#define ODR_ALIGN_EN		   0x09
#define GYRO_SMPLRT_DIV		   0x00
#define GYRO_CONFIG_1          0x01
#define ACCEL_SMPLRT_DIV_1     0x10
#define ACCEL_SMPLRT_DIV_2     0x11
#define ACCEL_CONFIG		   0x14

typedef enum
{
	_b0 = 0,
	_b1 = 1 << 4,
	_b2 = 2 << 4,
	_b3 = 3 << 4
} user_bank_select;

typedef struct
{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
} icm_20948_data;

void icm_20948_init(const struct i2c_dt_spec dev_i2c);
void select_register_bank(const struct i2c_dt_spec dev_i2c, user_bank_select bank_register_number);
void icm_20948_i2c_write_reg(user_bank_select bank_number, const struct i2c_dt_spec dev_i2c, uint8_t register, uint8_t data);
void icm_20948_read_data(const struct i2c_dt_spec dev_i2c, icm_20948_data* data);