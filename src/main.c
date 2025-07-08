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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/lbs.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>
#include "icm_20948.h"

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define SLEEP_TIME_MS 1000
#define I2C_NODE DT_NODELABEL(mysensor)

/* Custom 128-bit UUIDs */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_CUSTOM_CHAR_VAL \
	BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x6789, 0x1234, 0x56789abcdef0)


static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 custom_char_uuid    = BT_UUID_INIT_128(BT_UUID_CUSTOM_CHAR_VAL);

static struct bt_conn *notify_conn;
static uint8_t notify_enabled;

static struct k_work adv_work;

uint8_t id = 0;
uint8_t regs[] = {WHO_AM_I_REGISTER};
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

icm_20948_data imu_data;

K_FIFO_DEFINE(sensor_fifo);

//Callback for subscription following
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}
//Defining the GATT service and characteristic
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),
	BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (!err) {
		notify_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (notify_conn) {
		bt_conn_unref(notify_conn);
		notify_conn = NULL;
	}
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

static void recycled_cb(void)
{
	printk("Connection object available from previous conn. Disconnect is complete!\n");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};


//Thread for sensor data notification
void imu_reader_thread(void)
{
	while (1) {
		sensor_data_t *entry = k_malloc(sizeof(sensor_data_t));
		if (!entry) continue;

		icm_20948_read_data(dev_i2c, (icm_20948_data *)entry);

		entry->x_accel = imu_data.x_accel;
		entry->y_accel = imu_data.y_accel;
		entry->z_accel = imu_data.z_accel;
		entry->x_gyro  = imu_data.x_gyro;
		entry->y_gyro  = imu_data.y_gyro;
		entry->z_gyro  = imu_data.z_gyro;

		k_fifo_put(&sensor_fifo, entry);
		k_msleep(50); 
	}
}
//Thread for data transmition over BLE
void ble_sender_thread(void)
{
	while (1) {
		sensor_data_t *entry = k_fifo_get(&sensor_fifo, K_FOREVER);

		if (notify_conn && notify_enabled) {
			bt_gatt_notify(notify_conn, &custom_svc.attrs[1],
			               entry, sizeof(sensor_data_t));
		}

		k_free(entry);
	}
}

K_THREAD_DEFINE(imu_thread_id, 1024, imu_reader_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ble_thread_id, 1024, ble_sender_thread, NULL, NULL, NULL, 5, 0, 0);


int main(void)
{
	int err;
	
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

	printk("Starting Bluetooth Peripheral LBS sample\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		k_sleep(K_FOREVER);
	}
}
