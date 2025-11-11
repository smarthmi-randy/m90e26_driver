/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include "nus_module.h"

LOG_MODULE_REGISTER(nus_module, LOG_LEVEL_INF);
#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)
#define BT_LOOPBACK_THREAD_STACK_SIZE 512
#define BT_LOOPBACK_THREAD_PRIORITY 5
#define MSGQ_MAX_LEN    (10)
// 宣告執行緒的堆疊 (stack) 空間
K_THREAD_STACK_DEFINE(g_bt_nus_loopback_thread_stack, BT_LOOPBACK_THREAD_STACK_SIZE);
// 宣告執行緒的控制區塊
static struct k_thread g_bt_nus_loopback_thread_thread;

struct k_msgq loopback_msgq;
uint8_t msgq_buf[CONFIG_BT_L2CAP_TX_MTU + 1];

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

int nus_module_loopback(void);

static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);

	LOG_INF("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	char message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	memcpy(message, data, MIN(sizeof(message) - 1, len));
	LOG_INF("%s() - Len: %d, Message: %s\n", __func__, len, message);
}

struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = received,
};

int nus_module_init(void)
{
	int err;

	LOG_INF("Sample - Bluetooth Peripheral NUS\n");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		LOG_ERR("Failed to register NUS callback: %d\n", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to start advertising: %d\n", err);
		return err;
	}

    k_msgq_init(&loopback_msgq, msgq_buf, CONFIG_BT_L2CAP_TX_MTU + 1, MSGQ_MAX_LEN);
    k_thread_create(&g_bt_nus_loopback_thread_thread, g_bt_nus_loopback_thread_stack,
			K_THREAD_STACK_SIZEOF(g_bt_nus_loopback_thread_stack),
			nus_module_loopback, // 執行緒函式
			NULL, NULL, NULL,        // 參數 (未使用)
			BT_LOOPBACK_THREAD_PRIORITY, // 優先權
			0, K_NO_WAIT);           // 選項 & 延遲
	LOG_INF("Initialization complete\n");

	return 0;
}

int nus_module_loopback(void)
{
    while (true) {
		const char local_buf[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

		if (k_msgq_get(&msgq_buf, local_buf, K_MSEC(100)) == 0)
        {
            int err = bt_nus_send(NULL, local_buf, strlen(local_buf));
		    LOG_INF("Data send - Result: %d\n", err);
            if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
			    LOG_ERR("BT loopback fail: %d", err);
		    }
        }
	}
}