#ifndef HMI_UART_H__
#define HMI_UART_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

// UART 接收緩衝區大小
#define HMI_UART_RX_MSG_SIZE 64 // 每個接收消息的最大長度 (包含 null 終止符)

/**
 * @brief HMI UART 實例的數據結構。
 * 包含每個 UART 實例的運行時數據。
 */
struct hmi_uart_data {
    const struct device *dev;         // UART 設備指針
    uint8_t rx_buf[HMI_UART_RX_MSG_SIZE]; // 接收緩衝區
    size_t rx_buf_pos;                // 接收緩衝區當前位置
    struct k_msgq *rx_msgq;           // 綁定到此實例的接收消息隊列指針
};

/**
 * @brief 初始化 HMI UART 模組的特定實例。
 *
 * @param instance_data 指向 HMI UART 數據結構的指針，用於儲存實例的運行時數據。
 * @param baud_rate 要設置的 UART 波特率。
 * 此數據結構和其內部消息佇列應由外部調用者定義和管理。
 *
 * @return 回傳傳送成功或失敗，0為成功，負數 errno 表示失敗。
 */
int hmi_uart_init_instance(struct hmi_uart_data *instance_data, uint32_t baud_rate);

/**
 * @brief 向指定的 HMI UART 設備發送數據 (非阻塞)。
 *
 * @param uart_dev 指向要發送數據的 UART 設備的指針。
 * @param data 指向要發送數據的緩衝區的指針。
 * @param len 要發送的數據長度。
 *
 * @return 回傳傳送成功或失敗，0為成功，負數 errno 表示失敗。
 */
int hmi_uart_send(const struct device *uart_dev, const uint8_t *data, size_t len);

#endif // HMI_UART_H__