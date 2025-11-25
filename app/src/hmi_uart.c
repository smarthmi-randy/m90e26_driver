#include "hmi_uart.h"
#include <string.h>

// 註冊日誌模組
LOG_MODULE_REGISTER(hmi_uart_module, LOG_LEVEL_DBG);

// 靜態函數聲明
static void hmi_uart_callback_internal(const struct device *dev, struct uart_event *evt, void *user_data);

// 內部 UART 回調函數
// 這個回調函數將處理所有由 Zephyr UART 驅動發出的事件，並將其分發到正確的 hmi_uart_data 實例。
static void hmi_uart_callback_internal(const struct device *dev, struct uart_event *evt, void *user_data)
{
    struct hmi_uart_data *data = (struct hmi_uart_data *)user_data;

    if (data == NULL) {
        LOG_ERR("UART callback received NULL user_data for %s!", dev->name);
        return;
    }

    switch (evt->type) {
        case UART_RX_RDY: {
            //接收完成的事件，在這邊處理接收資料以及傳送msgq到Application的邏輯
            //LOG_INF("%s: UART_RX_RDY:%X,%u", dev->name, data->rx_buf[data->rx_buf_pos], data->rx_buf_pos);
            if (data->rx_buf[data->rx_buf_pos] == '\n' || data->rx_buf[data->rx_buf_pos] == '\r')
            {
                data->rx_buf[data->rx_buf_pos + 1] = '\0';
                if (k_msgq_put(data->rx_msgq, data->rx_buf, K_NO_WAIT) != 0) {
                    LOG_WRN("%s: Failed to put message in RX queue (full?).", data->dev->name);
                }
                data->rx_buf_pos = 0;
                //LOG_INF("%s: UART_RX_RDY:Get newline or return:%s", dev->name, data->rx_buf);
                break;
            }
            data->rx_buf_pos++;
            if(data->rx_buf_pos > HMI_UART_RX_MSG_SIZE - 2)
            {
                data->rx_buf[data->rx_buf_pos] = '\0'; 
                if (k_msgq_put(data->rx_msgq, data->rx_buf, K_NO_WAIT) != 0) {
                    LOG_WRN("%s: Failed to put message in RX queue (full?).", data->dev->name);
                }
                data->rx_buf_pos = 0;
            }
            break;
        }
        case UART_RX_STOPPED: {
            //LOG_INF("%s: UART_RX_STOPPED", dev->name);
            break;
        }
        case UART_RX_BUF_REQUEST:
            // 驅動通常會自動請求下一個緩衝區，無需手動處理
            break;

        case UART_RX_BUF_RELEASED:
            // 緩衝區釋放事件，用於資源管理
            break;
        case UART_RX_DISABLED:
        {
            int ret = uart_rx_enable(dev, data->rx_buf + data->rx_buf_pos, 1, SYS_FOREVER_US);
            if (ret) {
                LOG_ERR("啟用 UART %s 接收失敗: %d", dev->name, ret);
            }
            //LOG_INF("%s: UART_RX_DISABLED", dev->name);
            break;
        }
        case UART_TX_DONE:
            // 傳輸完成事件。如果需要，可以在此處理 TX 完成的邏輯
            // 例如，釋放 TX 緩衝區或通知等待的線程
            LOG_INF("%s: UART_TX_DONE", dev->name);
            break;

        case UART_TX_ABORTED:
            //LOG_WRN("%s: UART_EVT_TX_ABORTED", dev->name);
            break;

        default:
            // 處理其他未知的 UART 事件
            break;
    }
}

int hmi_uart_init_instance(struct hmi_uart_data *instance_data, uint32_t baud_rate)
{
    const struct device *uart_dev = instance_data->dev;
    struct k_msgq *rx_msgq = instance_data->rx_msgq;
    struct uart_config uart_cfg = {
        .baudrate = baud_rate,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    int ret;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART 設備 %s 未準備就緒！", uart_dev->name);
        return -ENODEV;
    }

    if (instance_data == NULL || rx_msgq == NULL) {
        LOG_ERR("hmi_uart_init_instance 接收到 NULL 指針！");
        return -EINVAL;
    }

    // 將外部提供的數據結構和消息佇列指針賦值給實例
    instance_data->dev = uart_dev;
    instance_data->rx_msgq = rx_msgq;
    instance_data->rx_buf_pos = 0;

    // 清除消息隊列，移除任何舊消息
    k_msgq_purge(instance_data->rx_msgq);

    ret = uart_configure(uart_dev, &uart_cfg);
    if (ret) {
        LOG_ERR("配置 UART 設備 %s 失敗: %d", uart_dev->name, ret);
        return ret;
    }

    // 設置回調函數，將實例數據指針作為 user_data 傳遞
    ret = uart_callback_set(uart_dev, hmi_uart_callback_internal, (void *)instance_data);
    if (ret) {
        LOG_ERR("設定 UART %s 回調失敗: %d", uart_dev->name, ret);
        return ret;
    }

    // 啟用接收，使用實例的接收緩衝區
    ret = uart_rx_enable(uart_dev, instance_data->rx_buf, 1, SYS_FOREVER_US);
    if (ret) {
        LOG_ERR("啟用 UART %s 接收失敗: %d", uart_dev->name, ret);
        return ret;
    }

    LOG_INF("HMI UART 模組 %s 初始化成功，波特率: %d", uart_dev->name, baud_rate);
    return 0;
}

int hmi_uart_send(const struct device *uart_dev, const uint8_t *data, size_t len)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART 設備 %s 未準備就緒，無法傳送！", uart_dev->name);
        return -ENODEV;
    }

    int ret = uart_tx(uart_dev, data, len, SYS_FOREVER_US);
    if (ret < 0) {
        LOG_ERR("UART %s 寫入 FIFO 失敗: %d", uart_dev->name, ret);
    }
    return ret; // 返回
}