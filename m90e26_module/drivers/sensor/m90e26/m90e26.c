/*
 * Copyright (c) 2023, Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_m90e26

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "m90e26.h"

LOG_MODULE_REGISTER(M90E26, CONFIG_SENSOR_LOG_LEVEL);

/* M90E26 Register Addresses */
#define M90E26_REG_SOFTRESET    0x00
#define M90E26_REG_CALSTART     0x20
#define M90E26_REG_ADJSTART     0x30
#define M90E26_REG_IRMS         0x48
#define M90E26_REG_URMS         0x49
#define M90E26_REG_PMEAN        0x4A
#define M90E26_REG_QMEAN        0x4B
#define M90E26_REG_FREQ         0x4C
#define M90E26_REG_POWERF       0x4D
#define M90E26_REG_SMEAN        0x4F

/* UART protocol constants */
#define M90E26_UART_START_BYTE  0xFE
#define M90E26_UART_READ_BIT    0x80
#define READ_RESP_LEN           3 /* DATA_MSB, DATA_LSB, CHKSUM */
#define WRITE_RESP_LEN          1 /* CHKSUM */
#define MSGQ_MAX_LEN            MAX(READ_RESP_LEN, WRITE_RESP_LEN)

struct m90e26_data {
    uint16_t urms;
    uint16_t irms;
    int16_t pmean;
    int16_t qmean;
    uint16_t smean;
    uint16_t freq;
    int16_t powerf;

    /* UART specific data */
    struct k_msgq rx_msgq;
    uint8_t msgq_buf[MSGQ_MAX_LEN];
    uint8_t rx_buf[MSGQ_MAX_LEN];
    uint8_t rx_buf_pos;
    uint8_t expected_len;
};

struct m90e26_config {
    const struct device *uart;
};

static const struct uart_config m90e26_uart_cfg = {
    .baudrate = 9600,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

static uint8_t m90e26_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

static void uart_cb_handler(const struct device *dev, void *user_data)
{
    const struct device *sensor = user_data;
    struct m90e26_data *data = sensor->data;
    int ret;

    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
        return;
    }

    while (data->rx_buf_pos < data->expected_len) {
        ret = uart_fifo_read(dev, &data->rx_buf[data->rx_buf_pos], 1);
        if (ret == 0) {
            break; // No more data in FIFO
        }
        data->rx_buf_pos++;
    }

    if (data->rx_buf_pos >= data->expected_len) {
        k_msgq_put(&data->rx_msgq, data->rx_buf, K_NO_WAIT);
        data->rx_buf_pos = 0;
        uart_irq_rx_disable(dev);
    }
}

static int m90e26_reg_read(const struct device *dev, uint8_t reg_addr, uint16_t *val)
{
    const struct m90e26_config *config = dev->config;
    struct m90e26_data *data = dev->data;
    uint8_t tx_buf[3];
    uint8_t rx_buf[READ_RESP_LEN];

    tx_buf[0] = M90E26_UART_START_BYTE;
    tx_buf[1] = reg_addr | M90E26_UART_READ_BIT;
    tx_buf[2] = tx_buf[1]; // Checksum for read is just the address byte [cite: 531]

    data->expected_len = READ_RESP_LEN;
    data->rx_buf_pos = 0;
    k_msgq_purge(&data->rx_msgq);
    uart_irq_rx_enable(config->uart);

    for (int i = 0; i < sizeof(tx_buf); i++) {
        uart_poll_out(config->uart, tx_buf[i]);
    }

    if (k_msgq_get(&data->rx_msgq, rx_buf, K_MSEC(100)) != 0) {
        uart_irq_rx_disable(config->uart);
        return -ETIMEDOUT;
    }

    uint8_t expected_checksum = m90e26_checksum(rx_buf, 2); // Checksum is sum of DATA_MSB and DATA_LSB [cite: 532]
    if (rx_buf[2] != expected_checksum) {
        LOG_ERR("Read checksum error. Got %02x, expected %02x", rx_buf[2], expected_checksum);
        return -EIO;
    }

    *val = sys_get_be16(rx_buf);
    return 0;
}

static int m90e26_reg_write(const struct device *dev, uint8_t reg_addr, uint16_t val)
{
    const struct m90e26_config *config = dev->config;
    struct m90e26_data *data = dev->data;
    uint8_t tx_buf[5];
    uint8_t rx_buf[WRITE_RESP_LEN];

    tx_buf[0] = M90E26_UART_START_BYTE;
    tx_buf[1] = reg_addr & ~M90E26_UART_READ_BIT;
    sys_put_be16(val, &tx_buf[2]);
    tx_buf[4] = m90e26_checksum(&tx_buf[1], 3); // Checksum for write [cite: 529]

    data->expected_len = WRITE_RESP_LEN;
    data->rx_buf_pos = 0;
    k_msgq_purge(&data->rx_msgq);
    uart_irq_rx_enable(config->uart);

    for (int i = 0; i < sizeof(tx_buf); i++) {
        uart_poll_out(config->uart, tx_buf[i]);
    }

    if (k_msgq_get(&data->rx_msgq, rx_buf, K_MSEC(100)) != 0) {
        uart_irq_rx_disable(config->uart);
        return -ETIMEDOUT;
    }

    if (rx_buf[0] != tx_buf[4]) {
        LOG_ERR("Write checksum error. Got %02x, expected %02x", rx_buf[0], tx_buf[4]);
        return -EIO;
    }
    
    return 0;
}


static int m90e26_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct m90e26_data *data = dev->data;
    int ret = 0;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_WRN("Unsupported channel %d", chan);
        return -ENOTSUP;
    }

    ret |= m90e26_reg_read(dev, M90E26_REG_URMS, &data->urms);
    ret |= m90e26_reg_read(dev, M90E26_REG_IRMS, &data->irms);
    ret |= m90e26_reg_read(dev, M90E26_REG_PMEAN, (uint16_t *)&data->pmean);
    ret |= m90e26_reg_read(dev, M90E26_REG_QMEAN, (uint16_t *)&data->qmean);
    ret |= m90e26_reg_read(dev, M90E26_REG_SMEAN, &data->smean);
    ret |= m90e26_reg_read(dev, M90E26_REG_FREQ, &data->freq);
    ret |= m90e26_reg_read(dev, M90E26_REG_POWERF, (uint16_t *)&data->powerf);

    return ret;
}

static void m90e26_convert_and_set(struct sensor_value *val, int32_t raw_val, int scale)
{
    val->val1 = raw_val / scale;
    val->val2 = (raw_val % scale) * (1000000 / scale);
}

static int m90e26_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct m90e26_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_VOLTAGE:
        m90e26_convert_and_set(val, data->urms, 100);
        break;
    case SENSOR_CHAN_CURRENT:
        m90e26_convert_and_set(val, data->irms, 1000);
        break;
    case SENSOR_CHAN_M90E26_ACTIVE_POWER:
        m90e26_convert_and_set(val, data->pmean, 1000);
        break;
    case SENSOR_CHAN_M90E26_REACTIVE_POWER:
        m90e26_convert_and_set(val, data->qmean, 1000);
        break;
    case SENSOR_CHAN_M90E26_APPARENT_POWER:
        m90e26_convert_and_set(val, data->smean, 1000);
        break;
    // case SENSOR_CHAN_FREQ:
    //     m90e26_convert_and_set(val, data->freq, 100);
    //     break;
    case SENSOR_CHAN_M90E26_POWER_FACTOR:
        m90e26_convert_and_set(val, data->powerf, 1000);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}


static const struct sensor_driver_api m90e26_api = {
    .sample_fetch = m90e26_sample_fetch,
    .channel_get = m90e26_channel_get,
};

static int m90e26_init(const struct device *dev)
{
    const struct m90e26_config *config = dev->config;
    struct m90e26_data *data = dev->data;
    int ret;

    if (!device_is_ready(config->uart)) {
        LOG_ERR("UART bus is not ready");
        return -ENODEV;
    }
    
    ret = uart_configure(config->uart, &m90e26_uart_cfg);
    if (ret < 0) {
        LOG_ERR("Could not configure UART");
        return ret;
    }

    k_msgq_init(&data->rx_msgq, data->msgq_buf, MSGQ_MAX_LEN, 1);
    
    uart_irq_callback_user_data_set(config->uart, uart_cb_handler, (void *)dev);

    /* Software Reset */
    ret = m90e26_reg_write(dev, M90E26_REG_SOFTRESET, 0x789A);
    if (ret < 0) {
        LOG_ERR("Failed to reset device");
        return ret;
    }
    k_sleep(K_MSEC(20)); // Wait for reset and UART timeout [cite: 463]

    /* Start metering and measurement calibration */
    ret = m90e26_reg_write(dev, M90E26_REG_CALSTART, 0x5678);
    if (ret < 0) {
        LOG_ERR("Failed to start metering calibration");
        return ret;
    }
    k_sleep(K_MSEC(20));

    ret = m90e26_reg_write(dev, M90E26_REG_ADJSTART, 0x5678);
    if (ret < 0) {
        LOG_ERR("Failed to start measurement calibration");
        return ret;
    }
    k_sleep(K_MSEC(20));

    LOG_INF("M90E26 driver initialized via UART");

    return 0;
}

#define M90E26_DEFINE(inst)                                           \
    static struct m90e26_data m90e26_data_##inst;                     \
                                                                      \
    static const struct m90e26_config m90e26_config_##inst = {         \
        .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                      \
    };                                                                \
                                                                      \
    DEVICE_DT_INST_DEFINE(inst, &m90e26_init, NULL,                    \
                          &m90e26_data_##inst, &m90e26_config_##inst,  \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
                          &m90e26_api);

DT_INST_FOREACH_STATUS_OKAY(M90E26_DEFINE)