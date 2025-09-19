#define DT_DRV_COMPAT atmel_m90e26

#include "m90e26_emul.h"
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/uart_emul.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(m90e26_emul, CONFIG_LOG_DEFAULT_LEVEL);

// 從 m90e26_emul.h 移到此處，因為這是內部定義
#define REG_SoftReset   0x00
#define REG_SysStatus   0x01
#define REG_FuncEn      0x02
#define REG_SagTh       0x03
#define REG_LastData    0x06
#define REG_CalStart    0x20
#define REG_MMode       0x2B
#define REG_AdjStart    0x30
#define REG_APenergy    0x40
#define REG_Irms        0x48
#define REG_Urms        0x49
#define REG_Pmean       0x4A
#define REG_Qmean       0x4B
#define REG_Freq        0x4C
#define REG_PowerF      0x4D
#define REG_Irms2       0x68

typedef struct {
    uint16_t SoftReset;   uint16_t SysStatus;   uint16_t FuncEn;
    uint16_t SagTh;       uint16_t _rsvd1[2];   uint16_t LastData;
    uint16_t _rsvd2[25];
    uint16_t CalStart;    uint16_t _rsvd3[10];  uint16_t MMode;
    uint16_t _rsvd4[3];
    uint16_t AdjStart;    uint16_t _rsvd5[15];  uint16_t APenergy;
    uint16_t _rsvd6[7];
    uint16_t Irms;        uint16_t Urms;        uint16_t Pmean;
    uint16_t Qmean;      uint16_t Freq;        uint16_t PowerF;
    uint16_t _rsvd8[32];
    uint16_t Irms2;
} m90e26_registers_t;

struct m90e26_emul_data {
    m90e26_registers_t regs;
    enum {
        UART_STATE_IDLE, UART_STATE_WAIT_ADDR, UART_STATE_WAIT_DATA_MSB,
        UART_STATE_WAIT_DATA_LSB, UART_STATE_WAIT_CHKSUM
    } uart_rx_state;
    uint8_t uart_rx_buffer[5];
    uint8_t uart_rx_idx;
    uint8_t uart_tx_buffer[3]; // 模擬器的TX FIFO
    size_t uart_tx_len;
    bool respond_to_requests; // 新增旗標
};

static void emul_reset(struct m90e26_emul_data *data);
static uint16_t* get_reg_ptr(struct m90e26_emul_data* data, uint8_t addr);
static void emul_reset(struct m90e26_emul_data *data);
static void process_uart_write(const struct device *dev, struct m90e26_emul_data *data);
static void process_uart_read(const struct device *dev, struct m90e26_emul_data *data);
static void m90e26_emul_uart_receive_byte(const struct device *dev, struct m90e26_emul_data *data, uint8_t byte_from_host);
static int m90e26_emul_init(const struct emul *target);

/* Our dummy device echoes all data received */
static void uart_dummy_emul_tx_ready(const struct device *dev, size_t size,
				     const struct emul *target)
{
	uint32_t ret;
	uint8_t byte;

	for (size_t i = 0; i < size; ++i) {
		ret = uart_emul_get_tx_data(dev, &byte, 1);
            //LOG_WRN("Test: %s, size: %u, data: 0x%02X", __FUNCTION__, size, byte);
		//ret = uart_emul_put_rx_data(dev, &byte, 1);
        m90e26_emul_uart_receive_byte(dev, target->data, byte);
	}
    //LOG_WRN("End: %s", __FUNCTION__);


}

static const struct uart_emul_device_api m90e26_emul_uart_api = {
	.tx_data_ready = uart_dummy_emul_tx_ready,
};

int m90e26_emul_set_register(const struct emul *target, uint8_t addr, uint16_t value)
{
    struct m90e26_emul_data *data = target->data;
    
    // 軟體重置是一個特殊情況，它會重置整個模擬器狀態
    if (addr == REG_SoftReset && value == 0x789A) {
        LOG_INF("Emulator reset via test API");
        emul_reset(data);
        return 0;
    }

    uint16_t* reg_ptr = get_reg_ptr(data, addr);
    if (reg_ptr) {
        *reg_ptr = value;
        LOG_DBG("Set register 0x%02X to 0x%04X via test API", addr, value);
        return 0;
    }

    LOG_ERR("Cannot set unknown register 0x%02X", addr);
    return -EINVAL;
}

// 實作新的 API 函式
int m90e26_emul_set_response_mode(const struct emul *target, bool respond)
{
    struct m90e26_emul_data *data = target->data;
    data->respond_to_requests = respond;
    LOG_INF("Emulator response mode set to: %s", respond ? "ON" : "OFF");
    return 0;
}

// 內部實作函式 (保持不變或稍作修改)
static uint16_t* get_reg_ptr(struct m90e26_emul_data* data, uint8_t addr) {
    switch (addr) {
        case REG_SysStatus: return &data->regs.SysStatus;
        case REG_FuncEn:    return &data->regs.FuncEn;
        case REG_SagTh:     return &data->regs.SagTh;
        case REG_LastData:  return &data->regs.LastData;
        case REG_CalStart:  return &data->regs.CalStart;
        case REG_MMode:     return &data->regs.MMode;
        case REG_AdjStart:  return &data->regs.AdjStart;
        case REG_APenergy:  return &data->regs.APenergy;
        case REG_Irms:      return &data->regs.Irms;
        case REG_Urms:      return &data->regs.Urms;
        case REG_Pmean:     return &data->regs.Pmean;
        case REG_Qmean:     return &data->regs.Qmean;
        case REG_Freq:      return &data->regs.Freq;
        case REG_PowerF:    return &data->regs.PowerF;
        case REG_Irms2:     return &data->regs.Irms2;
        default: return NULL;
    }
}

static void emul_reset(struct m90e26_emul_data *data) {
    memset(&data->regs, 0, sizeof(m90e26_registers_t));
    data->regs.FuncEn = 0x000C; data->regs.SagTh = 0x1D6A;
    data->regs.CalStart = 0x6886; data->regs.MMode = 0x9422;
    data->regs.AdjStart = 0x6886; data->regs.Freq = 5000;
    data->uart_rx_state = UART_STATE_IDLE; data->uart_rx_idx = 0;
    data->uart_tx_len = 0;
    data->respond_to_requests = true; // 預設為正常回應
}

static void process_uart_write(const struct device *dev, struct m90e26_emul_data *data) {
    uint8_t addr = data->uart_rx_buffer[1] & 0x7F;
    uint16_t write_val = ((uint16_t)data->uart_rx_buffer[2] << 8) | data->uart_rx_buffer[3];
    int ret = 0;
    LOG_INF("UART WRITE addr=0x%02X, val=0x%04X", addr, write_val);
    
    if (!data->respond_to_requests) {
        data->uart_tx_len = 0; // 不產生任何回應
        return;
    }

    data->regs.LastData = write_val;
    if (addr == REG_SoftReset && write_val == 0x789A) {
        LOG_INF("Software Reset triggered!");
        emul_reset(data);
    }else
    {
        uint16_t* reg_ptr = get_reg_ptr(data, addr);
        if (reg_ptr) { *reg_ptr = write_val; }
        else { LOG_WRN("Write to unsupported addr 0x%02X", addr); }
    }

    data->uart_tx_buffer[0] = addr + data->uart_rx_buffer[2] + data->uart_rx_buffer[3];
    data->uart_tx_len = 1;
    ret = uart_emul_put_rx_data(dev, data->uart_tx_buffer, 1);
    LOG_INF("Put data to rx data 0x%02X, ret: %d", data->uart_tx_buffer[0], ret);
    
}

static void process_uart_read(const struct device *dev, struct m90e26_emul_data *data) {
    uint8_t addr = data->uart_rx_buffer[1] & 0x7F;
    LOG_INF("UART READ addr=0x%02X", addr);
    uint16_t read_val = 0;
    uint16_t* reg_ptr = get_reg_ptr(data, addr);

    if (!data->respond_to_requests) {
        data->uart_tx_len = 0; // 不產生任何回應
        return;
    }

    if (reg_ptr) { read_val = *reg_ptr; }
    else { LOG_WRN("Read from unsupported addr 0x%02X", addr); }
    data->regs.LastData = read_val;
    data->uart_tx_buffer[0] = (read_val >> 8) & 0xFF; // MSB
    data->uart_tx_buffer[1] = read_val & 0xFF;        // LSB
    data->uart_tx_buffer[2] = data->uart_tx_buffer[0] + data->uart_tx_buffer[1]; // Checksum
    data->uart_tx_len = 3;
    if (addr == REG_SysStatus || addr == REG_APenergy) {
        if(reg_ptr) *reg_ptr = 0;
    }
    LOG_INF("Read: %u, Put data to rx data 0x%02X, 0x%02X, 0x%02X", read_val, data->uart_tx_buffer[0], data->uart_tx_buffer[1], data->uart_tx_buffer[2]);
    uart_emul_put_rx_data(dev, data->uart_tx_buffer, 3);
}

static void m90e26_emul_uart_receive_byte(const struct device *dev, struct m90e26_emul_data *data, uint8_t byte_from_host) {
    if (data->uart_rx_state == UART_STATE_IDLE) {
        if (byte_from_host == 0xFE) {
            data->uart_rx_buffer[0] = byte_from_host;
            data->uart_rx_idx = 1;
            data->uart_rx_state = UART_STATE_WAIT_ADDR;
        } return;
    }
    data->uart_rx_buffer[data->uart_rx_idx++] = byte_from_host;
    bool is_read = (data->uart_rx_buffer[1] & 0x80) != 0;
    switch (data->uart_rx_state) {
    case UART_STATE_WAIT_ADDR:
        data->uart_rx_state = is_read ? UART_STATE_WAIT_CHKSUM : UART_STATE_WAIT_DATA_MSB; break;
    case UART_STATE_WAIT_DATA_MSB: data->uart_rx_state = UART_STATE_WAIT_DATA_LSB; break;
    case UART_STATE_WAIT_DATA_LSB: data->uart_rx_state = UART_STATE_WAIT_CHKSUM; break;
    case UART_STATE_WAIT_CHKSUM: {
        uint8_t received_chk = data->uart_rx_buffer[data->uart_rx_idx - 1];
        uint8_t calculated_chk = 0;
        if (is_read) {
            calculated_chk = data->uart_rx_buffer[1];
            if (received_chk == calculated_chk) { process_uart_read(dev, data); }
            else { LOG_ERR("Read checksum mismatch! Got 0x%02X, expected 0x%02X", received_chk, calculated_chk); }
        } else {
            calculated_chk = data->uart_rx_buffer[1] + data->uart_rx_buffer[2] + data->uart_rx_buffer[3];
            if (received_chk == calculated_chk) {
                process_uart_write(dev, data);
                data->uart_tx_buffer[0] = calculated_chk; data->uart_tx_len = 1;
            } else { LOG_ERR("Write checksum mismatch! Got 0x%02X, expected 0x%02X", received_chk, calculated_chk); }
        }
        data->uart_rx_state = UART_STATE_IDLE; data->uart_rx_idx = 0;
    } break;
    default: data->uart_rx_state = UART_STATE_IDLE; data->uart_rx_idx = 0; break;
    }
}

// 公開 API 的實作
int m90e26_emul_host_uart_write(const struct emul *target, const uint8_t *tx_data, size_t tx_len) {
    struct m90e26_emul_data *data = target->data;
    data->uart_tx_len = 0; // 清除舊的回應
    for (size_t i = 0; i < tx_len; i++) {
        m90e26_emul_uart_receive_byte(NULL, data, tx_data[i]);
    }
    return 0;
}

int m90e26_emul_host_uart_read(const struct emul *target, uint8_t *rx_buf, size_t rx_buf_len) {
    struct m90e26_emul_data *data = target->data;
    if (data->uart_tx_len == 0) { return 0; }
    size_t len_to_copy = MIN(data->uart_tx_len, rx_buf_len);
    memcpy(rx_buf, data->uart_tx_buffer, len_to_copy);
    data->uart_tx_len = 0; // 數據已被讀走
    return len_to_copy;
}

int m90e26_emul_update_measurements(const struct emul *target, uint16_t v_rms, uint16_t i_rms) {
    struct m90e26_emul_data *data = target->data;
    data->regs.Urms = v_rms; data->regs.Irms = i_rms;
    LOG_INF("Updated measurements: Urms=%u, Irms=%u", v_rms, i_rms);
    return 0;
}

// 驅動程式初始化函式
static int m90e26_emul_init(const struct emul *target) {
    struct m90e26_emul_data *data = target->data;
    emul_reset(data);
    LOG_INF("Atmel M90E26 emulator initialized for %s", target->dev->name);
    return 0;
}

// 實例化驅動程式
#define M90E26_EMUL_INIT(n, api) \
    EMUL_DT_INST_DEFINE(n, \
                        m90e26_emul_init, \
                        &m90e26_emul_data_##n, \
                        NULL, \
                        &api, \
                        NULL);

#define M90E26_EMUL_UART(n) \
    static struct m90e26_emul_data m90e26_emul_data_##n; \
    M90E26_EMUL_INIT(n, m90e26_emul_uart_api)

DT_INST_FOREACH_STATUS_OKAY(M90E26_EMUL_UART)