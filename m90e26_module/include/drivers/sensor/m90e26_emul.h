#ifndef M90E26_EMUL_API_H
#define M90E26_EMUL_API_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/emul_sensor.h>


/**
 * @brief 模擬主機 (MCU) 向 M90E26 模擬器寫入 UART 數據。
 *
 * @param dev 指向 M90E26 模擬器設備實例的指標。
 * @param tx_data 指向要寫入的數據緩衝區。
 * @param tx_len 要寫入的數據長度。
 * @return 0 on success, negative error code on failure.
 */
int m90e26_emul_host_uart_write(const struct emul *target, const uint8_t *tx_data, size_t tx_len);

/**
 * @brief 模擬主機 (MCU) 從 M90E26 模擬器讀取 UART 回應數據。
 *
 * @param dev 指向 M90E26 模擬器設備實例的指標。
 * @param rx_buf 指向用於存放讀取數據的緩衝區。
 * @param rx_buf_len 緩衝區的最大長度。
 * @return 讀取到的位元組數，如果沒有數據則為 0，或在失敗時返回負的錯誤碼。
 */
int m9e26_emul_host_uart_read(const struct emul *target, uint8_t *rx_buf, size_t rx_buf_len);

/**
 * @brief (供測試使用) 更新模擬器的內部量測值。
 *
 * @param dev 指向 M90E26 模擬器設備實例的指標。
 * @param v_rms 模擬的電壓 RMS 值 (單位: 0.01V)。
 * @param i_rms 模擬的 L-line 電流 RMS 值 (單位: 0.001A)。
 * @return 0 on success, negative error code on failure.
 */
int m90e26_emul_update_measurements(const struct emul *target, uint16_t v_rms, uint16_t i_rms);

/**
 * @brief (供測試使用) 直接設定模擬器內部暫存器的值。
 * @note 這是一個「後門」函式，專為 Ztest 的 Arrange 步驟設計。
 *
 * @param dev 指向 M90E26 模擬器設備實例的指標。
 * @param addr 要設定的暫存器位址。
 * @param value 要設定的值。
 * @return 0 on success, negative error code on failure.
 */
int m90e26_emul_set_register(const struct emul *target, uint8_t addr, uint16_t value);

/**
 * @brief (供測試使用) 設定模擬器的回應模式。
 * @note 用於測試驅動程式的超時和錯誤處理能力。
 *
 * @param dev 指向模擬器設備的指標。
 * @param respond 如果為 true，模擬器正常回應；如果為 false，則不回應任何請求。
 * @return 0 on success.
 */
int m90e26_emul_set_response_mode(const struct emul *target, bool respond);


#endif /* M90E26_EMUL_API_H */