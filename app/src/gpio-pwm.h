#ifndef GPIO_PWM_H__
#define GPIO_PWM_H__

/**
 * @brief 啟動軟體 GPIO PWM
 *
 * 初始化一個 GPIO 腳位 (透過 'led0' alias) 並建立一個
 * 專門的執行緒來以 1Hz (50% duty cycle) 翻轉它。
 *
 * @retval 0 成功
 * @retval -EIO GPIO 相關錯誤
 * @retval -ENODEV 找不到 GPIO 設備
 */
int gpio_pwm_start(void);

#endif // GPIO_PWM_H__