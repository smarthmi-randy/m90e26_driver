#include "gpio-pwm.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_pwm, LOG_LEVEL_DBG);

// 1Hz = 1000ms 週期
// 50% Duty = 500ms ON, 500ms OFF
#define SLEEP_TIME_MS 500

/*
 * 取得 'led0' alias 的 GPIO 規格
 * 這必須在 app.overlay 中定義
 */
#define LED_NODE DT_ALIAS(gpiopwm0)
#define LED_NODE1 DT_ALIAS(gpiopwm1)
#define LED_NODE2 DT_ALIAS(gpiopwm2)
#define LED_NODE3 DT_ALIAS(gpiopwm3)
#define LED_NODE4 DT_ALIAS(gpiopwm4)
#define LED_NODE5 DT_ALIAS(gpiopwm5)



#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#define LED_NUM	(6)

// 宣告 GPIO 規格結構體
static const struct gpio_dt_spec leds[] = { GPIO_DT_SPEC_GET(LED_NODE, gpios)
	, GPIO_DT_SPEC_GET(LED_NODE1, gpios)
	, GPIO_DT_SPEC_GET(LED_NODE2, gpios)
	, GPIO_DT_SPEC_GET(LED_NODE3, gpios)
	, GPIO_DT_SPEC_GET(LED_NODE4, gpios)
	, GPIO_DT_SPEC_GET(LED_NODE5, gpios)};

/*
 * 定義我們專門用來閃燈的執行緒
 */
#define GPIO_PWM_THREAD_STACK_SIZE 512
#define GPIO_PWM_THREAD_PRIORITY 5

// 宣告執行緒的堆疊 (stack) 空間
K_THREAD_STACK_DEFINE(g_gpio_pwm_thread_stack, GPIO_PWM_THREAD_STACK_SIZE);
// 宣告執行緒的控制區塊
static struct k_thread g_gpio_pwm_thread;


/**
 * @brief GPIO PWM 執行緒的進入點 (entry point)
 *
 * 這個函式會在新建立的執行緒中執行，
 * 它只做一件事：在無限迴圈中翻轉 GPIO 並睡眠。
 */
static void gpio_pwm_thread_entry(void *p1, void *p2, void *p3)
{
	int ret;
	
	LOG_INF("GPIO PWM thread started");

	// 無限迴圈
	while (1) {
		// 翻轉 (Toggle) GPIO 腳位的狀態
		for(int i = 0; i < LED_NUM; i++)
		{
			ret = gpio_pin_toggle_dt(&leds[i]);
			if (ret < 0) {
				LOG_ERR("Error: Failed to toggle %s pin %d (err: %d)", 
						leds[i].port->name, leds[i].pin, ret);
				// 即使出錯，我們也繼續嘗試
			}
		}
		// 讓當前執行緒睡眠 500ms
		k_msleep(SLEEP_TIME_MS);
	}
}

/**
 * @brief 啟動軟體 GPIO PWM (gpio-pwm.h 中的函式實作)
 */
int gpio_pwm_start(void)
{
	int ret;
	for(int i = 0; i < LED_NUM; i++)
	{
		// 1. 檢查 GPIO 設備是否準備就緒
		if (!gpio_is_ready_dt(&leds[i])) {
			LOG_ERR("Error: GPIO device %s is not ready", leds[i].port->name);
			return -ENODEV;
		}

		// 2. 將 GPIO 腳位設定為輸出 (Output)
		ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Error: Failed to configure %s pin %d (err: %d)", 
					leds[i].port->name, leds[i].pin, ret);
			return -EIO;
		}

		LOG_INF("GPIO PWM module started on %s pin %d", leds[i].port->name, leds[i].pin);
	}

	// 3. 建立並啟動執行緒
	k_thread_create(&g_gpio_pwm_thread, g_gpio_pwm_thread_stack,
			K_THREAD_STACK_SIZEOF(g_gpio_pwm_thread_stack),
			gpio_pwm_thread_entry, // 執行緒函式
			NULL, NULL, NULL,        // 參數 (未使用)
			GPIO_PWM_THREAD_PRIORITY, // 優先權
			0, K_NO_WAIT);           // 選項 & 延遲
	
	return 0;
}