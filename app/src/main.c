#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <hw_id.h>
#include <stdio.h>
#include <string.h>
#include "gpio-pwm.h"
#include "m90e26.h" // 引入 M90E26 的標頭檔
#include "nus_module.h"
#include "hmi_uart.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
static const struct device *energy_sensora = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_a));
static const struct device *energy_sensorb = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_b));
static const struct device *energy_sensorc = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_c));
#if DT_HAS_ALIAS(uart00) //在這邊使用實際Device Tree所定義的別名
#define HMI_UART_NODE00 DT_ALIAS(uart00)
#else
#error "請在設備樹中定義 uart30 別名！"
#endif

//宣告 HMI UART 實例的數據結構和消息隊列
K_MSGQ_DEFINE(my_uart_rx_msgq1, HMI_UART_RX_MSG_SIZE, 5, 4); // 為此實例定義自己的消息佇列
struct hmi_uart_data my_uart_instance_data1 = {.dev = DEVICE_DT_GET(HMI_UART_NODE00), .rx_msgq = &my_uart_rx_msgq1};

int main(void)
{
    char sensor_msg[3][128] = {0};
    char hw_id[HW_ID_LEN] = "";
    int ret = 0;

    ret = hw_id_get(hw_id, (size_t)HW_ID_LEN);
    if(ret != 0)
    {
        LOG_ERR("Get HW_ID failed\n");
    }
    LOG_INF("Device ID: %s", hw_id);

    if (!device_is_ready(energy_sensora)) {
        LOG_ERR("Energy sensor device not ready\n");
        //return -1;
    }

    if (!device_is_ready(energy_sensorb)) {
        LOG_ERR("Energy sensor device not ready\n");
        //return -1;
    }

    if (!device_is_ready(energy_sensorc)) {
        LOG_ERR("Energy sensor device not ready\n");
        //return -1;
    }

    ret = gpio_pwm_start();
	if (ret != 0) {
		LOG_ERR("Failed to start GPIO PWM module (err: %d)", ret);
		return 0;
	}

    ret = nus_module_init();
    if (ret != 0) {
		LOG_ERR("Failed to start NUS loopback module (err: %d)", ret);
		return 0;
	}

    // 初始化 HMI UART 實例，傳遞設備指針、波特率、實例數據和消息佇列
    ret = hmi_uart_init_instance(&my_uart_instance_data1, 115200);
    if (ret != 0) {
        LOG_ERR("HMI UART:%s 實例初始化失敗，錯誤碼: %d", my_uart_instance_data1.dev->name, ret);
        return 1;
    }

    while (1) {
        struct sensor_value voltage, current, power, pf;

        if (sensor_sample_fetch(energy_sensora) < 0) {
            LOG_ERR("Failed to fetch sample 1\n");
            //return -1;
        }

        if (sensor_sample_fetch(energy_sensorb) < 0) {
            LOG_ERR("Failed to fetch sample 2\n");
            //return -1;
        }

        if (sensor_sample_fetch(energy_sensorc) < 0) {
            LOG_ERR("Failed to fetch sample 3\n");
            //return -1;
        }

        sensor_channel_get(energy_sensora, SENSOR_CHAN_VOLTAGE, &voltage);
        sensor_channel_get(energy_sensora, SENSOR_CHAN_CURRENT, &current);
        sensor_channel_get(energy_sensora, SENSOR_CHAN_M90E26_ACTIVE_POWER, &power);
        sensor_channel_get(energy_sensora, SENSOR_CHAN_M90E26_POWER_FACTOR, &pf);

        LOG_INF("Sensor1 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        snprintf(sensor_msg[0], 128, "%s: Sensor1 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               hw_id,
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        LOG_INF("Send by uart 0: %s", sensor_msg[0]);
        ret = hmi_uart_send(my_uart_instance_data1.dev, sensor_msg[0], strlen(sensor_msg[0]));
        if(ret != 0)
        {
            LOG_ERR("Failed to send sensor1 value to 9151\n");
        }
/*==================================================================================================*/
        sensor_channel_get(energy_sensorb, SENSOR_CHAN_VOLTAGE, &voltage);
        sensor_channel_get(energy_sensorb, SENSOR_CHAN_CURRENT, &current);
        sensor_channel_get(energy_sensorb, SENSOR_CHAN_M90E26_ACTIVE_POWER, &power);
        sensor_channel_get(energy_sensorb, SENSOR_CHAN_M90E26_POWER_FACTOR, &pf);

        LOG_INF("Sensor2 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        snprintf(sensor_msg[1], 128, "%s: Sensor2 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               hw_id,
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        LOG_INF("Send by uart 1: %s", sensor_msg[1]);
        ret = hmi_uart_send(my_uart_instance_data1.dev, sensor_msg[1], strlen(sensor_msg[1]));
        if(ret != 0)
        {
            LOG_ERR("Failed to send sensor2 value to 9151\n");
        }
/*==================================================================================================*/
        sensor_channel_get(energy_sensorc, SENSOR_CHAN_VOLTAGE, &voltage);
        sensor_channel_get(energy_sensorc, SENSOR_CHAN_CURRENT, &current);
        sensor_channel_get(energy_sensorc, SENSOR_CHAN_M90E26_ACTIVE_POWER, &power);
        sensor_channel_get(energy_sensorc, SENSOR_CHAN_M90E26_POWER_FACTOR, &pf);

        LOG_INF("Sensor3 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        snprintf(sensor_msg[2], 128, "%s: Sensor3 | V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, PF: %d.%06d\n",
               hw_id,
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               pf.val1, pf.val2);
        LOG_INF("Send by uart 2: %s", sensor_msg[2]);
        ret = hmi_uart_send(my_uart_instance_data1.dev, sensor_msg[2], strlen(sensor_msg[2]));
        if(ret != 0)
        {
            LOG_ERR("Failed to send sensor3 value to 9151\n");
        }
        k_sleep(K_SECONDS(1));
    }
}