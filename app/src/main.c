#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include "gpio-pwm.h"
#include "m90e26.h" // 引入 M90E26 的標頭檔
#include "nus_module.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
static const struct device *energy_sensora = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_a));
static const struct device *energy_sensorb = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_b));
static const struct device *energy_sensorc = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor_c));


int main(void)
{
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

    int ret = gpio_pwm_start();
	if (ret != 0) {
		LOG_ERR("Failed to start GPIO PWM module (err: %d)", ret);
		return 0;
	}

    ret = nus_module_init();
    if (ret != 0) {
		LOG_ERR("Failed to start NUS loopback module (err: %d)", ret);
		return 0;
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

        k_sleep(K_SECONDS(1));
    }
}