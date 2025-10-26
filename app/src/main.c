#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>
#include "m90e26.h" // 引入 M90E26 的標頭檔

#define TOTAL_SENSOR_NUM 1
#define PWM_PERIOD_NS (1000000000ULL) /* 1 秒 = 1,000,000,000 奈秒 */
#define PWM_PULSE_NS  (500000000ULL)  /* 0.5 秒 = 500,000,000 奈秒 */

static const struct device *energy_sensors[] = {DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor1)), DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor2)), DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor3))};
static const struct device *pwm_devs[] = {DEVICE_DT_GET(DT_NODELABEL(pwm20)), DEVICE_DT_GET(DT_NODELABEL(pwm21)), DEVICE_DT_GET(DT_NODELABEL(pwm22))};


int main(void)
{
    int ret = 0;
    for(int i = 0; i < TOTAL_SENSOR_NUM; i++)
    {
        if (!device_is_ready(energy_sensors[i])) {
            printf("Energy sensor device %d not ready\n", i + 1);
        }
        if (!device_is_ready(pwm_devs[i])) {
            printf("PWM device %s is not ready", pwm_devs[i]->name);
        }
        ret = pwm_set(pwm_devs[i], 0, PWM_PERIOD_NS, PWM_PULSE_NS, 0);
        if (ret) {
            printf("Error %d: failed to set PWM on %s channel 1", ret, pwm_devs[i]->name);
        }
    }

    while (1) {
        struct sensor_value voltage, current, power, freq, pf;
        for(int i = 0; i < TOTAL_SENSOR_NUM; i++) {
            if (sensor_sample_fetch(energy_sensors[i]) < 0) {
                printf("Failed to fetch sensor %d sample\n", i + 1);
            }
        }

        for(int i = 0; i < TOTAL_SENSOR_NUM; i++) {
            sensor_channel_get(energy_sensors[i], SENSOR_CHAN_VOLTAGE, &voltage);
            sensor_channel_get(energy_sensors[i], SENSOR_CHAN_CURRENT, &current);
            sensor_channel_get(energy_sensors[i], SENSOR_CHAN_M90E26_ACTIVE_POWER, &power);
            sensor_channel_get(energy_sensors[i], SENSOR_CHAN_M90E26_POWER_FACTOR, &pf);

            printf("V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, Freq: %d.%06d Hz, PF: %d.%06d\n",
                voltage.val1, voltage.val2,
                current.val1, current.val2,
                power.val1, power.val2,
                freq.val1, freq.val2,
                pf.val1, pf.val2);
        }

        k_sleep(K_SECONDS(1));
    }
}