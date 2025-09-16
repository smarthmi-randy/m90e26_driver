#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "m90e26.h" // 引入 M90E26 的標頭檔

static const struct device *energy_sensor = DEVICE_DT_GET(DT_NODELABEL(m90e26));

int main(void)
{
    if (!device_is_ready(energy_sensor)) {
        printf("Energy sensor device not ready\n");
        return -1;
    }

    while (1) {
        struct sensor_value voltage, current, power, freq, pf;

        if (sensor_sample_fetch(energy_sensor) < 0) {
            printf("Failed to fetch sample\n");
            return -1;
        }

        sensor_channel_get(energy_sensor, SENSOR_CHAN_VOLTAGE, &voltage);
        sensor_channel_get(energy_sensor, SENSOR_CHAN_CURRENT, &current);
        sensor_channel_get(energy_sensor, SENSOR_CHAN_M90E26_ACTIVE_POWER, &power);
        //sensor_channel_get(energy_sensor, SENSOR_CHAN_FREQ, &freq);
        sensor_channel_get(energy_sensor, SENSOR_CHAN_M90E26_POWER_FACTOR, &pf);

        printf("V: %d.%06d V, I: %d.%06d A, P: %d.%06d W, Freq: %d.%06d Hz, PF: %d.%06d\n",
               voltage.val1, voltage.val2,
               current.val1, current.val2,
               power.val1, power.val2,
               freq.val1, freq.val2,
               pf.val1, pf.val2);

        k_sleep(K_SECONDS(5));
    }
}