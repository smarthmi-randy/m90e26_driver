#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/* 包含自訂通道定義 */
#include "m90e26.h"
/* 包含模擬器的後門 API */
#include "m90e26_emul.h"

#define REAL_NODE	DT_NODELABEL(m90e26_sensor)

/* 從 Devicetree 獲取被測驅動程式 (DUT) 和模擬器的實例 */
static const struct device *dut_dev = DEVICE_DT_GET(DT_NODELABEL(m90e26_sensor));
static const struct emul *emul_dev = EMUL_DT_GET(DT_NODELABEL(m90e26_sensor));

static void test_suite_before_each(void *fixture)
{
    /* 確保模擬器在每個測試開始前都處於正常回應模式 */
    m90e26_emul_set_response_mode(emul_dev, true);
    /* 透過後門重置模擬器，以獲得一個乾淨的初始狀態 */
    m90e26_emul_set_register(emul_dev, 0x00 /* SoftReset */, 0x789A);
}

ZTEST(m90e26_driver_suite, test_init_and_device_ready)
{
    /*
     * 驅動程式的 `init` 函式會自動被 Zephyr 核心呼叫。
     * 這個測試僅驗證 `init` 函式執行後，設備是否處於 ready 狀態。
     * 這也間接測試了 `init` 流程中的 `m90e26_reg_write` 是否沒有返回致命錯誤。
     */
    zassert_true(device_is_ready(dut_dev), "Driver device should be ready after init");
}

ZTEST(m90e26_driver_suite, test_fetch_all_and_get_channels)
{
    int ret;
    struct sensor_value val;

    /* 2. Act: 呼叫 sample_fetch。這會觸發驅動程式讀取上述所有暫存器 */
    ret = sensor_sample_fetch(dut_dev);
    zassert_ok(ret, "sensor_sample_fetch failed: %d", ret);

    /* 1. Arrange: 在模擬器中設定一組已知的原始值 */
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x49 /*Urms*/, 23058)); // 230.58V
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x48 /*Irms*/, 1234));  // 1.234A
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x4A /*Pmean*/, 16225));   // 0.280kW = 280W
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x4B /*Qmean*/, 50));    // 0.050kvar = 50var
    //zassert_ok(m90e26_emul_set_register(emul_dev, 0x4F /*Smean*/, 285));   // 0.285kVA = 285VA
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x4C /*Freq*/, 5001));   // 50.01Hz
    zassert_ok(m90e26_emul_set_register(emul_dev, 0x4D /*PowerF*/, 982));  // 0.982

    ret = sensor_sample_fetch(dut_dev);
    zassert_ok(ret, "sensor_sample_fetch failed: %d", ret);

    /* 3. Assert: 逐一獲取每個通道的值，並驗證轉換是否正確 */
    ret = sensor_channel_get(dut_dev, SENSOR_CHAN_VOLTAGE, &val);
    zassert_ok(ret, "get SENSOR_CHAN_VOLTAGE failed");
    zassert_equal(val.val1, 230, "Voltage val1 mismatch");
    zassert_equal(val.val2, 580000, "Voltage val2 mismatch");

    ret = sensor_channel_get(dut_dev, SENSOR_CHAN_CURRENT, &val);
    zassert_ok(ret, "get SENSOR_CHAN_CURRENT failed");
    zassert_equal(val.val1, 1, "Current val1 mismatch");
    zassert_equal(val.val2, 234000, "Current val2 mismatch");

    ret = sensor_channel_get(dut_dev, SENSOR_CHAN_M90E26_ACTIVE_POWER, &val);
    zassert_ok(ret, "get ACTIVE_POWER failed");
    zassert_equal(val.val1, 16, "Active Power val1 mismatch");
    zassert_equal(val.val2, 225000, "Active Power val2 mismatch");

    ret = sensor_channel_get(dut_dev, SENSOR_CHAN_M90E26_REACTIVE_POWER, &val);
    zassert_ok(ret, "get REACTIVE_POWER failed");
    zassert_equal(val.val1, 0, "Reactive Power val1 mismatch");
    zassert_equal(val.val2, 50000, "Reactive Power val2 mismatch");

    // ret = sensor_channel_get(dut_dev, SENSOR_CHAN_M90E26_APPARENT_POWER, &val);
    // zassert_ok(ret, "get APPARENT_POWER failed");
    // zassert_equal(val.val1, 285, "Apparent Power val1 mismatch");
    // zassert_equal(val.val2, 0, "Apparent Power val2 mismatch");

    ret = sensor_channel_get(dut_dev, SENSOR_CHAN_M90E26_POWER_FACTOR, &val);
    zassert_ok(ret, "get POWER_FACTOR failed");
    zassert_equal(val.val1, 0, "Power Factor val1 mismatch");
    zassert_equal(val.val2, 982000, "Power Factor val2 mismatch");

    // 注意：您的驅動程式中註解掉了 SENSOR_CHAN_FREQ，所以我們不測試它
}

// ZTEST(m90e26_driver_suite, test_fetch_specific_channel_fails)
// {
//     /* 您的驅動程式實作應只接受 SENSOR_CHAN_ALL */
//     int ret = sensor_sample_fetch(dut_dev);
//     zassert_equal(ret, 0, "fetch should return 0 but return :%d", ret);
// }

ZTEST(m90e26_driver_suite, test_read_timeout)
{
    /* 1. Arrange: 設定模擬器為無回應模式 */
    zassert_ok(m90e26_emul_set_response_mode(emul_dev, false));

    /* 2. Act: 呼叫 fetch，它內部的第一個 `m90e26_reg_read` 應該會超時 */
    int ret = sensor_sample_fetch(dut_dev);

    /* 3. Assert: 驗證 `fetch` 是否回傳了超時錯誤 */
    zassert_equal(ret, -ETIMEDOUT, "fetch should have returned -ETIMEDOUT, but got %d", ret);
}

ZTEST_SUITE(m90e26_driver_suite, NULL, NULL, test_suite_before_each, NULL, NULL);