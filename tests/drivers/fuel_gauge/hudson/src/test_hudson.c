#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <stdlib.h>
#include <math.h>

#define FG_NODE DT_ALIAS(hudsonfuelgauge)
#define BETA 3977         /* Beta coefficient */
#define R25 10000         /* Thermistor resistance at 25 C in ohms */
#define T25_K 298.15      /* 25 C in Kelvin */

struct temp_range {
    uint16_t v_min;
    uint16_t v_max;
};

struct temp_range temp_ranges[] = {
    { 250, 900 }, /* TempSense  */
    { 1, 900 },   /* vTherm  */
    { 800, 1800 } /* Vthmbias  */
};

struct hudson_fixture {
    const struct device *fg;
    const struct fuel_gauge_driver_api *api;
};

int32_t calculate_temperature(uint16_t adc_raw, struct temp_range temp_range, uint32_t r_fixed)
{
    uint32_t r_thermistor;
    int32_t temp_k;

    if (adc_raw < temp_range.v_min || adc_raw > temp_range.v_max) {
        return -1;
    }

    r_thermistor = (uint32_t)r_fixed * adc_raw / (temp_range.v_max - adc_raw);
    temp_k = (int32_t)(BETA / (log((float)r_thermistor / R25) + (BETA / T25_K)));

    return (temp_k - 273.15) * 100;
}

static void *hudson_setup(void)
{
    static ZTEST_DMEM struct hudson_fixture fixture;

    fixture.fg = DEVICE_DT_GET(FG_NODE);
    k_object_access_all_grant(fixture.fg);

    zassert_true(device_is_ready(fixture.fg), "Fuel Gauge not found");

    return &fixture;
}

ZTEST(hudson, test_fuel_gauge_properties)
{
    const struct device *fg = DEVICE_DT_GET(FG_NODE);
    fuel_gauge_prop_t props[] = {
        FUEL_GAUGE_VOLTAGE,
        FUEL_GAUGE_CURRENT,
        FUEL_GAUGE_MOISTURE,
    };
    union fuel_gauge_prop_val vals[ARRAY_SIZE(props)];
    int ret;

    zassert_true(device_is_ready(fg), "Fuel gauge device is not ready");

    ret = fuel_gauge_get_props(fg, props, vals, ARRAY_SIZE(props));
    zassert_equal(ret, 0, "Failed to get fuel gauge properties");

    printk("Voltage: %d.%03d mV\n", vals[0].voltage / 1000, vals[0].voltage % 1000);
    printk("Current: %d.%03d mA\n", vals[1].current / 1000, vals[1].current % 1000);
    printk("Moisture: %d g/m^3\n", vals[2].moisture);

    zassert_equal(vals[0].voltage, 304000, "Voltage mismatch");
    zassert_equal(vals[1].current, 25000, "Current mismatch");
    zassert_equal(vals[2].moisture, 200, "Moisture mismatch");
}

ZTEST(hudson, test_fuel_gauge_temperature)
{
    const struct device *fg = DEVICE_DT_GET(FG_NODE);
    uint16_t temperatures[3];
    int32_t temps[3];
    int ret;
    zassert_true(device_is_ready(fg), "Fuel gauge device is not ready");

    ret = fuel_gauge_get_buffer_prop(fg, FUEL_GAUGE_TEMPERATURE, temperatures, sizeof(temperatures));
    zassert_equal(ret, 0, "Failed to get temperature buffer");

    temps[0] = calculate_temperature(temperatures[0], temp_ranges[0], R25);
    temps[1] = calculate_temperature(temperatures[1], temp_ranges[1], R25);
    temps[2] = calculate_temperature(temperatures[2], temp_ranges[2], R25);

    printk("Battery temp: %d.%02d°C\n", temps[0] / 100, abs(temps[0] % 100));
    printk("Die temp: %d.%02d°C\n", temps[1] / 100, abs(temps[1] % 100));
    printk("Charger temp: %d.%02d°C\n", temps[2] / 100, abs(temps[2] % 100));

    zassert_equal(temps[0], 3485, "Invalid battery temperature");
    zassert_equal(temps[1], 1985, "Invalid die temperature");
    zassert_equal(temps[2], 2985, "Invalid charger temperature");
}

ZTEST_SUITE(hudson, NULL, hudson_setup, NULL, NULL, NULL);
