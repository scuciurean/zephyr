#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/hudson.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(test_hudson_pmic, LOG_LEVEL_INF);

struct hudson_fixture {
    const struct device *pmic;
};

static void *hudson_pmic_setup(void)
{
    static ZTEST_DMEM struct hudson_fixture fixture;

    fixture.pmic = DEVICE_DT_GET(DT_NODELABEL(hudsonpmic));
    k_object_access_all_grant(fixture.pmic);

    zassert_true(device_is_ready(fixture.pmic), "PMIC not found");

    return &fixture;
}

static void test_buck(const struct device *dev, bool always_on)
{
    regulator_error_flags_t error_flags;
    int32_t voltage, voltage2;
    regulator_mode_t mode;
    int ret, voltage_count;

    if (!always_on) {
        ret = regulator_enable(dev);
        zassert_equal(ret, 0, "Failed to enable BUCK regulator");
    }
    voltage_count = regulator_count_voltages(dev);
    zassert_true(voltage_count > 0, "BUCK regulator does not support any voltages");

    for (int i = 0; i < voltage_count; i++) {
        ret = regulator_list_voltage(dev, i, &voltage);
        zassert_equal(ret, 0, "Failed to list voltage for BUCK regulator");
        LOG_INF("Supported voltage %d for BUCK: %d uV", i, voltage);
    }

    ret = regulator_set_voltage(dev, voltage, voltage);
    zassert_equal(ret, 0, "Failed to set voltage for BUCK regulator");

    ret = regulator_get_voltage(dev, &voltage2);
    zassert_equal(voltage, voltage2, "Failed to get voltage for BUCK regulator");

    ret = regulator_get_error_flags(dev, &error_flags);
    zassert_equal(error_flags, 0, "Failed to get error flags for BUCK regulator");

    if (!always_on) {
        ret = regulator_set_mode(dev, HUDSON_DCDC_MODE_FPWM);
        zassert_equal(ret, 0, "Failed to set mode for BUCK regulator");

        ret = regulator_get_mode(dev, &mode);
        zassert_equal(HUDSON_DCDC_MODE_FPWM, mode, "Failed to get mode for BUCK regulator");

        ret = regulator_disable(dev);
        zassert_equal(ret, 0, "Failed to disable BUCK regulator");
    }
}

static void test_ldo(const struct device *dev)
{
    int ret;

    ret = regulator_enable(dev);
    zassert_equal(ret, 0, "Failed to enable LDO regulator");


    ret = regulator_disable(dev);
    zassert_equal(ret, 0, "Failed to disable LDO regulator");
}

ZTEST(hudson_pmic, test_pmic_device_ready)
{
    const struct device *pmic_dev = DEVICE_DT_GET(DT_NODELABEL(hudsonpmic));

    zassert_true(device_is_ready(pmic_dev), "PMIC device is not ready");
}

ZTEST(hudson_pmic, test_buck0)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(buck0));
    test_buck(dev, false);
}

ZTEST(hudson_pmic, test_buck1)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(buck1));
    test_buck(dev, true);
}

ZTEST(hudson_pmic, test_buck2)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(buck2));
    test_buck(dev, true);
}

ZTEST(hudson_pmic, test_ldo0)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ldo0));
    test_ldo(dev);
}

ZTEST(hudson_pmic, test_ldo1)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ldo1));
    test_ldo(dev);
}

ZTEST(hudson_pmic, test_ldo2)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ldo2));
    test_ldo(dev);
}

ZTEST(hudson_pmic, test_ldo3)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ldo3));
    test_ldo(dev);
}

ZTEST_SUITE(hudson_pmic, NULL, hudson_pmic_setup, NULL, NULL, NULL);
