#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <time.h>

#define DT_DRV_COMPAT adi_hudson_fuel_gauge

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(EMUL_HUDSON_GAUGE);

static uint32_t emulated_registers[256];

static int hudson_gauge_emul_reg_read(uint32_t reg, uint32_t *val)
{
    if (reg >= ARRAY_SIZE(emulated_registers)) {
        LOG_ERR("Invalid register address 0x%02X", reg);
        return -EINVAL;
    }
    switch (reg) {
        case 0x44:
            // *val = (25 + (rand() % (40000 - 25 + 1))) / 1000;
            *val = 25;
            break;
        case 0x4C:
            // *val = 1 + (rand() % (900 - 1 + 1));
            *val = 500;
            break;
        case 0x50:
            // *val = 800 + (rand() % (1800 - 800 + 1));
            *val = 800;
            break;
        case 0x48:
            // *val = 250 + (rand() % (900 - 250 + 1));
            *val = 350;
            break;
        case 0x54:
            // *val = 2500 + (rand() % (4500 - 2500 + 1));
            *val = 4400;
            break;
        case 0x58:
            // *val = rand() % 2000;
            *val = 200;
            break;
        case 0x94:
            *val = 0x01;
            break;
        default:
            *val = emulated_registers[reg];
            break;
    }
    return 0;
}

static int hudson_gauge_emul_reg_write(uint32_t reg, uint32_t val)
{
    if (reg >= ARRAY_SIZE(emulated_registers)) {
        LOG_ERR("Invalid register address 0x%02X", reg);
        return -EINVAL;
    }

    emulated_registers[reg] = val;
    return 0;
}

static int hudson_gauge_emul_i2c(const struct emul *target, struct i2c_msg *msgs,
                   int num_msgs, int addr)
{
    struct hudson_gauge_emul_data *data;
    unsigned int val;
    int reg;
    int rc = 0;

    data = target->data;

    if (num_msgs != 2) {
        LOG_ERR("Invalid number of messages: %d", num_msgs);
        return -EIO;
    }

    reg = msgs[0].buf[0];

    if (msgs[1].flags & I2C_MSG_READ) {
        rc = hudson_gauge_emul_reg_read(reg, &val);
        sys_put_be32(val, msgs[1].buf);
    }

    if (msgs[1].flags & I2C_MSG_WRITE) {
        rc = hudson_gauge_emul_reg_write(reg, sys_get_le32(msgs[1].buf));
    }

    return rc;
}

static const struct i2c_emul_api hudson_gauge_emul_bus_api = {
    .transfer = hudson_gauge_emul_i2c,
};

static void hudson_fuel_gauge_emul_reset(const struct emul *emul)
{
    LOG_INF("Hudson Fuel Gauge emulator reset");
}

static int hudson_gauge_emul_init(const struct emul *emul, const struct device *parent)
{
	ARG_UNUSED(parent);

    hudson_fuel_gauge_emul_reset(emul);

    return 0;
}

#define HUDSON_GAUGE_EMUL_DEFINE(inst)                                                      \
    EMUL_DT_INST_DEFINE(inst, hudson_gauge_emul_init,                                       \
                        NULL, NULL, &hudson_gauge_emul_bus_api,  NULL);

DT_INST_FOREACH_STATUS_OKAY(HUDSON_GAUGE_EMUL_DEFINE)
