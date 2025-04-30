
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(emul_hudson, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT adi_hudson_pmic

static volatile uint32_t emulated_registers[256];

static int hudson_regulator_emul_reg_read(uint32_t reg, uint32_t *val)
{
    if (reg >= ARRAY_SIZE(emulated_registers)) {
        LOG_ERR("Invalid register address 0x%02X", reg);
        return -EINVAL;
    }
    switch (reg) {
        default:
            *val = emulated_registers[reg];
            break;
    }

    return 0;
}

static int hudson_regulator_emul_reg_write(uint32_t reg, uint32_t val)
{
    if (reg >= ARRAY_SIZE(emulated_registers)) {
        LOG_ERR("Invalid register address 0x%02X", reg);
        return -EINVAL;
    }

    switch (reg) {
        case 0x24:
            // Emulate the PMIC_STAT en status for the regulators
            emulated_registers[reg] = val;
            emulated_registers[0] = emulated_registers[0] & 0xFFFFFFE0;
            emulated_registers[0] |= val | 0x1F;
            break;

        default:
            emulated_registers[reg] = val;
            break;
    }

    return 0;
}

static int hudson_regulator_emul_i2c(const struct emul *target, struct i2c_msg *msgs,
                   int num_msgs, int addr)
{
    struct hudson_regulator_emul_data *data;
    unsigned int val;
    int reg;
    int rc = 0;

    data = target->data;

    if (num_msgs != 2) {
        LOG_ERR("Invalid number of messages: %d", num_msgs);
        return -EIO;
    }

    reg = msgs[0].buf[0];

    if ((msgs[1].flags & I2C_MSG_READ) && (msgs[1].flags & I2C_MSG_STOP)) {
        rc = hudson_regulator_emul_reg_read(reg, &val);
        sys_put_le32(val, msgs[1].buf);
    } else if (msgs[1].flags & I2C_MSG_STOP) {
        rc = hudson_regulator_emul_reg_write(reg, sys_get_le32(msgs[1].buf));
    }

    return rc;
}

static int hudson_regulator_emul_init(const struct emul *emul, const struct device *parent)
{
    ARG_UNUSED(emul);
    ARG_UNUSED(parent);


    return 0;
}

static const struct i2c_emul_api hudson_regulator_emul_bus_api = {
    .transfer = hudson_regulator_emul_i2c,
};

#define HUDSON_REGULATOR_EMUL_INIT(inst)                                                        \
    EMUL_DT_INST_DEFINE(inst, hudson_regulator_emul_init,                                       \
                        NULL, NULL, &hudson_regulator_emul_bus_api,  NULL);

DT_INST_FOREACH_STATUS_OKAY(HUDSON_REGULATOR_EMUL_INIT)