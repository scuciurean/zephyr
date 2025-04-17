#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/hudson.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT adi_hudson_pmic

LOG_MODULE_REGISTER(PMIC_HUDSON);

// Hudson PMIC register map
#define HUDSON_REG_PMIC_SEQ_STAT    0x00
#define HUDSON_REG_DCLM_DLY         0x04
#define HUDSON_REG_DCLM_CFG         0x08
#define HUDSON_REG_DCRF_CFG         0x0C
#define HUDSON_REG_DCAVH_CFG        0x10
#define HUDSON_REG_CLASSD_LDO0_DLY  0x14
#define HUDSON_REG_CLASSD_LDO1_DLY  0x18
#define HUDSON_REG_ADC_LDO2_DLY     0x1C
#define HUDSON_REG_1V2_LDO3_DLY     0x20
#define HUDSON_REG_PMIC_CTRL        0x24

// PMIC_STAT bit descriptions
#define HUDSON_PMIC_STAT_FAULT_PMIC         BIT(10)
#define HUDSON_PMIC_STAT_FAULT_1V2          BIT(9)
#define HUDSON_PMIC_STAT_FAULT_ADC          BIT(8)
#define HUDSON_PMIC_STAT_FAULT_CLASSD1      BIT(7)
#define HUDSON_PMIC_STAT_FAULT_CLASSD0      BIT(6)
#define HUDSON_PMIC_STAT_FAULT_DCLM         BIT(5)
#define HUDSON_PMIC_STAT_EN_1V2_STAT        BIT(4)
#define HUDSON_PMIC_STAT_EN_ADC_STAT        BIT(3)
#define HUDSON_PMIC_STAT_EN_CLASSD1_STAT    BIT(2)
#define HUDSON_PMIC_STAT_EN_CLASSD0_STAT    BIT(1)
#define HUDSON_PMIC_STAT_EN_DCLM_STAT       BIT(0)

// PMIC_CTRL bit descriptions
#define HUDSON_PMIC_CTRL_MAN_MODE_1V2       BIT(14)
#define HUDSON_PMIC_CTRL_MAN_MODE_ADC       BIT(13)
#define HUDSON_PMIC_CTRL_MAN_MODE_CLASSD1   BIT(12)
#define HUDSON_PMIC_CTRL_MAN_MODE_CLASSD0   BIT(11)
#define HUDSON_PMIC_CTRL_MAN_MODE_DCLM      BIT(10)
#define HUDSON_PMIC_CTRL_EN_MAN_1V2         BIT(9)
#define HUDSON_PMIC_CTRL_EN_MAN_ADC         BIT(8)
#define HUDSON_PMIC_CTRL_EN_MAN_CLASSD1     BIT(7)
#define HUDSON_PMIC_CTRL_EN_MAN_CLASSD0     BIT(6)
#define HUDSON_PMIC_CTRL_EN_MAN_DCLM        BIT(5)
#define HUDSON_PMIC_CTRL_ARM_EN_1V2         BIT(4)
#define HUDSON_PMIC_CTRL_ARM_EN_ADC         BIT(3)
#define HUDSON_PMIC_CTRL_ARM_EN_CLASSD1     BIT(2)
#define HUDSON_PMIC_CTRL_ARM_EN_CLASSD0     BIT(1)
#define HUDSON_PMIC_CTRL_ARM_EN_DCLM        BIT(0)

// DCDC_CONFIG bit descriptions
#define HUDSON_DCDC_CONFIG_FLIM_MODE        BIT(0)
#define HUDSON_DCDC_CONFIG_HIZ              BIT(1)
#define HUDSON_DCDC_CONFIG_IDLE             BIT(2)
#define HUDSON_DCDC_CONFIG_TRIM_ILR         GENMASK(5, 3)
#define HUDSON_DCDC_CONFIG_TRIM_VREG        GENMASK(10, 6)
#define HUDSON_DCDC_CONFIG_VSEL             GENMASK(16, 11)
#define HUDSON_DCDC_CONFIG_OUTPUT_MASK      GENMASK(16, 3)

// DLY bit descriptions
#define HUDSON_DLY_CONFIG_OFF               GENMASK(24, 18)
#define HUDSON_DLY_CONFIG_ON                GENMASK(17, 9)
#define HUDSON_DLY_CONFIG_ROK               GENMASK(8, 0)

// DCDC_MODE bit descriptions
#define HUDSON_DCDC_MODE_EN_MASK            BIT(3)
#define HUDSON_DCDC_MODE_MASK               GENMASK(2, 0)


enum hudson_sources {
    HUDSON_SOURCE_BUCK0 = 0,    // DCLM
    HUDSON_SOURCE_BUCK1,        // DCRF
    HUDSON_SOURCE_BUCK2,        // DCAVH
    HUDSON_SOURCE_LDO0,         // CLASSD0
    HUDSON_SOURCE_LDO1,         // CLASSD1
    HUDSON_SOURCE_LDO2,         // ADC
    HUDSON_SOURCE_LDO3,         // 1V2
    HUDSON_NUM_SOURCES,
};

struct regulator_hudson_data {
	struct regulator_common_data data;
};

struct regulator_hudson_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
    const struct regulator_hudson_desc *desc;
    uint32_t trim_ilr;
    uint32_t trim_vreg;
    uint32_t turn_on_us;
    uint32_t turn_off_us;
    uint32_t rok_us;
};

struct regulator_hudson_source_config {
	enum hudson_sources source;
    uint32_t control_reg;
    uint32_t sequence_reg;
    uint32_t enable;
    uint32_t manual_override;
    uint32_t status;
    uint32_t fault;
};

struct regulator_hudson_desc {
    struct regulator_hudson_source_config source_config;
    uint8_t num_ranges;
    const struct linear_range *ranges;
};

struct regulator_hudson_pmic_config {
    const struct device *dev;
	struct i2c_dt_spec i2c;
	regulator_dvs_state_t dvs_state;
};

static const struct linear_range linear_ranges[7][1] = {
    // DCLM 0.5V to 0.75V
    [HUDSON_SOURCE_BUCK0] = {
        LINEAR_RANGE_INIT(500000, 100000U, 0x0U, 0x0BU)
    },
    // DCRF 0.94V
    [HUDSON_SOURCE_BUCK1] = {
        LINEAR_RANGE_INIT(940000, 0, 0, 0),
    },
    // DCAVH 1.9V
    [HUDSON_SOURCE_BUCK2] = {
        LINEAR_RANGE_INIT(1900000, 0, 0, 0),
    },
    // CLASSD0 1.8V
    [HUDSON_SOURCE_LDO0] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    // CLASSD1 1.8V
    [HUDSON_SOURCE_LDO1] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    // ADC 1.8V
    [HUDSON_SOURCE_LDO2] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    // 1V2 1.2V
    [HUDSON_SOURCE_LDO3] = {
        LINEAR_RANGE_INIT(1200000, 0, 0, 0),
    },
};

static const struct regulator_hudson_desc regulator_hudson_desc_list[] = {
    [HUDSON_SOURCE_BUCK0] = {
        .source_config = {
            .source = HUDSON_SOURCE_BUCK0,
            .control_reg = HUDSON_REG_DCLM_CFG,
            .sequence_reg = HUDSON_REG_DCLM_DLY,
            .enable = HUDSON_PMIC_CTRL_ARM_EN_DCLM,
            .manual_override = HUDSON_PMIC_CTRL_MAN_MODE_DCLM,
            .fault = HUDSON_PMIC_STAT_FAULT_DCLM,
            .status = HUDSON_PMIC_STAT_EN_DCLM_STAT,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_BUCK0],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_BUCK0]),
    },
    [HUDSON_SOURCE_BUCK1] = {
        .source_config = {
            .source = HUDSON_SOURCE_BUCK1,
            .control_reg = HUDSON_REG_DCRF_CFG,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_BUCK1],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_BUCK1]),
    },
    [HUDSON_SOURCE_BUCK2] = {
        .source_config = {
            .source = HUDSON_SOURCE_BUCK2,
            .control_reg = HUDSON_REG_DCAVH_CFG,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_BUCK2],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_BUCK2]),
    },
    [HUDSON_SOURCE_LDO0] = {
        .source_config = {
            .source = HUDSON_SOURCE_LDO0,
            .sequence_reg = HUDSON_REG_CLASSD_LDO0_DLY,
            .enable = HUDSON_PMIC_CTRL_ARM_EN_CLASSD0,
            .manual_override = HUDSON_PMIC_CTRL_MAN_MODE_CLASSD0,
            .fault = HUDSON_PMIC_STAT_FAULT_CLASSD0,
            .status = HUDSON_PMIC_STAT_EN_CLASSD0_STAT,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_LDO0],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_LDO0]),
    },
    [HUDSON_SOURCE_LDO1] = {
        .source_config = {
            .source = HUDSON_SOURCE_LDO1,
            .sequence_reg = HUDSON_REG_CLASSD_LDO1_DLY,
            .enable = HUDSON_PMIC_CTRL_ARM_EN_CLASSD1,
            .manual_override = HUDSON_PMIC_CTRL_MAN_MODE_CLASSD1,
            .fault = HUDSON_PMIC_STAT_FAULT_CLASSD1,
            .status = HUDSON_PMIC_STAT_EN_CLASSD1_STAT,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_LDO1],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_LDO1]),
    },
    [HUDSON_SOURCE_LDO2] = {
        .source_config = {
            .source = HUDSON_SOURCE_LDO2,
            .sequence_reg = HUDSON_REG_ADC_LDO2_DLY,
            .enable = HUDSON_PMIC_CTRL_ARM_EN_ADC,
            .manual_override = HUDSON_PMIC_CTRL_MAN_MODE_ADC,
            .fault = HUDSON_PMIC_STAT_FAULT_ADC,
            .status = HUDSON_PMIC_STAT_EN_ADC_STAT,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_LDO2],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_LDO2]),
    },
    [HUDSON_SOURCE_LDO3] = {
        .source_config = {
            .source = HUDSON_SOURCE_LDO3,
            .sequence_reg = HUDSON_REG_1V2_LDO3_DLY,
            .enable = HUDSON_PMIC_CTRL_ARM_EN_1V2,
            .manual_override = HUDSON_PMIC_CTRL_MAN_MODE_1V2,
            .fault = HUDSON_PMIC_STAT_FAULT_1V2,
            .status = HUDSON_PMIC_STAT_EN_1V2_STAT,
        },
        .ranges = linear_ranges[HUDSON_SOURCE_LDO3],
        .num_ranges = ARRAY_SIZE(linear_ranges[HUDSON_SOURCE_LDO3]),
    }
};

static int i2c_reg_read_32(struct i2c_dt_spec i2c, uint32_t reg, uint32_t *val)
{
    uint8_t buf[4];
    int ret;

    ret = i2c_burst_read_dt(&i2c, reg, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%02X", reg);
        return ret;
    }

    *val = sys_get_be32(buf);

    return 0;
}

static int i2c_reg_write_32(struct i2c_dt_spec i2c, uint8_t reg, uint32_t val)
{
    uint8_t buf[4];

    sys_put_be32(val, &buf[0]);

    return i2c_burst_write_dt(&i2c, reg, buf, sizeof(buf));
}

static int i2c_reg_update_32(struct i2c_dt_spec i2c, uint8_t reg, uint32_t mask, uint32_t val)
{
    uint32_t current_val;
    int ret;

    ret = i2c_reg_read_32(i2c, reg, &current_val);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%02X for update", reg);
        return ret;
    }

    current_val = (current_val & ~mask) | (val & mask);

    ret = i2c_reg_write_32(i2c, reg, current_val);
    if (ret < 0) {
        LOG_ERR("Failed to write updated value to register 0x%02X", reg);
        return ret;
    }

    return 0;
}

static int regulator_hudson_is_enabled(const struct device *dev, bool *is_enabled)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t pmic_stat;
    int ret;

    if (!source_config->status) {
        LOG_ERR("Status not supported for this regulator");
        return -ENOTSUP;
    }

    ret = i2c_reg_read_32(config->i2c, HUDSON_REG_PMIC_SEQ_STAT, &pmic_stat);
    if (ret < 0) {
        LOG_ERR("Failed to read PMIC_STAT register");
        return ret;
    }

    *is_enabled = (pmic_stat & source_config->status) != 0;

    return 0;
}

static int regulator_hudson_configure_delays(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t reg_val;

    if (source_config->sequence_reg == 0) {
        return 0;
    }

    reg_val = config->turn_off_us | (config->turn_off_us << 8) | (config->rok_us << 16);

    return i2c_reg_write_32(config->i2c, source_config->sequence_reg, reg_val);
}

static int regulator_hudson_enable(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    int ret;

    if (desc->source_config.enable == 0) {
        return -ENOTSUP;
    }

    ret = i2c_reg_update_32(config->i2c, HUDSON_REG_PMIC_CTRL,
                            desc->source_config.manual_override,
                            desc->source_config.manual_override);
    if (ret < 0) {
        LOG_ERR("Failed to enable manual mode for regulator");
        return ret;
    }

    ret = i2c_reg_update_32(config->i2c, HUDSON_REG_PMIC_CTRL,
                            desc->source_config.enable,
                            desc->source_config.enable);
    if (ret < 0) {
        LOG_ERR("Failed to enable regulator");
        return ret;
    }

    return 0;
}

static int regulator_hudson_disable(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    int ret;

    if (desc->source_config.enable == 0) {
        return -ENOTSUP;
    }

    ret = i2c_reg_update_32(config->i2c, HUDSON_REG_PMIC_CTRL,
                            desc->source_config.enable,
                            0);
    if (ret < 0) {
        LOG_ERR("Failed to disable regulator");
        return ret;
    }

    return 0;
}

static unsigned int regulator_hudson_count_voltages(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;

    return linear_range_group_values_count(desc->ranges, desc->num_ranges);
}

static int regulator_hudson_list_voltages(const struct device *dev, unsigned int idx,
    int32_t *volt_uv)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;

	return linear_range_group_get_value(desc->ranges, desc->num_ranges,
                                        idx, volt_uv);
}

static int regulator_hudson_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint16_t target;
    uint32_t reg_val;
    int ret;

    if (source_config->control_reg == 0) {
        LOG_ERR("Voltage control not supported for this regulator");
        return -ENOTSUP;
    }

    ret = linear_range_group_get_win_index(desc->ranges, desc->num_ranges, min_uv, max_uv, &target);
    if (ret < 0) {
        LOG_ERR("Failed to find suitable voltage range");
        return ret;
    }

    reg_val = ((uint32_t)target << 11) & HUDSON_DCDC_CONFIG_VSEL;

    if (config->trim_ilr != 0) {
        reg_val |= (config->trim_ilr << 3) & HUDSON_DCDC_CONFIG_TRIM_ILR;
    }

    if (config->trim_vreg != 0) {
        reg_val |= (config->trim_vreg << 6) & HUDSON_DCDC_CONFIG_TRIM_VREG;
    }

    ret = i2c_reg_update_32(config->i2c, source_config->control_reg, HUDSON_DCDC_CONFIG_OUTPUT_MASK, reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to set voltage for regulator");
        return ret;
    }

    return 0;
}

static int regulator_hudson_get_voltage(const struct device *dev, int32_t *voltage)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t reg_val;
    uint32_t selector;
    int ret;

    if (source_config->control_reg == 0) {
        *voltage = desc->ranges[0].min;
        return 0;
    }

    ret = i2c_reg_read_32(config->i2c, source_config->control_reg, &reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to read voltage for regulator");
        return ret;
    }

    selector = (reg_val & HUDSON_DCDC_CONFIG_VSEL) >> 11;

    ret = linear_range_group_get_value(desc->ranges, desc->num_ranges, selector, voltage);
    if (ret < 0) {
        LOG_ERR("Failed to get voltage value from selector");
        return ret;
    }

    return 0;
}

static unsigned int regulator_hudson_count_current_limits(const struct device *dev)
{
    return 0;
}

static int hudson_regulator_list_current_limit(const struct device *dev, unsigned int idx,
    int32_t *current_ua)
{
    return -ENOTSUP;
}

static int regulator_hudson_set_current_limit(const struct device *dev,
    int32_t min_ua, int32_t max_ua)
{
    return -ENOTSUP;
}

static int regulator_hudson_get_current_limit(const struct device *dev, int32_t *current_ua)
{
    return -ENOTSUP;
}

static int regulator_hudson_set_mode(const struct device *dev, regulator_mode_t mode)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t en;
    int ret;

    if (source_config->control_reg == 0) {
        LOG_ERR("Mode control not supported for this regulator");
        return -ENOTSUP;
    }

    if ((source_config->source == HUDSON_SOURCE_BUCK1 ||
        source_config->source == HUDSON_SOURCE_BUCK2) && mode == HUDSON_DCDC_MODE_FPWM) {
        LOG_ERR("FPWM mode not supported for this regulator");
        return -ENOTSUP;
    }

    en = mode & HUDSON_DCDC_MODE_EN_MASK;

    if (en) {
        ret = regulator_hudson_enable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to enable regulator");
            return ret;
        }
    } else {
        ret = regulator_hudson_disable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to disable regulator");
            return ret;
        }
    }

    ret = i2c_reg_update_32(config->i2c, source_config->control_reg, HUDSON_DCDC_MODE_MASK, mode);
    if (ret < 0) {
        LOG_ERR("Failed to set mode for regulator");
        return ret;
    }

    return 0;
}

static int regulator_hudson_get_mode(const struct device *dev, regulator_mode_t *mode)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t reg_val;
    bool is_enabled;
    int ret;

    if (source_config->control_reg == 0) {
        LOG_ERR("Mode control not supported for this regulator");
        return -ENOTSUP;
    }

    ret = i2c_reg_read_32(config->i2c, source_config->control_reg, &reg_val);
    if (ret < 0) {
        LOG_ERR("Failed to read mode for regulator");
        return ret;
    }

    ret = regulator_hudson_is_enabled(dev, &is_enabled);
    if (ret < 0) {
        return ret;
    }

    *mode = (reg_val & HUDSON_DCDC_MODE_MASK) | (is_enabled ? HUDSON_DCDC_MODE_EN_MASK : 0);

    return 0;

}

static int regulator_hudson_set_active_discharge(const struct device *dev, bool active_discharge)
{
    return -ENOTSUP;
}

static int regulator_hudson_get_active_discharge(const struct device *dev, bool *active_discharge)
{
    return -ENOTSUP;
}

static int regulator_hudson_get_error_flags(const struct device *dev, regulator_error_flags_t *error_flags)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;
    const struct regulator_hudson_source_config *source_config = &desc->source_config;
    uint32_t pmic_stat;
    int ret;

    if (!source_config->fault) {
        LOG_ERR("Error flags not supported for this regulator");
        return -ENOTSUP;
    }

    ret = i2c_reg_read_32(config->i2c, HUDSON_REG_PMIC_SEQ_STAT, &pmic_stat);
    if (ret < 0) {
        LOG_ERR("Failed to read PMIC_STAT register");
        return ret;
    }

    *error_flags = pmic_stat & source_config->fault;

    return 0;
}

static int regulator_hudson_init(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    bool is_enabled;
    int ret;

    regulator_common_data_init(dev);

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    ret = regulator_hudson_is_enabled(dev, &is_enabled);
    if (ret < 0) {
        LOG_ERR("Failed to check if regulator is enabled");
        return ret;
    }

    ret = regulator_hudson_configure_delays(dev);
    if (ret < 0) {
        LOG_ERR("Failed to configure sequence delays");
        return ret;
    }

    return regulator_common_init(dev, is_enabled);
}

static DEVICE_API(regulator, regulator_api) = {
    .enable = regulator_hudson_enable,
    .disable = regulator_hudson_disable,
    .count_voltages = regulator_hudson_count_voltages,
    .list_voltage = regulator_hudson_list_voltages,
    .set_voltage = regulator_hudson_set_voltage,
    .get_voltage = regulator_hudson_get_voltage,
    .count_current_limits = regulator_hudson_count_current_limits,
    .list_current_limit = hudson_regulator_list_current_limit,
    .get_current_limit = regulator_hudson_get_current_limit,
    .set_current_limit = regulator_hudson_set_current_limit,
    .set_mode = regulator_hudson_set_mode,
    .get_mode = regulator_hudson_get_mode,
    .set_active_discharge = regulator_hudson_set_active_discharge,
    .get_active_discharge = regulator_hudson_get_active_discharge,
    .get_error_flags = regulator_hudson_get_error_flags,
};

static int regulator_hudson_pmic_init(const struct device *dev)
{
    const struct regulator_hudson_pmic_config *pmic_config = dev->config;
    uint32_t pmic_stat;
    int ret;

    ret = i2c_reg_read_32(pmic_config->i2c, HUDSON_REG_PMIC_SEQ_STAT, &pmic_stat);
    if (ret < 0) {
        LOG_ERR("Failed to read PMIC_STAT register");
        return ret;
    }

    return 0;
}

static int regulator_hudson_dvs_state_set(const struct device *dev, regulator_dvs_state_t state)
{
    return -ENOTSUP;
}

static int regulator_hudson_ship_mode(const struct device *dev)
{
    return -ENOTSUP;
}

static DEVICE_API(regulator_parent, pmic_api) = {
    .dvs_state_set = regulator_hudson_dvs_state_set,
    .ship_mode = regulator_hudson_ship_mode,
};

#define REGULATOR_HUDSON_DEFINE(node_id, id, source, parent)                                \
    static struct regulator_hudson_data data_##id;                                          \
                                                                                            \
    static const struct regulator_hudson_config config_##id = {                             \
        .common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                                 \
        .i2c = I2C_DT_SPEC_INST_GET(parent),                                                \
        .turn_on_us = DT_PROP_OR(node_id, adi_turn_on_delay_us, 0),                         \
        .turn_off_us = DT_PROP_OR(node_id, adi_turn_off_delay_us, 0),                       \
        .rok_us = DT_PROP_OR(node_id, adi_rok_delay_us, 0),                                 \
        .trim_ilr = DT_PROP_OR(node_id, adi_trim_ilr, 0),                                   \
        .trim_vreg = DT_PROP_OR(node_id, adi_trim_vreg, 0),                                 \
        .desc = &regulator_hudson_desc_list[source],                                        \
    };                                                                                      \
                                                                                            \
    DEVICE_DT_DEFINE(node_id, regulator_hudson_init, NULL, &data_##id, &config_##id,        \
            POST_KERNEL, CONFIG_REGULATOR_HUDSON_INIT_PRIORITY, &regulator_api);

#define REGULATOR_HUDSON_PMIC_DEFINE(inst)                                                  \
    static const struct regulator_hudson_pmic_config config_##inst = {                      \
        .dev = DEVICE_DT_GET(DT_DRV_INST(inst)),                                            \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                  \
    };                                                                                      \
                                                                                            \
    DEVICE_DT_INST_DEFINE(inst, regulator_hudson_pmic_init, NULL, NULL,                     \
            &config_##inst , POST_KERNEL, CONFIG_REGULATOR_HUDSON_INIT_PRIORITY,            \
            &pmic_api);                                                                     \

#define REGULATOR_HUDSON_DEFINE_PMIC(inst)                                                  \
    REGULATOR_HUDSON_PMIC_DEFINE(inst)

#define REGULATOR_HUDSON_DEFINE_REGULATOR(inst, child, source)                              \
    COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),                                 \
        (REGULATOR_HUDSON_DEFINE(DT_INST_CHILD(inst, child), child##inst, source, inst)),   \
        ())

#define REGULATOR_HUDSON_DEFINE_ALL(inst)                                                   \
    REGULATOR_HUDSON_DEFINE_PMIC(inst)                                                      \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, buck0, HUDSON_SOURCE_BUCK0)                     \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, buck1, HUDSON_SOURCE_BUCK1)                     \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, buck2, HUDSON_SOURCE_BUCK2)                     \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, ldo0, HUDSON_SOURCE_LDO0)                       \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, ldo1, HUDSON_SOURCE_LDO1)                       \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, ldo2, HUDSON_SOURCE_LDO2)                       \
    REGULATOR_HUDSON_DEFINE_REGULATOR(inst, ldo3, HUDSON_SOURCE_LDO3)

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_HUDSON_DEFINE_ALL)