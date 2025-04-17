/*
 * Copyright (c) 2025 Analog Devices
 * SPDX-License-Identifier: Apache-2.0
 */

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
/**
 * @file
 * @brief Driver implementation for Hudson PMIC driver.
 *
 * This file contains the implementation of the driver for Hudson PMIC driver.
 * It provides functions to initialize, configure, and interact with the PMIC and its regulators.
 */

/**
 * @defgroup hudson_pmic_registers Hudson PMIC Registers
 * @ingroup hudson_pmic
 * @{
 */

/** PMIC Sequence Status Register */
#define HUDSON_REG_PMIC_SEQ_STAT            0x00

/** DCLM Delay Register */
#define HUDSON_REG_DCLM_DLY                 0x04

/** DCLM Configuration Register */
#define HUDSON_REG_DCLM_CFG                 0x08

/** DCRF Configuration Register */
#define HUDSON_REG_DCRF_CFG                 0x0C

/** DCAVH Configuration Register */
#define HUDSON_REG_DCAVH_CFG                0x10

/** CLASSD LDO0 Delay Register */
#define HUDSON_REG_CLASSD_LDO0_DLY          0x14

/** CLASSD LDO1 Delay Register */
#define HUDSON_REG_CLASSD_LDO1_DLY          0x18

/** ADC LDO2 Delay Register */
#define HUDSON_REG_ADC_LDO2_DLY             0x1C

/** 1V2 LDO3 Delay Register */
#define HUDSON_REG_1V2_LDO3_DLY             0x20

/** PMIC Control Register */
#define HUDSON_REG_PMIC_CTRL                0x24

/** @} */

/**
 * @defgroup hudson_pmic_registers Hudson PMIC Registers
 * @ingroup hudson_pmic
 * @{
 */

/** @name PMIC_STAT Register (0x00)
 *  @brief PMIC Status Register bit definitions.
 *  @{
 */
 #define HUDSON_PMIC_STAT_FAULT_PMIC         BIT(10) /**< Fault detected: PMIC */
 #define HUDSON_PMIC_STAT_FAULT_1V2          BIT(9)  /**< Fault detected: 1V2 LDO */
 #define HUDSON_PMIC_STAT_FAULT_ADC          BIT(8)  /**< Fault detected: ADC LDO */
 #define HUDSON_PMIC_STAT_FAULT_CLASSD1      BIT(7)  /**< Fault detected: CLASSD1 LDO */
 #define HUDSON_PMIC_STAT_FAULT_CLASSD0      BIT(6)  /**< Fault detected: CLASSD0 LDO */
 #define HUDSON_PMIC_STAT_FAULT_DCLM         BIT(5)  /**< Fault detected: DCLM DCDC */
 #define HUDSON_PMIC_STAT_EN_1V2_STAT        BIT(4)  /**< 1V2 LDO enabled status */
 #define HUDSON_PMIC_STAT_EN_ADC_STAT        BIT(3)  /**< ADC LDO enabled status */
 #define HUDSON_PMIC_STAT_EN_CLASSD1_STAT    BIT(2)  /**< CLASSD1 LDO enabled status */
 #define HUDSON_PMIC_STAT_EN_CLASSD0_STAT    BIT(1)  /**< CLASSD0 LDO enabled status */
 #define HUDSON_PMIC_STAT_EN_DCLM_STAT       BIT(0)  /**< DCLM DCDC enabled status */
 /** @} */

 /** @name DCDC_CONFIG Register (0x08, 0x0C, 0x10)
  *  @brief DCDC Configuration Register bit definitions.
  *  @{
  */
 #define HUDSON_DCDC_CONFIG_FLIM_MODE        BIT(0)          /**< Flim mode */
 #define HUDSON_DCDC_CONFIG_HIZ              BIT(1)          /**< High impedance mode */
 #define HUDSON_DCDC_CONFIG_IDLE             BIT(2)          /**< Idle mode */
 #define HUDSON_DCDC_CONFIG_TRIM_ILR         GENMASK(5, 3)   /**< Current trim */
 #define HUDSON_DCDC_CONFIG_TRIM_VREG        GENMASK(10, 6)  /**< Voltage trim */
 #define HUDSON_DCDC_CONFIG_VSEL             GENMASK(16, 11) /**< Voltage selection */
 #define HUDSON_DCDC_CONFIG_OUTPUT_MASK      GENMASK(16, 3)  /**< Output configuration mask */
 #define HUDSON_DCDC_MODE_EN_MASK            BIT(3)          /**< Enable mask */
 #define HUDSON_DCDC_MODE_MASK               GENMASK(2, 0)   /**< Mode selection mask */
 /** @} */

 /** @name DLY Register (0x04, 0x14, 0x18, 0x1C, 0x20)
  *  @brief Delay Configuration Register bit definitions.
  *  @{
  */
 #define HUDSON_DLY_CONFIG_OFF               GENMASK(24, 18) /**< Turn-off delay */
 #define HUDSON_DLY_CONFIG_ON                GENMASK(17, 9)  /**< Turn-on delay */
 #define HUDSON_DLY_CONFIG_ROK               GENMASK(8, 0)   /**< Ready-signal delay */
 /** @} */


 /** @name PMIC_CTRL Register (0x24)
  *  @brief PMIC Control Register bit definitions.
  *  @{
  */
  #define HUDSON_PMIC_CTRL_MAN_MODE_1V2       BIT(14) /**< Manual mode for 1V2 LDO */
  #define HUDSON_PMIC_CTRL_MAN_MODE_ADC       BIT(13) /**< Manual mode for ADC LDO */
  #define HUDSON_PMIC_CTRL_MAN_MODE_CLASSD1   BIT(12) /**< Manual mode for CLASSD1 LDO */
  #define HUDSON_PMIC_CTRL_MAN_MODE_CLASSD0   BIT(11) /**< Manual mode for CLASSD0 LDO */
  #define HUDSON_PMIC_CTRL_MAN_MODE_DCLM      BIT(10) /**< Manual mode for DCLM DCDC */
  #define HUDSON_PMIC_CTRL_EN_MAN_1V2         BIT(9)  /**< Enable manual mode for 1V2 LDO */
  #define HUDSON_PMIC_CTRL_EN_MAN_ADC         BIT(8)  /**< Enable manual mode for ADC LDO */
  #define HUDSON_PMIC_CTRL_EN_MAN_CLASSD1     BIT(7)  /**< Enable manual mode for CLASSD1 LDO */
  #define HUDSON_PMIC_CTRL_EN_MAN_CLASSD0     BIT(6)  /**< Enable manual mode for CLASSD0 LDO */
  #define HUDSON_PMIC_CTRL_EN_MAN_DCLM        BIT(5)  /**< Enable manual mode for DCLM DCDC */
  #define HUDSON_PMIC_CTRL_ARM_EN_1V2         BIT(4)  /**< Enable 1V2 LDO */
  #define HUDSON_PMIC_CTRL_ARM_EN_ADC         BIT(3)  /**< Enable ADC LDO */
  #define HUDSON_PMIC_CTRL_ARM_EN_CLASSD1     BIT(2)  /**< Enable CLASSD1 LDO */
  #define HUDSON_PMIC_CTRL_ARM_EN_CLASSD0     BIT(1)  /**< Enable CLASSD0 LDO */
  #define HUDSON_PMIC_CTRL_ARM_EN_DCLM        BIT(0)  /**< Enable DCLM DCDC */
  /** @} */

 /** @} */ /* End of hudson_pmic_registers group */

/**
    * @name Hudson regulator fixed current limit
    * @{
    */
/** Current limit */
 #define HUDSON_FIXED_CURRENT_LIMIT_UA  1000000 /**< Fixed current limit in microamps */

/**
 * @defgroup hudson_pmic_data_structures Hudson PMIC Data Structures
 * @ingroup hudson_pmic
 * @{
 */

/**
 * @enum hudson_sources
 * @brief Enumerates the available power sources in the Hudson PMIC.
 *
 * This enumeration defines the various power sources supported by the Hudson PMIC,
 * including buck converters and LDOs.
 */
 enum hudson_sources {
    HUDSON_SOURCE_BUCK0 = 0,    /**< DCLM Buck Converter */
    HUDSON_SOURCE_BUCK1,        /**< DCRF Buck Converter */
    HUDSON_SOURCE_BUCK2,        /**< DCAVH Buck Converter */
    HUDSON_SOURCE_LDO0,         /**< CLASSD0 LDO */
    HUDSON_SOURCE_LDO1,         /**< CLASSD1 LDO */
    HUDSON_SOURCE_LDO2,         /**< ADC LDO */
    HUDSON_SOURCE_LDO3,         /**< 1V2 LDO */
    HUDSON_NUM_SOURCES,         /**< Total number of sources */
};

/**
 * @struct regulator_hudson_data
 * @brief Runtime data for the Hudson PMIC regulator.
 *
 * This structure holds runtime data for the Hudson PMIC regulator, including
 * common regulator data.
 */
struct regulator_hudson_data {
    struct regulator_common_data data; /**< Common regulator runtime data */
};

/**
 * @struct regulator_hudson_config
 * @brief Configuration data for a Hudson PMIC regulator.
 *
 * This structure holds the configuration data for a Hudson PMIC regulator,
 * including I2C details, voltage trim settings, and delay configurations.
 */
struct regulator_hudson_config {
    struct regulator_common_config common; /**< Common regulator configuration */
    struct i2c_dt_spec i2c;                /**< I2C device specification */
    const struct regulator_hudson_desc *desc; /**< Pointer to the regulator descriptor */
    uint32_t trim_ilr;                     /**< Current trim value */
    uint32_t trim_vreg;                    /**< Voltage trim value */
    uint32_t turn_on_us;                   /**< Turn-on delay in microseconds */
    uint32_t turn_off_us;                  /**< Turn-off delay in microseconds */
    uint32_t rok_us;                       /**< Ready delay in microseconds */
};

/**
 * @struct regulator_hudson_source_config
 * @brief Configuration for a specific Hudson PMIC power source.
 *
 * This structure defines the configuration for a specific power source in the
 * Hudson PMIC, including control registers, status, and fault handling.
 */
struct regulator_hudson_source_config {
    enum hudson_sources source; /**< Power source identifier */
    uint32_t control_reg;       /**< Control register address */
    uint32_t sequence_reg;      /**< Sequence register address */
    uint32_t enable;            /**< Enable bit mask */
    uint32_t manual_override;   /**< Manual override bit mask */
    uint32_t status;            /**< Status bit mask */
    uint32_t fault;             /**< Fault bit mask */
};

/**
 * @struct regulator_hudson_desc
 * @brief Descriptor for a Hudson PMIC regulator.
 *
 * This structure provides a descriptor for a Hudson PMIC regulator, including
 * its source configuration and supported voltage ranges.
 */
struct regulator_hudson_desc {
    struct regulator_hudson_source_config source_config; /**< Source configuration */
    uint8_t num_ranges;                                  /**< Number of supported voltage ranges */
    const struct linear_range *ranges;                  /**< Pointer to voltage ranges */
};

/**
 * @struct regulator_hudson_pmic_config
 * @brief Configuration for the Hudson PMIC device.
 *
 * This structure defines the configuration for the Hudson PMIC device, including
 * the I2C device and DVS state.
 */
struct regulator_hudson_pmic_config {
    const struct device *dev;       /**< Pointer to the PMIC device */
    struct i2c_dt_spec i2c;         /**< I2C device specification */
    regulator_dvs_state_t dvs_state; /**< Dynamic Voltage Scaling (DVS) state */
};
/** @} */

/** @} */ /* End of hudson_pmic_data_structures group */

/**
 * @brief Voltage ranges for Hudson PMIC regulators.
 *
 * This array defines the supported voltage ranges for each regulator in the Hudson PMIC.
 * Each entry corresponds to a specific regulator source.
 */
static const struct linear_range linear_ranges[7][1] = {
    /* DCLM 0.5V to 0.75V */
    [HUDSON_SOURCE_BUCK0] = {
        LINEAR_RANGE_INIT(500000, 100000U, 0x0U, 0x0BU)
    },
    /*  DCRF 0.94V */
    [HUDSON_SOURCE_BUCK1] = {
        LINEAR_RANGE_INIT(940000, 0, 0, 0),
    },
    /* DCAVH 1.9V */
    [HUDSON_SOURCE_BUCK2] = {
        LINEAR_RANGE_INIT(1900000, 0, 0, 0),
    },
    /* CLASSD0 1.8V */
    [HUDSON_SOURCE_LDO0] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    /* CLASSD1 1.8V */
    [HUDSON_SOURCE_LDO1] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    /* ADC 1.8V */
    [HUDSON_SOURCE_LDO2] = {
        LINEAR_RANGE_INIT(1800000, 0, 0, 0),
    },
    /* 1V2 1.2V */
    [HUDSON_SOURCE_LDO3] = {
        LINEAR_RANGE_INIT(1200000, 0, 0, 0),
    },
};

/**
 * @brief Descriptors for Hudson PMIC regulators.
 *
 * This array defines the configuration and supported features for each regulator
 * in the Hudson PMIC. Each entry corresponds to a specific regulator source.
 */
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

/**
 * @brief Read a 32-bit value from an I2C register.
 *
 * Reads a 32-bit value from the specified I2C register.
 *
 * @param i2c I2C device specification.
 * @param reg Register address to read from.
 * @param val Pointer to store the read value.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Write a 32-bit value to an I2C register.
 *
 * Writes a 32-bit value to the specified I2C register.
 *
 * @param i2c I2C device specification.
 * @param reg Register address to write to.
 * @param val Value to write.
 * @return 0 on success, negative error code on failure.
 */
static int i2c_reg_write_32(struct i2c_dt_spec i2c, uint8_t reg, uint32_t val)
{
    uint8_t buf[4];

    sys_put_be32(val, &buf[0]);

    return i2c_burst_write_dt(&i2c, reg, buf, sizeof(buf));
}

/**
 * @brief Update specific bits in a 32-bit I2C register.
 *
 * Reads the current value of the register, modifies the specified bits, and writes it back.
 *
 * @param i2c I2C device specification.
 * @param reg Register address to update.
 * @param mask Bit mask for the bits to update.
 * @param val New value for the specified bits.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Check if a regulator is enabled.
 *
 * Reads the PMIC status register to determine if the regulator is enabled.
 *
 * @param dev Pointer to the device structure.
 * @param is_enabled Pointer to store the enabled status.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Configure delays for a regulator.
 *
 * Configures the turn-on, turn-off, and ready-on delays for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Enable a regulator.
 *
 * Enables the specified regulator by updating the PMIC control register.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Disable a regulator.
 *
 * Disables the specified regulator by updating the PMIC control register.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Count the number of supported voltage levels.
 *
 * Returns the number of supported voltage levels for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @return Number of supported voltage levels.
 */
static unsigned int regulator_hudson_count_voltages(const struct device *dev)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;

    return linear_range_group_values_count(desc->ranges, desc->num_ranges);
}

/**
 * @brief List a specific voltage level.
 *
 * Retrieves the voltage level corresponding to the specified index.
 *
 * @param dev Pointer to the device structure.
 * @param idx Index of the voltage level.
 * @param volt_uv Pointer to store the voltage level in microvolts.
 * @return 0 on success, negative error code on failure.
 */
static int regulator_hudson_list_voltages(const struct device *dev, unsigned int idx,
    int32_t *volt_uv)
{
    const struct regulator_hudson_config *config = dev->config;
    const struct regulator_hudson_desc *desc = config->desc;

	return linear_range_group_get_value(desc->ranges, desc->num_ranges,
                                        idx, volt_uv);
}

/**
 * @brief Set the output voltage of a regulator.
 *
 * Sets the output voltage of the regulator within the specified range.
 *
 * @param dev Pointer to the device structure.
 * @param min_uv Minimum voltage in microvolts.
 * @param max_uv Maximum voltage in microvolts.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Get the current output voltage of a regulator.
 *
 * Reads the current output voltage of the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param voltage Pointer to store the output voltage in microvolts.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Count the number of supported current limits.
 *
 * Returns the number of supported current limits for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @return Number of supported current limits (always 1 for this driver).
 */
static unsigned int regulator_hudson_count_current_limits(const struct device *dev)
{
    return 1;
}

/**
 * @brief List the fixed current limit.
 *
 * Retrieves the fixed current limit for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param idx Index of the current limit (must be 0).
 * @param current_ua Pointer to store the current limit in microamps.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_regulator_list_current_limit(const struct device *dev, unsigned int idx,
    int32_t *current_ua)
{
    if (idx != 0) {
        return -EINVAL;
    }

    *current_ua = HUDSON_FIXED_CURRENT_LIMIT_UA;
    return 0;
}

/**
 * @brief Set the current limit.
 *
 * Since the current limit is fixed, this function always returns success
 * if the requested range includes the fixed limit.
 *
 * @param dev Pointer to the device structure.
 * @param min_ua Minimum current in microamps.
 * @param max_ua Maximum current in microamps.
 * @return 0 on success, negative error code on failure.
 */
static int regulator_hudson_set_current_limit(const struct device *dev,
    int32_t min_ua, int32_t max_ua)
{
    if (min_ua <= HUDSON_FIXED_CURRENT_LIMIT_UA &&
        max_ua >= HUDSON_FIXED_CURRENT_LIMIT_UA) {
        return 0;
    }

    return -EINVAL;
}

/**
 * @brief Get the current limit.
 *
 * Retrieves the fixed current limit for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param current_ua Pointer to store the current limit in microamps.
 * @return 0 on success, negative error code on failure.
 */
static int regulator_hudson_get_current_limit(const struct device *dev, int32_t *current_ua)
{
    *current_ua = HUDSON_FIXED_CURRENT_LIMIT_UA;
    return 0;
}

/**
 * @brief Configures the Hudson PMIC regulator mode.
 *
 * This function allows setting the operating mode of the Hudson PMIC regulator.
 * The available modes are defined in the Hudson PMIC Devicetree helpers:
 *
 * - HUDSON_DCDC_MODE_PFM: Buck mode with PWM.
 * - HUDSON_DCDC_MODE_FPWM: Buck mode with FPWM.
 * - HUDSON_DCDC_MODE_IDLE: Idle mode for reduced power consumption.
 * - HUDSON_DCDC_MODE_HIGH_IMPEDANCE: High impedance mode for minimal current draw.
 * - HUDSON_DCDC_MODE_SHUTDOWN: Shutdown mode to disable the regulator.
 *
 * @param mode The desired regulator mode. Use one of the predefined macros:
 *             HUDSON_DCDC_MODE_PFM, HUDSON_DCDC_MODE_FPWM,
 *             HUDSON_DCDC_MODE_IDLE, HUDSON_DCDC_MODE_HIGH_IMPEDANCE,
 *             or HUDSON_DCDC_MODE_SHUTDOWN.
 *
 * @return 0 on success, or a negative error code on failure.
 */
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

/**
 * @brief Get the current operating mode of a regulator.
 *
 * Reads the current operating mode of the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param mode Pointer to store the current operating mode.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Set active discharge for a regulator.
 *
 * Configures the active discharge setting for the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param active_discharge Desired active discharge state.
 * @return -ENOTSUP as active discharge is not supported.
 */
static int regulator_hudson_set_active_discharge(const struct device *dev, bool active_discharge)
{
    return -ENOTSUP;
}

/**
 * @brief Get the active discharge state of a regulator.
 *
 * Reads the active discharge state of the regulator.
 *
 * @param dev Pointer to the device structure.
 * @param active_discharge Pointer to store the active discharge state.
 * @return -ENOTSUP as active discharge is not supported.
 */
static int regulator_hudson_get_active_discharge(const struct device *dev, bool *active_discharge)
{
    return -ENOTSUP;
}

/**
 * @brief Get error flags for a regulator.
 *
 * Reads the error flags for the regulator from the PMIC status register.
 *
 * @param dev Pointer to the device structure.
 * @param error_flags Pointer to store the error flags.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Initialize a Hudson regulator.
 *
 * Initializes the specified regulator, including configuring delays and checking its status.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Initialize the Hudson PMIC device.
 *
 * Initializes the Hudson PMIC device, including reading its status.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
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

/**
 * @brief Set the DVS state for the Hudson PMIC.
 *
 * Configures the Dynamic Voltage Scaling (DVS) state for the PMIC.
 *
 * @param dev Pointer to the device structure.
 * @param state Desired DVS state.
 * @return -ENOTSUP as DVS is not supported.
 */
static int regulator_hudson_dvs_state_set(const struct device *dev, regulator_dvs_state_t state)
{
    return -ENOTSUP;
}

/**
 * @brief Configure the Hudson PMIC for ship mode.
 *
 * Configures the PMIC to enter ship mode.
 *
 * @param dev Pointer to the device structure.
 * @return -ENOTSUP as ship mode is not supported.
 */
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
