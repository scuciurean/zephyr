/*
 * Copyright 2025, Analog Devices
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/**
 * @file
 * @brief Hudson Fuel Gauge Driver
 *
 * This file contains the implementation of the Hudson Fuel Gauge drive
 */

/**
 * @defgroup hudson_fuel_gauge_driver Hudson Fuel Gauge Driver
 * @ingroup fuel_gauge_interface
 * @{
 */

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT adi_hudson_fuel_gauge

LOG_MODULE_REGISTER(HUDSON_GAUGE);

/**
 * @brief Hudson Fuel Gauge channel configuration structure.
 *
 * This structure defines the configuration for a single fuel gauge channel.
 */
struct hudson_fg_channel {
    uint8_t channel_id; /**< Channel ID */
    const char *label;  /**< Channel label */
};

/**
 * @brief Hudson Fuel Gauge runtime data structure.
 *
 * This structure holds runtime data for the Hudson Fuel Gauge device.
 */
struct hudson_fg_data {
    uint16_t adc_vref_mv; /**< ADC reference voltage in millivolts */
};

/**
 * @brief Hudson Fuel Gauge device configuration structure.
 *
 * This structure defines the static configuration for the Hudson Fuel Gauge device.
 */
struct hudson_fg_config {
    struct i2c_dt_spec i2c;                     /**< I2C device specification */
    const struct device *adc;                   /**< Pointer to the ADC device */
    const struct hudson_fg_channel *channels;   /**< Array of channel configurations */
    uint8_t num_channels;                       /**< Number of fuel gauge channels */
};

/**
 * @brief Hudson Fuel Gauge channel IDs.
 *
 * Enumerates the available channels in the Hudson Fuel Gauge hardware.
 */
enum hudson_fg_channel_id {
    HUDSON_FG_CHANNEL_VCM = 0,    /**< Common-mode voltage channel */
    HUDSON_FG_CHANNEL_IBAT,       /**< Battery current channel */
    HUDSON_FG_CHANNEL_TEMPSENS,   /**< Temperature sensor channel */
    HUDSON_FG_CHANNEL_VTHERM,     /**< Thermistor voltage channel */
    HUDSON_FG_CHANNEL_VTHMBIAS,   /**< Thermistor bias voltage channel */
    HUDSON_FG_CHANNEL_VBATT,      /**< Battery voltage channel */
    HUDSON_FG_CHANNEL_MOISTURE,   /**< Moisture sensor channel */
};

/**
 * @brief Read a channel value from the fuel gauge.
 *
 * Reads the raw value of the specified channel from the ADC.
 *
 * @param dev Pointer to the device structure.
 * @param channel_id ID of the channel to read.
 * @param result Pointer to store the raw ADC result.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_fg_read_channel(const struct device *dev, uint8_t channel_id,
    uint16_t *result)
{
    const struct hudson_fg_config *config = dev->config;
    struct adc_sequence sequence = {
        .channels = BIT(channel_id),
        .buffer = result,
        .buffer_size = sizeof(*result),
        .resolution = 12,
    };

    return adc_read(config->adc, &sequence);
}

/**
 * @brief Read a channel value in millivolts.
 *
 * Reads the value of the specified channel from the ADC and converts it to millivolts.
 *
 * @param dev Pointer to the device structure.
 * @param channel_id ID of the channel to read.
 * @param result Pointer to store the result in millivolts.
 * @param oversampling Oversampling configuration.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_fg_read_channel_mv(const struct device *dev,
    uint8_t channel_id, uint16_t *result, uint32_t oversampling)
{
    struct hudson_fg_data *data = dev->data;
    uint32_t conversion_result;
    uint16_t res;
    int ret;

    ret = hudson_fg_read_channel(dev, channel_id, &res);
    if (ret < 0)
        return ret;

    conversion_result = res;
    ret = adc_raw_to_millivolts(data->adc_vref_mv, ADC_GAIN_1, 12,&conversion_result);
    if (ret < 0)
        return ret;

    *result = conversion_result;

    return ret;
}

/**
 * @brief Get a fuel gauge property.
 *
 * Retrieves the value of a specific fuel gauge property.
 *
 * @param dev Pointer to the device structure.
 * @param prop Fuel gauge property to retrieve.
 * @param val Pointer to store the property value.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_fg_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
    union fuel_gauge_prop_val *val)
{
    uint16_t result;
    int ret;

    switch (prop) {

    /** Battery average current (uA); negative=discharging */
    case FUEL_GAUGE_AVG_CURRENT:
        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_IBAT, &result, 0);
        if (ret < 0) {
            return ret;
        }
        /** The result is q13.3 */
        val->avg_current = (result >> 3) * 1000 + ((result & 0x7) * 1000) / 8;
        break;

    /** Battery current (uA); negative=discharging */
    case FUEL_GAUGE_CURRENT:
        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_IBAT, &result, 0);
        if (ret < 0) {
            return ret;
        }
        val->current = (uint16_t)result * 1000;
        break;

    /** Battery voltage (uV) */
    case FUEL_GAUGE_VOLTAGE:
        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_VBATT, &result, 0);
        if (ret < 0) {
            return ret;
        }
        val->voltage = (uint16_t)result * 1000;
        break;

    /** Moisture (g/m^3) */
    case FUEL_GAUGE_MOISTURE:
        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_MOISTURE, &result, 0);
        if (ret < 0) {
            return ret;
        }
        val->moisture = (uint16_t)result;
        break;

    case FUEL_GAUGE_STATUS:
        val->fg_status = 0;
        break;

    default:
        LOG_ERR("Unsupported property type");
        return -ENOTSUP;
    }

    return 0;

}

/**
 * @brief Get a buffer property from the fuel gauge.
 *
 * Retrieves a buffer property (e.g., multiple temperature readings) from the fuel gauge.
 *
 * @param dev Pointer to the device structure.
 * @param prop Fuel gauge property to retrieve.
 * @param dst Pointer to the destination buffer.
 * @param dst_len Length of the destination buffer.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_fg_get_buffer_prop(const struct device *dev,
    fuel_gauge_prop_t prop, void *dst, size_t dst_len)
{
    uint16_t conversion_result;
    int ret;

    switch (prop) {
    /** Battery temperature, die temperature, charger temperature (0.1K)*/
    case FUEL_GAUGE_TEMPERATURE:
        size_t required_size = sizeof(uint16_t) * 3;

        if (dst_len < required_size) {
            LOG_ERR("Insufficient buffer size for temperature readings");
            return -EINVAL;
        }
        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_TEMPSENS, &conversion_result, 0);
        if (ret < 0) {
            return ret;
        }
        ((uint16_t *)dst)[0] = conversion_result;

        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_VTHERM, &conversion_result, 0);
        if (ret < 0) {
            return ret;
        }
        ((uint16_t *)dst)[1] = conversion_result;

        ret = hudson_fg_read_channel_mv(dev, HUDSON_FG_CHANNEL_VTHMBIAS, &conversion_result, 0);
        if (ret < 0) {
            return ret;
        }
        ((uint16_t *)dst)[2] = conversion_result;

        break;

    default:
        LOG_ERR("Unsupported property type");
        return -ENOTSUP;
    }

    return ret;
}

/**
 * @brief Initialize the Hudson Fuel Gauge driver.
 *
 * Initializes the Hudson Fuel Gauge driver, including verifying the readiness of
 * the I2C and ADC devices.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_fg_init(const struct device *dev)
{
    const struct hudson_fg_config *config = dev->config;
    struct hudson_fg_data *data = dev->data;

    if (!device_is_ready(config->i2c.bus)) {
        return -ENODEV;
    }

    if (!device_is_ready(config->adc)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    data->adc_vref_mv = adc_ref_internal(config->adc);

    LOG_INF("Fuel gauge initialized");

    return 0;
}

static DEVICE_API(fuel_gauge, hudson_driver_api) = {
    .get_property = &hudson_fg_get_prop,
    .get_buffer_property = &hudson_fg_get_buffer_prop,
};

#define HUDSON_FG_CHANNEL(inst, prop, idx)                                     \
{                                                                              \
    .channel_id = DT_IO_CHANNELS_INPUT_BY_IDX(inst, idx),                      \
},

#define HUDSON_FG_DEFINE(inst)                                                 \
    static const struct hudson_fg_channel hudson_fg_channels_##inst[] = {      \
        DT_FOREACH_PROP_ELEM(DT_DRV_INST(inst), io_channels, HUDSON_FG_CHANNEL)\
    };                                                                         \
                                                                               \
    static const struct hudson_fg_config hudson_fg_config_##inst = {           \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                     \
        .adc = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), adc)),              \
        .channels = hudson_fg_channels_##inst,                                 \
        .num_channels = DT_PROP_LEN(DT_DRV_INST(inst), io_channels),           \
    };                                                                         \
                                                                               \
    static struct hudson_fg_data hudson_fg_data_##inst;                        \
                                                                               \
    DEVICE_DT_INST_DEFINE(inst, &hudson_fg_init, NULL, &hudson_fg_data_##inst, \
                         &hudson_fg_config_##inst, POST_KERNEL,                \
                         CONFIG_FUEL_GAUGE_INIT_PRIORITY, &hudson_driver_api); \

DT_INST_FOREACH_STATUS_OKAY(HUDSON_FG_DEFINE)

/** @} */ /* End of hudson_fuel_gauge_driver group */