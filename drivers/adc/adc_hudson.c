/**
 * @file
 * @brief Hudson ADC Driver
 *
 * This file contains the implementation of the Hudson ADC driver, which provides
 * support for analog-to-digital conversions using the Hudson ADC hardware.
 */

/**
 * @defgroup hudson_adc_driver Hudson ADC Driver
 * @ingroup adc_interface
 * @{
 */

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(ADC_HUDSON, CONFIG_ADC_LOG_LEVEL);

#define DT_DRV_COMPAT adi_hudson_adc
/** ADC resolution in bits */
#define HUDSON_ADC_RESOLUTION 12

/** Hudson ADC register definitions */
#define HUDSON_REG_CTRL                    0x00  /**< Control register */
#define HUDSON_REG_T_AUTO                  0x04  /**< Auto trigger timing register */
#define HUDSON_REG_T_SAMPLE                0x08  /**< Sample timing register */
#define HUDSON_REG_VCM                     0x10  /**< Common-mode voltage register */
#define HUDSON_REG_IBAT                    0x14  /**< Battery current register */
#define HUDSON_REG_TEMPSENS                0x18  /**< Temperature sensor register */
#define HUDSON_REG_VTHERM                  0x1C  /**< Thermistor voltage register */
#define HUDSON_REG_VTHMBIAS                0x20  /**< Thermistor bias voltage register */
#define HUDSON_REG_VBATT                   0x24  /**< Battery voltage register */
#define HUDSON_REG_SAR_CTRL                0x30  /**< SAR control register */
#define HUDSON_REG_SAR_RSTB                0x34  /**< SAR reset register */
#define HUDSON_REG_LFSR_CTRL               0x38  /**< LFSR control register */
#define HUDSON_REG_LFSR_STAT               0x3C  /**< LFSR status register */
#define HUDSON_REG_MEAS_VCM                0x40  /**< Measured common-mode voltage register */
#define HUDSON_REG_MEAS_IBAT               0x44  /**< Measured battery current register */
#define HUDSON_REG_MEAS_TEMPSENS           0x48  /**< Measured temperature sensor register */
#define HUDSON_REG_MEAS_VTHERM             0x4C  /**< Measured thermistor voltage register */
#define HUDSON_REG_MEAS_VTHMBIAS           0x50  /**< Measured thermistor bias voltage register */
#define HUDSON_REG_MEAS_VBATT              0x54  /**< Measured battery voltage register */
#define HUDSON_REG_MEAS_MOISTURE           0x58  /**< Measured moisture register */
#define HUDSON_REG_RAW_MEAS_VCM            0x60  /**< Raw measured common-mode voltage register */
#define HUDSON_REG_RAW_MEAS_IBAT           0x64  /**< Raw measured battery current register */
#define HUDSON_REG_RAW_MEAS_TEMPSENS       0x68  /**< Raw measured temperature sensor register */
#define HUDSON_REG_RAW_MEAS_VTHERM         0x6C  /**< Raw measured thermistor voltage register */
#define HUDSON_REG_RAW_MEAS_VTHMBIAS       0x70  /**< Raw measured thermistor bias voltage register */
#define HUDSON_REG_RAW_MEAS_VBATT          0x74  /**< Raw measured battery voltage register */
#define HUDSON_REG_RAW_MEAS_MOISTURE       0x78  /**< Raw measured moisture register */
#define HUDSON_REG_GAIN_1                  0x80  /**< Gain configuration register 1 */
#define HUDSON_REG_GAIN_2                  0x84  /**< Gain configuration register 2 */
#define HUDSON_REG_TEMP_CONV               0x88  /**< Temperature conversion register */
#define HUDSON_REG_IRQ_MASK                0x90  /**< IRQ mask register */
#define HUDSON_REG_IRQ_STAT                0x94  /**< IRQ status register */

/** ADC configuration macros */
#define HUDSON_ADC_REG_CFG(ch) HUDSON_REG_VCM + (ch * 4) /**< ADC configuration register for a channel */
#define HUDSON_ADC_REG_SAMPLE_RAW(ch) HUDSON_REG_RAW_MEAS_VCM + (ch * 4) /**< Raw sample register for a channel */
#define HUDSON_ADC_REG_SAMPLE(ch) HUDSON_REG_MEAS_VCM + (ch * 4) /**< Processed sample register for a channel */

/** ADC control macros */
#define TRIGGER_MEASUREMENT         BIT(0) /**< Trigger measurement bit */
#define TRIGGER_MEASUREMENT_EN      BIT(0) /**< Enable measurement trigger */
#define TRIGGER_MEASUREMENT_DIS     0      /**< Disable measurement trigger */
#define IRQ_STAT_DONE               BIT(0) /**< IRQ status: measurement done */

/**
 * @brief Hudson ADC channel configuration structure.
 *
 * This structure defines the configuration for a single ADC channel.
 */
struct hudson_adc_channel_config {
    uint32_t channel_id;    /**< Channel ID */
    uint32_t gain;          /**< Gain configuration */
    uint32_t oversampling;  /**< Oversampling configuration */
};

/**
 * @brief Hudson ADC device configuration structure.
 *
 * This structure defines the static configuration for the Hudson ADC device.
 */
struct hudson_adc_config {
    const struct hudson_adc_channel_config *channels; /**< Array of channel configurations */
    struct i2c_dt_spec i2c;                           /**< I2C device specification */
    uint8_t num_channels;                             /**< Number of ADC channels */
    uint32_t vref_mv;                                 /**< Reference voltage in millivolts */
};

/**
 * @brief Hudson ADC runtime data structure.
 *
 * This structure holds runtime data for the Hudson ADC device.
 */
struct hudson_adc_data {
    const struct device *dev; /**< Pointer to the device structure */
    struct adc_context ctx;   /**< ADC context for managing conversions */
    uint32_t oversampling;    /**< Oversampling configuration */
    uint16_t *buffer;         /**< Pointer to the ADC result buffer */
    uint16_t *repeat_buffer;  /**< Pointer to the repeat buffer */
    uint8_t channels;         /**< Active channels for sampling */
    struct k_thread thread;   /**< Thread for ADC acquisition */
    struct k_sem sem;         /**< Semaphore for synchronization */

    K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_HUDSON_THREAD_STACK_SIZE); /**< Thread stack */
};

 /**
  * @brief Read a 32-bit value from an ADC register.
  *
  * Reads a 32-bit value from the specified ADC register over I2C.
  *
  * @param dev Pointer to the device structure.
  * @param reg Register address to read from.
  * @param value Pointer to store the read value.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_read_reg(const struct device *dev, uint8_t reg,
    uint32_t *value)
{
    const struct hudson_adc_config *config = dev->config;
    uint8_t buf[4];
    int ret;

    ret = i2c_burst_read_dt(&config->i2c, reg, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read register %d", reg);
        return ret;
    }

    *value = sys_get_be32(buf);

    return 0;
}

 /**
  * @brief Write a 32-bit value to an ADC register.
  *
  * Writes a 32-bit value to the specified ADC register over I2C.
  *
  * @param dev Pointer to the device structure.
  * @param reg Register address to write to.
  * @param value Value to write.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_write_reg(const struct device *dev, uint8_t reg,
    uint32_t value)
{
    const struct hudson_adc_config *config = dev->config;
    uint8_t buf[4];

    sys_put_be32(value, &buf[0]);

    return i2c_burst_write_dt(&config->i2c, reg, buf, sizeof(buf));
}

 /**
  * @brief Update specific bits in an ADC register.
  *
  * Reads the current value of the register, modifies the specified bits, and writes it back.
  *
  * @param dev Pointer to the device structure.
  * @param reg Register address to update.
  * @param mask Bit mask for the bits to update.
  * @param value New value for the specified bits.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_write_reg_mask(const struct device *dev, uint8_t reg,
    uint32_t mask, uint32_t value)
{
    uint32_t reg_value;
    int ret;

    ret = hudson_adc_read_reg(dev, reg, &reg_value);
    if (ret < 0) {
        return ret;
    }

    reg_value &= ~mask;
    reg_value |= value & mask;

    return hudson_adc_write_reg(dev, reg, reg_value);
}

 /**
  * @brief Set the gain for an ADC channel.
  *
  * Configures the gain for the specified ADC channel.
  *
  * @param dev Pointer to the device structure.
  * @param channel_cfg Pointer to the channel configuration structure.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_channel_set_gain(const struct device *dev,
    const struct adc_channel_cfg *channel_cfg)
{
    uint32_t mask, gain_value;
    uint8_t reg;

    switch (channel_cfg->gain) {
        case ADC_GAIN_1_7:
            gain_value = 0x17;
            break;
        case ADC_GAIN_1_4:
            gain_value = 0x14;
            break;
        case ADC_GAIN_1_2:
            gain_value = 0x12;
            break;
        case ADC_GAIN_1:
            gain_value = 0x1;
            break;
        case ADC_GAIN_2:
            gain_value = 0x2;
            break;
        case ADC_GAIN_4:
            gain_value = 0x4;
            break;
        case ADC_GAIN_6:
            gain_value = 0x6;
            break;
        case ADC_GAIN_8:
            gain_value = 0x8;
            break;
        case ADC_GAIN_10:
            gain_value = 0xA;
            break;
        case ADC_GAIN_12:
            gain_value = 0xC;
            break;
        case ADC_GAIN_14:
            gain_value = 0xE;
            break;
        default:
            LOG_ERR("Unsupported gain value");
            return -EINVAL;
    }

    reg = channel_cfg->channel_id < 4 ? HUDSON_REG_GAIN_1 : HUDSON_REG_GAIN_2;
    mask = GENMASK((channel_cfg->channel_id % 4 * 4) + 3, channel_cfg->channel_id % 4 * 4);

    return hudson_adc_write_reg_mask(dev, reg, mask, gain_value);
}

 /**
  * @brief Set up an ADC channel.
  *
  * Configures the specified ADC channel with the provided settings.
  *
  * @param dev Pointer to the device structure.
  * @param channel_cfg Pointer to the channel configuration structure.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
    int ret;

    if (channel_cfg->channel_id > 7 ) {
        LOG_ERR("Invalid channel id '%d'", channel_cfg->channel_id);
        return -EINVAL;
    }

    ret = hudson_adc_channel_set_gain(dev, channel_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to set gain for channel %d", channel_cfg->channel_id);
        return ret;
    }

    return 0;
}

 /**
  * @brief Read a sample from an ADC channel.
  *
  * Reads a sample from the specified ADC channel.
  *
  * @param dev Pointer to the device structure.
  * @param channel Channel ID to read from.
  * @param result Pointer to store the ADC result.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_channel_read(const struct device *dev, uint8_t channel, uint16_t *result)
{
    struct hudson_adc_data *data = dev->data;
    uint32_t value, reg;
    int ret;

    reg = data->oversampling > 0 ? HUDSON_ADC_REG_SAMPLE_RAW(channel) : HUDSON_ADC_REG_SAMPLE(channel);
    ret = hudson_adc_read_reg(data->dev, reg, &value);
    if (ret < 0) {
        LOG_ERR("Failed to read channel %d", channel);
    }

    *result = (uint16_t)(value & 0xFFF);

    return ret;

}

 /**
  * @brief Validate the buffer size for an ADC sequence.
  *
  * Ensures that the buffer size is sufficient for the requested ADC sequence.
  *
  * @param dev Pointer to the device structure.
  * @param sequence Pointer to the ADC sequence structure.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_validate_buffer_size(const struct device *dev,
                                           const struct adc_sequence *sequence)
{
    uint8_t channels;
    size_t needed;

    channels = POPCOUNT(sequence->channels);
    needed = channels * sizeof(uint16_t);

    if (sequence->buffer_size < needed) {
            return -ENOMEM;
    }

    return 0;
}

/**
 * @brief Start ADC sampling.
 *
 * Starts the ADC sampling process for the specified ADC context.
 *
 * @param ctx Pointer to the ADC context structure.
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct hudson_adc_data *data = CONTAINER_OF(ctx, struct hudson_adc_data, ctx);

    data->channels = ctx->sequence.channels;
    data->buffer = ctx->sequence.buffer;
    data->repeat_buffer = data->buffer;
    data->oversampling = ctx->sequence.oversampling;

    k_sem_give(&data->sem);
}

/**
 * @brief Update the ADC buffer pointer.
 *
 * Updates the buffer pointer for the ADC context, depending on whether
 * repeat sampling is enabled.
 *
 * @param ctx Pointer to the ADC context structure.
 * @param repeat_sampling Indicates if repeat sampling is enabled.
 */
static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
    struct hudson_adc_data *data = CONTAINER_OF(ctx, struct hudson_adc_data, ctx);

    if (repeat_sampling) {
            data->buffer = data->repeat_buffer;
    }
}

/**
 * @brief ADC acquisition thread.
 *
 * This thread handles the ADC acquisition process, including reading
 * samples from the ADC channels and managing the sampling sequence.
 *
 * @param data Pointer to the ADC runtime data structure.
 */
static void hudson_adc_acquisition_thread(struct hudson_adc_data *data)
{
    uint32_t status;
    uint16_t result;
    uint8_t channel;
    int timeout = 100;
    int ret;

    while (true) {
        k_sem_take(&data->sem, K_FOREVER);

        while (data->channels != 0) {
            do {
                ret = hudson_adc_read_reg(data->dev, HUDSON_REG_IRQ_STAT, &status);
                if (ret < 0) {
                    LOG_ERR("Failed to read IRQ_STAT_REG");
                    adc_context_complete(&data->ctx, ret);
                    return;
                }

                if (status & IRQ_STAT_DONE) {
                    break;
                }

                k_busy_wait(10);
            } while (--timeout > 0);

            if (timeout == 0) {
                LOG_ERR("Timeout waiting for IRQ_STAT_DONE");
                adc_context_complete(&data->ctx, -ETIMEDOUT);
                break;
            }

            channel = find_lsb_set(data->channels) - 1;

            ret = hudson_adc_channel_read(data->dev, channel, &result);
            if (ret < 0) {
                    LOG_ERR("failed to read channel %d (ret %d)", channel, ret);
                    adc_context_complete(&data->ctx, ret);
                    break;
            }

            *data->buffer++ = result;
            WRITE_BIT(data->channels, channel, 0);
        }

        adc_context_on_sampling_done(&data->ctx, data->dev);
    }
}

 /**
  * @brief Initialize the Hudson ADC driver.
  *
  * Initializes the Hudson ADC driver, including setting up the I2C bus and ADC context.
  *
  * @param dev Pointer to the device structure.
  * @return 0 on success, negative error code on failure.
  */
static int hudson_adc_init(const struct device *dev)
{
    const struct hudson_adc_config *config = dev->config;
    struct hudson_adc_data *data = dev->data;
        k_tid_t tid;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    data->dev = dev;
    k_sem_init(&data->sem, 0, 1);
    adc_context_init(&data->ctx);

    tid = k_thread_create(&data->thread, data->stack,
            K_KERNEL_STACK_SIZEOF(data->stack),
            (k_thread_entry_t)hudson_adc_acquisition_thread, data, NULL, NULL,
            CONFIG_ADC_HUDSON_THREAD_PRIO, 0, K_NO_WAIT);

            if (IS_ENABLED(CONFIG_THREAD_NAME)) {
        int ret = k_thread_name_set(tid, "adc_hudson");
        if (ret < 0) {
            LOG_ERR("Failed to set thread name");
            return ret;
        }
    }

        adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}

/**
 * @brief Start ADC conversion.
 *
 * Starts the ADC conversion process for the specified ADC sequence.
 *
 * @param dev Pointer to the device structure.
 * @param sequence Pointer to the ADC sequence structure.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_adc_conversion_start(const struct device *dev, const struct adc_sequence *sequence)
{
    const struct hudson_adc_config *config = dev->config;
    struct hudson_adc_data *data = dev->data;
    int ret;

    ret = hudson_adc_validate_buffer_size(dev, sequence);
    if (ret < 0) {
        LOG_ERR("Invalid buffer size");
        return ret;
    }

    if (sequence->resolution != HUDSON_ADC_RESOLUTION) {
        LOG_ERR("Invalid resolution %d", sequence->resolution);
        return -EINVAL;
    }

    for (uint8_t i = 0; i < config->num_channels; i++) {
        if ((BIT(i) & sequence->channels) != 0) {
            if (!(data->channels & BIT(i))) {
                ret = hudson_adc_write_reg_mask(dev, HUDSON_ADC_REG_CFG(i), BIT(8), BIT(8));
                if (ret < 0) {
                    LOG_ERR("Failed to enable channel %d", i);
                    return ret;
                }
            }
            if (sequence->oversampling >= 0 && sequence->oversampling <= 5) {
                if (sequence->oversampling != data->oversampling && sequence->oversampling != 0) {
                    ret = hudson_adc_write_reg_mask(dev, HUDSON_ADC_REG_CFG(i), sequence->oversampling, GENMASK(2, 0));
                    if (ret < 0) {
                        LOG_ERR("Failed to set oversampling for channel %d", i);
                        return ret;
                    }
                }
            } else {
                LOG_ERR("Invalid oversampling value %d", sequence->oversampling);
                return -EINVAL;
            }
        } else {
            if (!(data->channels & BIT(i))) {
                continue;
            }
            ret = hudson_adc_write_reg_mask(dev, HUDSON_ADC_REG_CFG(i), BIT(8), 0);
            if (ret < 0) {
                LOG_ERR("Failed to configure channel %d", i);
                return ret;
            }
        }
    }

    ret = hudson_adc_write_reg_mask(dev, HUDSON_REG_CTRL, TRIGGER_MEASUREMENT, TRIGGER_MEASUREMENT_EN);
    data->buffer = sequence->buffer;

        adc_context_start_read(&data->ctx, sequence);

    return adc_context_wait_for_completion(&data->ctx);
}

/**
 * @brief Perform an asynchronous ADC read.
 *
 * Starts an asynchronous ADC read operation for the specified ADC sequence.
 *
 * @param dev Pointer to the device structure.
 * @param sequence Pointer to the ADC sequence structure.
 * @param async Pointer to the k_poll_signal structure for asynchronous notification.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
                                 struct k_poll_signal *async)
{
    struct hudson_adc_data *data = dev->data;
    int ret;

    adc_context_lock(&data->ctx, async ? true : false, async);
    ret = hudson_adc_conversion_start(dev, sequence);
    adc_context_release(&data->ctx, ret);

    return ret;
}

/**
 * @brief Perform a synchronous ADC read.
 *
 * Starts a synchronous ADC read operation for the specified ADC sequence.
 *
 * @param dev Pointer to the device structure.
 * @param sequence Pointer to the ADC sequence structure.
 * @return 0 on success, negative error code on failure.
 */
static int hudson_adc_read(const struct device *dev, const struct adc_sequence *sequence)
{
    return hudson_adc_read_async(dev, sequence, NULL);
}
 /** @} */ /* End of hudson_adc_driver group */

#define HUDSON_ADC_CHANNEL(child)                                                                   \
    {                                                                                               \
        .channel_id = DT_REG_ADDR(child),                                                           \
        .gain = 0,                                                                                  \
        .oversampling = 0,                                                                          \
    },

#define HUDSON_ADC_DEFINE(inst)                                                                     \
    static DEVICE_API(adc, hudson_adc_api_##inst) = {                                               \
        .channel_setup = hudson_adc_channel_setup,                                                  \
        .ref_internal = DT_INST_PROP(inst, vref_mv),                                                \
        .read = hudson_adc_read,                                                                    \
        IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = hudson_adc_read_async))                         \
    };                                                                                              \
                                                                                                    \
    static const struct hudson_adc_channel_config hudson_adc_channels_##inst[] = {                  \
        DT_INST_FOREACH_CHILD(inst, HUDSON_ADC_CHANNEL)                                             \
    };                                                                                              \
                                                                                                    \
    static const struct hudson_adc_config hudson_adc_config_##inst = {                              \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                          \
        .channels = hudson_adc_channels_##inst,                                                     \
        .num_channels = ARRAY_SIZE(hudson_adc_channels_##inst),                                     \
        .vref_mv = DT_INST_PROP(inst, vref_mv),                                                     \
    };                                                                                              \
                                                                                                    \
    static struct hudson_adc_data hudson_adc_data_##inst;                                           \
                                                                                                    \
    DEVICE_DT_INST_DEFINE(inst, hudson_adc_init, NULL,                                              \
                          &hudson_adc_data_##inst, &hudson_adc_config_##inst,                       \
                          POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &hudson_adc_api_##inst);

DT_INST_FOREACH_STATUS_OKAY(HUDSON_ADC_DEFINE)
