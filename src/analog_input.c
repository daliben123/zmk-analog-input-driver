#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "analog_input.h"

LOG_MODULE_REGISTER(analog_input, CONFIG_ANALOG_INPUT_LOG_LEVEL);

static void sampling_timer_handler(struct k_timer *timer);
static void sampling_work_handler(struct k_work *work);
static void analog_input_async_init(struct k_work *work);
static int analog_input_report_data(const struct device *dev);
static void process_channel_data(const struct device *dev, uint8_t ch_idx, int32_t raw);

/**
 * @brief Initialize the analog input driver asynchronously
 */
static void analog_input_async_init(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, init_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;
    
    LOG_INF("Initializing analog input driver");
    
    /* Allocate memory for ADC buffer */
    data->as_buff = k_malloc(config->io_channels_len * sizeof(uint16_t));
    if (!data->as_buff) {
        LOG_ERR("Failed to allocate memory for ADC buffer");
        return;
    }
    
    data->delta = k_malloc(config->io_channels_len * sizeof(int32_t));
    if (!data->delta) {
        LOG_ERR("Failed to allocate memory for delta buffer");
        k_free(data->as_buff);
        return;
    }
    
    data->prev = k_malloc(config->io_channels_len * sizeof(int32_t));
    if (!data->prev) {
        LOG_ERR("Failed to allocate memory for prev buffer");
        k_free(data->as_buff);
        k_free(data->delta);
        return;
    }
    
    /* Configure ADC sequence */
    struct adc_sequence *as = &data->as;
    memset(as, 0, sizeof(*as));
    
    if (config->mode == ANALOG_INPUT_MODE_SCAN) {
        /* Scan mode configuration */
        LOG_INF("Configuring ADC in scan mode");
        
        as->channels = 0;
        for (uint8_t i = 0; i < config->scan_sequence_len; i++) {
            as->channels |= BIT(config->scan_sequence[i]);
        }
        
        as->buffer = data->as_buff;
        as->buffer_size = config->scan_sequence_len * sizeof(data->as_buff[0]);
        as->oversampling = 0; /* nRF52840 does not support oversampling in scan mode */
        as->calibrate = 0;
        
        /* Set scan sequence options */
        as->options = (struct adc_sequence_options) {
            .scan_sequence = config->scan_sequence,
            .scan_length = config->scan_sequence_len,
        };
    } else {
        /* Single channel mode */
        LOG_INF("Configuring ADC in single channel mode");
        
        as->channels = BIT(config->io_channels[0].adc_channel.channel_id);
        as->buffer = &data->as_buff[0];
        as->buffer_size = sizeof(data->as_buff[0]);
        as->oversampling = 0;
        as->calibrate = 0;
    }
    
    /* Initialize all channels */
    uint32_t ch_mask = 0;
    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        const struct analog_input_io_channel *ch_cfg = &config->io_channels[i];
        const struct device *adc = ch_cfg->adc_channel.dev;
        
        if (!device_is_ready(adc)) {
            LOG_ERR("ADC device %s is not ready", adc->name);
            continue;
        }
        
        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = ch_cfg->adc_channel.channel_id,
            .input_positive = ch_cfg->adc_channel.input_positive,
        };
        
        ch_mask |= BIT(ch_cfg->adc_channel.channel_id);
        
        int err = adc_channel_setup(adc, &channel_cfg);
        if (err < 0) {
            LOG_ERR("Failed to setup ADC channel %d: %d", ch_cfg->adc_channel.channel_id, err);
        }
        
        /* Initialize channel state */
        data->delta[i] = 0;
        data->prev[i] = 0;
    }
    
    /* Initialize sampling timer */
    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);
    k_work_init_delayable(&data->init_work, analog_input_async_init);
    k_work_init(&data->sampling_work, sampling_work_handler);
    
    /* Start sampling timer */
    data->enabled = true;
    k_timer_start(&data->sampling_timer, K_MSEC(1000 / config->sampling_hz), 
                  K_MSEC(1000 / config->sampling_hz));
    
    LOG_INF("Analog input driver initialized successfully");
    data->ready = true;
}

/**
 * @brief Process raw ADC data for a single channel
 */
static void process_channel_data(const struct device *dev, uint8_t ch_idx, int32_t raw) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    const struct analog_input_io_channel *ch_cfg = &config->io_channels[ch_idx];
    
    int32_t mv = raw;
    adc_raw_to_millivolts(adc_ref_internal(ch_cfg->adc_channel.dev), 
                         ADC_GAIN_1_6, data->as.resolution, &mv);
    
    /* Apply midpoint offset */
    int32_t v = mv - ch_cfg->mv_mid;
    
    /* Apply deadzone */
    if (abs(v) < ch_cfg->mv_deadzone) {
        v = 0;
    }
    
    /* Apply min/max limits */
    if (v > ch_cfg->mv_min_max) {
        v = ch_cfg->mv_min_max;
    } else if (v < -ch_cfg->mv_min_max) {
        v = -ch_cfg->mv_min_max;
    }
    
    /* Apply scaling */
    if (ch_cfg->scale_divisor > 0) {
        v = (v * ch_cfg->scale_multiplier) / ch_cfg->scale_divisor;
    }
    
    /* Apply inversion */
    if (ch_cfg->invert) {
        v = -v;
    }
    
    /* Update delta and report */
    if (ch_cfg->report_on_change_only) {
        data->delta[ch_idx] = v;
    } else {
        data->delta[ch_idx] += v;
    }
    
    if (IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)) {
        LOG_DBG("Channel %d: raw=%d, mv=%d, processed=%d", 
               ch_idx, raw, mv, data->delta[ch_idx]);
    }
}

/**
 * @brief Report processed ADC data to the input subsystem
 */
static int analog_input_report_data(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    
    if (!data->ready || !data->enabled) {
        return 0;
    }
    
    int err = 0;
    
    if (config->mode == ANALOG_INPUT_MODE_SCAN) {
        /* Scan mode: read all channels in one go */
        err = adc_read(config->io_channels[0].adc_channel.dev, &data->as);
        if (err < 0) {
            LOG_ERR("ADC scan read failed: %d", err);
            return err;
        }
        
        /* Process scan results */
        for (uint8_t i = 0; i < config->scan_sequence_len; i++) {
            uint8_t channel_id = config->scan_sequence[i];
            
            /* Find corresponding channel configuration index */
            uint8_t ch_idx = UINT8_MAX;
            for (uint8_t j = 0; j < config->io_channels_len; j++) {
                if (config->io_channels[j].adc_channel.channel_id == channel_id) {
                    ch_idx = j;
                    break;
                }
            }
            
            if (ch_idx != UINT8_MAX) {
                /* Process this channel's data */
                process_channel_data(dev, ch_idx, data->as_buff[i]);
            }
        }
    } else {
        /* Single channel mode */
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            const struct analog_input_io_channel *ch_cfg = &config->io_channels[i];
            const struct device *adc = ch_cfg->adc_channel.dev;
            
            if (i == 0) {
                /* First read, use configured sequence */
                err = adc_read(adc, &data->as);
            } else {
                /* Subsequent reads, update channel configuration */
                data->as.channels = BIT(ch_cfg->adc_channel.channel_id);
                data->as.buffer = &data->as_buff[i];
                data->as.buffer_size = sizeof(data->as_buff[i]);
                err = adc_read(adc, &data->as);
            }
            
            if (err < 0) {
                LOG_ERR("Failed to read ADC channel %d: %d", i, err);
                continue;
            }
            
            /* Process this channel's data */
            process_channel_data(dev, i, data->as_buff[i]);
        }
    }
    
    /* Report processed data */
    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        const struct analog_input_io_channel *ch_cfg = &config->io_channels[i];
        int32_t dv = data->delta[i];
        
        if (dv != 0) {
            if (ch_cfg->report_on_change_only) {
                /* Report only when value changes */
                if (dv != data->prev[i]) {
                    input_report(dev, ch_cfg->evt_type, ch_cfg->input_code, dv, true, K_NO_WAIT);
                    data->prev[i] = dv;
                    
                    if (IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)) {
                        LOG_DBG("Reporting channel %d: value=%d", i, dv);
                    }
                }
            } else {
                /* Accumulative reporting */
                input_report(dev, ch_cfg->evt_type, ch_cfg->input_code, dv, true, K_NO_WAIT);
                data->delta[i] = 0;
                
                if (IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)) {
                    LOG_DBG("Reporting channel %d: delta=%d", i, dv);
                }
            }
        }
    }
    
    return 0;
}

/**
 * @brief Sampling timer handler
 */
static void sampling_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
    k_work_submit(&data->sampling_work);
}

/**
 * @brief Sampling work handler
 */
static void sampling_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    const struct device *dev = data->dev;
    
    analog_input_report_data(dev);
}

/**
 * @brief Driver initialization function
 */
static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    data->dev = dev;
    
    /* Schedule asynchronous initialization */
    k_work_submit(&data->init_work);
    
    return 0;
}

/* Driver registration */
static struct analog_input_data analog_input_data;

DEVICE_DT_INST_DEFINE(0, analog_input_init, NULL,
                      &analog_input_data,
                      DEVICE_DT_INST_GET(0)->config,
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                      NULL);