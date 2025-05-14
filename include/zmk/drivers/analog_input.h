#ifndef ZEPHYR_INCLUDE_ANALOG_INPUT_H_
#define ZEPHYR_INCLUDE_ANALOG_INPUT_H_

/**
 * @file analog_input.h
 *
 * @brief Header file for the analog_input driver.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/input.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of supported ADC channels */
#define ANALOG_INPUT_MAX_CHANNELS 8

/** Analog input operating modes */
enum analog_input_mode {
    ANALOG_INPUT_MODE_SINGLE, /**< Single channel mode */
    ANALOG_INPUT_MODE_SCAN,   /**< Scan mode */
};

/** Configuration for each ADC channel */
struct analog_input_io_channel {
    struct adc_dt_spec adc_channel;
    uint16_t mv_mid;
    uint16_t mv_min_max;
    uint8_t mv_deadzone;
    bool invert;
    bool report_on_change_only;
    uint16_t scale_multiplier;
    uint16_t scale_divisor;
    uint8_t evt_type;
    uint8_t input_code;
};

/** Analog input driver configuration */
struct analog_input_config {
    uint32_t sampling_hz;
    enum analog_input_mode mode;
    uint8_t io_channels_len;
    struct analog_input_io_channel io_channels[];
    uint8_t scan_sequence[ANALOG_INPUT_MAX_CHANNELS];
    uint8_t scan_sequence_len;
};

/** Analog input driver runtime data */
struct analog_input_data {
    const struct device *dev;
    struct adc_sequence as;
    uint16_t *as_buff;
    int32_t *delta;
    int32_t *prev;
    struct k_work_delayable init_work;
    bool ready;

    uint32_t sampling_hz;
    bool enabled;

    struct k_work sampling_work;
    struct k_timer sampling_timer;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ANALOG_INPUT_H_ */