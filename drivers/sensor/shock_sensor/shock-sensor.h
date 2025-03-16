#pragma once

#include <zephyr/drivers/adc.h>

#ifdef CONFIG_SEQUENCE_32BITS_REGISTERS
    #define ADC_READING_TYPE uint32_t
#else
    #define ADC_READING_TYPE uint16_t
#endif

#define SHOCK_SENSOR_ACTIVE 1
#define SHOCK_SENSOR_INACTIVE 0
#define CONFIG_SEQUENCE_SAMPLES 16

struct sensor_data {
    struct adc_sequence sequence;
    k_timeout_t earliest_sample;
    ADC_READING_TYPE raw[CONFIG_SEQUENCE_SAMPLES];

    const struct device *dev;   // self reference
    struct k_work_q workq;   /* work queue */
    struct k_work_delayable dwork;  /* workers */

    // #ifndef CONFIG_USE_SYS_WORK_Q
    //     K_KERNEL_STACK_MEMBER(workq_stack, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);
    // #endif
    // struct k_poll_signal async_sig;
    sensor_trigger_handler_t warn_handler;
    const struct sensor_trigger *warn_trigger;
    sensor_trigger_handler_t main_handler;
    const struct sensor_trigger *main_trigger;
    int treshold_warn;
    int treshold_main;
    int treshhold_warn_initial;
    int treshhold_main_initial;
    int threshold_warn_max;
    int threshold_main_max;
    int tap_count;
    int min_coarsering_interval;
    int min_tap_interval;
    int max_tap_interval;
    int coarsering_warn_percents;
    int coarsering_main_percents;
    int64_t last_coarsering_time;
    int64_t last_tap_time;
    struct k_timer reset_timer;
    bool active;
};

enum shock_sensor_channel {
    SHOCK_SENSOR_CHANNEL_TRESHHOLDS_INITIAL=65,
    SHOCK_SENSOR_CHANNEL_TAP_MIN_MAX_INTERVALS,
    SHOCK_SENSOR_CHANNEL_MIN_COARSERING_INTERVAL,
    SHOCK_SENSOR_CHANNEL_THRESHHOLDS_WARN_MAIN_MAX,
    SHOCK_SENSOR_CHANNEL_COARSING_WARN_MAIN_PERCENRS,
    SHOCK_SENSOR_CHANNEL_ACTIVE,
};

enum shock_sensor_attrs {
    SHOCK_SENSOR_SPECIAL_ATTRS=32,
};

struct shock_sensor_dt_spec {
	const struct adc_dt_spec port;
    uint32_t sampling_period_ms;
};

/**
 * @brief Get shock sensor information from devicetree.
 *
 * This returns a static initializer for a @p shock_sensor_dt_spec structure
 * given a devicetree node.
 *
 * @param node_id Devicetree node identifier.
 *
 * @return Static initializer for an shock_sensor_dt_spec structure.
 */
#define SHOCK_SENSOR_DT_SPEC_GET(node_id)                                                   \
	{                                                                                       \
		.port = ADC_DT_SPEC_GET(node_id),                                                   \
		.sampling_period_ms = DT_PROP_OR(node_id, sampling_period_ms, 100),                     \
	}

typedef void (*warn_cb)(const struct device *sensor, int value);

__subsystem struct shock_sensor_driver_api {
    sensor_attr_set_t attr_set;
    sensor_attr_get_t attr_get;
    sensor_trigger_set_t trigger_set;
    sensor_sample_fetch_t sample_fetch;
    sensor_channel_get_t channel_get;
    sensor_get_decoder_t get_decoder;
    sensor_submit_t submit;
};

void reset_timer_handler(struct k_timer *);