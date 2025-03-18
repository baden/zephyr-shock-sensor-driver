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

    int warn_zones[16];
    int main_zones[16];
    int selected_warn_zone;
    int selected_main_zone;
    int current_warn_zone;
    int current_main_zone;

    int treshold_warn;
    int treshold_main;

    int warn_count;
    int main_count;

    int min_tap_interval;
    int max_tap_interval;

    int64_t last_tap_time_warn;
    int64_t last_tap_time_main;

    int min_coarsering_interval;
    int64_t last_coarsering_time_warn;
    int64_t last_coarsering_time_main;

    struct k_timer reset_timer_warn;
    struct k_timer reset_timer_main;
    struct k_timer reset_timer_alarm;

    bool active;
    int mode;
};

enum shock_sensor_channel {
    SHOCK_SENSOR_CHANNEL_TAP_MIN_MAX_INTERVALS=65,
    SHOCK_SENSOR_CHANNEL_MIN_COARSERING_INTERVAL,
    SHOCK_SENSOR_CHANNEL_ACTIVE,
    SHOCK_SENSOR_MODE,
    SHOCK_SENSOR_CHANNEL_WARN_ZONE,
    SHOCK_SENSOR_CHANNEL_MAIN_ZONE,
};

enum shock_sensor_attrs {
    SHOCK_SENSOR_SPECIAL_ATTRS=32,
};

enum shock_sensor_mode {
    SHOCK_SENSOR_MODE_ARMED=0,
    SHOCK_SENSOR_MODE_DISARMED,
    SHOCK_SENSOR_MODE_ALARM,
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

void reset_timer_handler_main(struct k_timer *);
void reset_timer_handler_warn(struct k_timer *);
void reset_timer_handler_alarm(struct k_timer *);