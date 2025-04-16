#pragma once

#include <zephyr/drivers/adc.h>

enum shock_sensor_channel {
    SHOCK_SENSOR_MODE=65,
    SHOCK_SENSOR_CHANNEL_WARN_ZONE,
    SHOCK_SENSOR_CHANNEL_MAIN_ZONE,
    SHOCK_SENSOR_INCREASE_SENSIVITY_INTERVAL_SEC,
    SHOCK_SENSOR_NOISE_SAMPLING_TIME_SEC,
};

enum shock_sensor_attrs {
    SHOCK_SENSOR_SPECIAL_ATTRS=32,
};

enum shock_sensor_mode {
    SHOCK_SENSOR_MODE_ARMED=0,
    SHOCK_SENSOR_MODE_DISARMED,
    SHOCK_SENSOR_MODE_ALARM,
    SHOCK_SENSOR_MODE_ALARM_STOP,
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
