#pragma once

#include <zephyr/drivers/adc.h>

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
    