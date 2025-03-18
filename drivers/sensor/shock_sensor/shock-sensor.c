/*
 * Copyright (c) 2023 FTP Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_shock_sensor

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include "shock-sensor.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shock_sensor, CONFIG_SENSOR_LOG_LEVEL);

struct sensor_config {
    struct shock_sensor_dt_spec sensor;
    struct gpio_dt_spec gpio_power;

    k_thread_stack_t *work_q_stack;
	size_t work_q_stack_size;
};

// #define CONFIG_USE_SYS_WORK_Q
// #define CONFIG_USE_ASYNC_ADC_READ

// struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);




#define CONFIG_SHAKE_CENTERED_COUNT (256 / CONFIG_SEQUENCE_SAMPLES)
// Можливо треба зробити вдвічі більше ніж CONFIG_SHAKE_CENTERED_COUNT.
// Хоча раз взагалі якось працює і при 1, то може значення 4 буде більш ніж достатньо.
#define MULTIPLIER 16

#define Secs(x) (x * 200 / CONFIG_SEQUENCE_SAMPLES)

// Кількість періодів опитування датчика для визначення спрацювання.
#define CONFIG_SHAKE_MAIN_TIME Secs(1)
#define CONFIG_SHAKE_WARN_TIME Secs(1)




static int fetch(const struct device *dev, enum sensor_channel chan)
{
    int ret = 0;
    #if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;

    if ((chan != SENSOR_CHAN_VOLTAGE) && (chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    /* Wait until sampling is valid */
    k_sleep(data->earliest_sample);

    ret = adc_read(config->voltage.port.dev, &data->sequence);
    if (ret != 0) {
        LOG_ERR("adc_read: %d", ret);
    }

#endif
    return ret;
}

static int get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    int ret = 0;
#if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;
    int32_t raw_val;
    int32_t v_mv;

    __ASSERT_NO_MSG(val != NULL);

    if (chan != SENSOR_CHAN_VOLTAGE) {
        return -ENOTSUP;
    }

    if (config->voltage.port.channel_cfg.differential) {
        raw_val = (int16_t)data->raw;
    } else {
        raw_val = data->raw;
    }

    ret = adc_raw_to_millivolts_dt(&config->voltage.port, &raw_val);
    if (ret != 0) {
        LOG_ERR("raw_to_mv: %d", ret);
        return ret;
    }

    v_mv = raw_val;

    /* Note if full_ohms is not specified then unscaled voltage is returned */
    (void)voltage_divider_scale_dt(&config->voltage, &v_mv);

    LOG_DBG("%d of %d, %dmV, voltage:%dmV", data->raw,
        (1 << data->sequence.resolution) - 1, raw_val, v_mv);
    val->val1 = v_mv / 1000;
    val->val2 = (v_mv * 1000) % 1000000;
#endif
    return ret;
}

static int attr_set(const struct device *dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value *val)
{
    struct sensor_data *data = dev->data;

    if (chan == SENSOR_CHAN_PROX && attr == SENSOR_ATTR_UPPER_THRESH) {
        data->treshold_warn = val->val1;
        data->treshold_main = val->val2;
        LOG_ERR("Seted treshold_warn: %d, treshold_main: %d", data->treshold_warn, data->treshold_main);
        return 0;
    }

    if (chan == SHOCK_SENSOR_CHANNEL_WARN_ZONE && attr == SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->current_warn_zone = val->val1;
        data->selected_warn_zone = val->val1;
        create_main_zones(dev, val->val1);
        data->current_main_zone = 0;
        data->selected_main_zone = 0;
        data->last_tap_time_warn = k_uptime_get();
        data->last_tap_time_main = k_uptime_get();
        data->last_coarsering_time_warn = k_uptime_get();
        data->last_coarsering_time_main = k_uptime_get();
        LOG_ERR("Seted warn_zone: %d", data->current_warn_zone);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        return 0;
    }

    if (chan == SHOCK_SENSOR_CHANNEL_MAIN_ZONE && attr == SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->current_main_zone = val->val1;
        data->selected_main_zone = val->val1;
        data->current_warn_zone = data->selected_warn_zone;
        data->last_tap_time_warn = k_uptime_get();
        data->last_tap_time_main = k_uptime_get();
        data->last_coarsering_time_warn = k_uptime_get();
        data->last_coarsering_time_main = k_uptime_get();
        LOG_ERR("Seted main_zone: %d", data->current_main_zone);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        return 0;
    }

    if (chan == SHOCK_SENSOR_CHANNEL_TAP_MIN_MAX_INTERVALS && attr == SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->min_tap_interval = val->val1;
        data->max_tap_interval = val->val2;
        data->last_tap_time_warn = k_uptime_get();
        data->last_tap_time_main = k_uptime_get();
        LOG_ERR("Seted min_tap_interval: %d, max_tap_interval: %d", data->min_tap_interval, data->max_tap_interval);
        return 0;
    }

    if (chan == SHOCK_SENSOR_CHANNEL_MIN_COARSERING_INTERVAL && attr == SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->min_coarsering_interval = val->val1;
        data->last_coarsering_time_warn = k_uptime_get();
        data->last_coarsering_time_main = k_uptime_get();
        LOG_ERR("Seted min_coarsering_interval: %d", data->min_coarsering_interval);
        return 0;
    }

    if (chan == SHOCK_SENSOR_MODE && attr == SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->mode = val->val1;
        LOG_ERR("Seted mode: %d", data->mode);
        if (data->mode == SHOCK_SENSOR_MODE_ALARM) {
            LOG_ERR("Entering alarm mode for %d seconds\n", val->val2);
            k_timer_start(&data->reset_timer_alarm, K_SECONDS(val->val2), K_NO_WAIT);
        }
        if (data->mode == SHOCK_SENSOR_MODE_DISARMED || data->mode == SHOCK_SENSOR_MODE_TURN_OFF) {
            data->current_warn_zone = data->selected_warn_zone;
            data->current_main_zone = data->selected_main_zone;
            set_zones(dev, data->current_warn_zone, data->current_main_zone);
            LOG_ERR("Sensor is disarmed\n");
        }
        if (data->mode == SHOCK_SENSOR_MODE_ARMED) {
            data->last_tap_time_warn = k_uptime_get();
            data->last_tap_time_main = k_uptime_get();
            data->last_coarsering_time_warn = k_uptime_get();
            data->last_coarsering_time_main = k_uptime_get();
            LOG_ERR("Sensor is armed\n");
        }
        return 0;
    }

    return -ENOTSUP;
}

static int trigger_set(const struct device *dev, const struct sensor_trigger *trig,
    sensor_trigger_handler_t handler)
{
    struct sensor_data *data = dev->data;
    // LOG_ERR("[%s - %s] dev: %p, trig->chan: %d, trig->type: %d, handler: %p", __func__,
    // (handler == NULL) ? "off" : "on", dev, trig->chan, trig->type, handler);

    if (trig == NULL || handler == NULL) return -EINVAL;

    if (trig->type != SENSOR_TRIG_THRESHOLD && trig->type != SENSOR_TRIG_TAP) {
		return -ENOTSUP;
	}

    switch (trig->type) {
        case SENSOR_TRIG_TAP:
            // LOG_ERR("----------------- Seted");
            data->warn_handler = handler;
            data->warn_trigger = trig;
        break;
        case SENSOR_TRIG_THRESHOLD:
            // LOG_ERR("----------------- Seted");
            data->main_handler = handler;
            data->main_trigger = trig;
        break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret = 0;
    #if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;

    if (config->gpio_power.port == NULL) {
        /* No work to do */
        return 0;
    }

    switch (action) {
    case PM_DEVICE_ACTION_TURN_ON:
        ret = gpio_pin_configure_dt(&config->gpio_power, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("failed to configure GPIO for PM on");
        }
        break;
    case PM_DEVICE_ACTION_RESUME:
        ret = gpio_pin_set_dt(&config->gpio_power, 1);
        if (ret != 0) {
            LOG_ERR("failed to set GPIO for PM resume");
        }
        data->earliest_sample = K_TIMEOUT_ABS_TICKS(
            k_uptime_ticks() + k_us_to_ticks_ceil32(config->sample_delay_us));
        break;
#ifdef CONFIG_PM_DEVICE
    case PM_DEVICE_ACTION_SUSPEND:
        ret = gpio_pin_set_dt(&config->gpio_power, 0);
        if (ret != 0) {
            LOG_ERR("failed to set GPIO for PM suspend");
        }
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
        break;
#endif /* CONFIG_PM_DEVICE */
    default:
        return -ENOTSUP;
    }
    #endif

    return ret;
}
#endif

// static void adc_vbus_process()
// {
//     printk("w");
//     /* Запускаємо асинхронне читання */
//     int ret = adc_read_async(adc_dev, &adc_seq_async, &adc_done_signal);
// }



// TODO: Перенести це все в data

// Ненулове значення блокує спрацювання датчика удару.
int share_ignore_time = 0;

// Ненульове значення означає шо датчик в спрацюванні
int shake_warn = 0;
int shake_main = 0;

// TODO: Перенести це в data
static int adc_centered_value = 0;


#define __TEMP_SENSOR_SHAKE_WARN_ZONE 14
#define __TEMP_SENSOR_SHAKE_MAIN_ZONE 100

// int debug_counter = 0;


static void adc_vbus_work_handler(struct k_work *work)
{
    // Шось це якось заплутано. Чи можна якось простіше?
    // Можна при ініціалізації скопіювати sampling_period_ms в data
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct sensor_data *data = CONTAINER_OF(delayable, struct sensor_data, dwork);
    // dev
    const struct device *dev = data->dev;
    // const struct sensor_config *config = data->dev->config;
    const struct sensor_config *config = dev->config;

    // adc_vbus_process();

    // printk("w");
    #if 0
    int ret = adc_read(config->sensor.port.dev, &data->sequence);
	if (ret != 0) {
		LOG_ERR("adc_read: %d", ret);
	}
    #endif

    // uint64_t timestart = k_ticks_to_us_floor64(k_uptime_ticks()); //k_uptime_get();

    #if defined(CONFIG_USE_ASYNC_ADC_READ)
        struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
        struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY, &async_sig);
        int ret = adc_read_async(config->sensor.port.dev, &data->sequence, &async_sig);
    #else
        int ret = adc_read(config->sensor.port.dev, &data->sequence);
    #endif
    if(ret != 0) {
        LOG_ERR("adc_read[_async]: %d", ret);
        goto end;
    }
    // debug_counter++;
    #if defined(CONFIG_USE_ASYNC_ADC_READ)
        ret = k_poll(&async_evt, 1, K_FOREVER);
        if(ret != 0) {
            LOG_ERR("k_poll: %d", ret);
            goto end;
        }
    #endif
    // LOG_ERR("Data: %d", data->raw);

    // uint64_t timestop = k_ticks_to_us_floor64(k_uptime_ticks()); //k_uptime_get();
    // LOG_ERR("ADC read time: %" PRIu64 " usec", timestop - timestart);

    // Час перетворення при interval_us=0:
    //   1 самплінг - 1200 usec
    //  10 самплінгів - 1700 usec
    // 100 самплінгів - 9200 usec

    #if 0
    #if CONFIG_SEQUENCE_SAMPLES == 1
        LOG_ERR("Data: %d", data->raw[0]);
    #elif CONFIG_SEQUENCE_SAMPLES == 10
        LOG_ERR("Data: "
            "%d %d %d %d %d %d %d %d %d %d",
            data->raw[0], data->raw[1], data->raw[2], data->raw[3], data->raw[4],
            data->raw[5], data->raw[6], data->raw[7], data->raw[8], data->raw[9]);
    #else
        uint32_t sum = 0;
        uint32_t min = data->raw[0];
        uint32_t max = data->raw[0];
        for(int i=0; i<CONFIG_SEQUENCE_SAMPLES; i++) {
            sum += data->raw[i];
            if(data->raw[i] < min) {
                min = data->raw[i];
            }
            if(data->raw[i] > max) {
                max = data->raw[i];
            }
        }
        LOG_ERR("Data: %d/%d/%d %d", min, sum/CONFIG_SEQUENCE_SAMPLES, max, max-min);
        // LOG_HEXDUMP_ERR(data->raw, sizeof(data->raw), "Data: ");
    #endif
    #endif

    // Збільшемо значення шоб подальша математика не втрачала точність
    #if CONFIG_SEQUENCE_SAMPLES == 1
        int new_val = data->raw[0] * MULTIPLIER;
    #else
        uint32_t sum = 0;
        uint32_t min = data->raw[0];
        uint32_t max = data->raw[0];
        for(int i=0; i<CONFIG_SEQUENCE_SAMPLES; i++) {
            sum += data->raw[i];
            if(data->raw[i] < min) {
                min = data->raw[i];
            }
            if(data->raw[i] > max) {
                max = data->raw[i];
            }
        }
        min *= MULTIPLIER;
        max *= MULTIPLIER;
        int new_val = sum * MULTIPLIER / CONFIG_SEQUENCE_SAMPLES;
    #endif

    // Set centered value. Middle of the last 16 samples
    // Save 16x value, but use normalised value later
    adc_centered_value = (adc_centered_value * (CONFIG_SHAKE_CENTERED_COUNT-1) + new_val) / CONFIG_SHAKE_CENTERED_COUNT;

    // LOG_ERR("Shake sensor sample_raw: %d", val.val1);

    if(shake_main) {
        shake_main--;
    }

    if(shake_warn) {
        shake_warn--;
    }

    if(share_ignore_time) {
        //if(!is_panic()) {
            share_ignore_time--;
        //}
        goto end;
    }

    #if CONFIG_SEQUENCE_SAMPLES == 1
        int amplitude_x = new_val - adc_centered_value;
        int amplitude_abs = abs(amplitude_x) / MULTIPLIER;
    #else
        int amplitude_x1 = max - adc_centered_value;
        int amplitude_x2 = adc_centered_value - min;
        int amplitude_x = (amplitude_x1 > amplitude_x2) ? amplitude_x1 : amplitude_x2;
        int amplitude_abs = amplitude_x / MULTIPLIER;
    #endif

    // if(debug_counter >= 31) {
    //     LOG_ERR("Shake amplitude: %d  %d-%d-%d", 
    //         amplitude_abs,
    //         min, adc_centered_value, max
    //     );
    //     debug_counter = 0;
    // }

    if(amplitude_abs > data->treshold_main /*TPF(sensor_shake_zone2)*/) {
        if(shake_main == 0) {
            // LOG_ERR("Shock MAIN: %d", amplitude_x/MULTIPLIER);
            
            shake_main = CONFIG_SHAKE_MAIN_TIME;

            if(data->main_handler != NULL) {
                // struct sensor_trigger trig = {
                //     .chan = SENSOR_CHAN_PROX,
                //     .type = SENSOR_TRIG_THRESHOLD,
                // };
                if (data->mode == 0) {
                    register_tap_main(data);
                    data->main_handler(dev, data->main_trigger);
                    k_timer_start(&data->reset_timer_main, K_SECONDS(data->max_tap_interval), K_NO_WAIT);
                    // printk("amplitude: %d\n", amplitude_abs);
                } else {
                    LOG_ERR("Tap detected, but sensor is inactive\n");
                }
            }
        } 
    } else if(amplitude_abs > data->treshold_warn) {
        if(shake_warn == 0) {
            // LOG_ERR("Shock WARN: %d (%d/%d)", amplitude_x/MULTIPLIER, new_val, adc_centered_value);
            shake_warn = CONFIG_SHAKE_WARN_TIME;

            if(data->warn_handler != NULL) {
                // struct sensor_trigger trig = {
                //     .chan = SENSOR_CHAN_PROX,
                //     .type = SENSOR_TRIG_TAP,
                // };
                if (data->mode == 0) {
                    register_tap_warn(data);
                    data->warn_handler(dev, data->warn_trigger);
                    k_timer_start(&data->reset_timer_warn, K_SECONDS(data->max_tap_interval), K_NO_WAIT);
                    // printk("amplitude: %d\n", amplitude_abs);
                } else {
                    LOG_ERR("Tap detected, but sensor is inactive\n");
                }
                
                // LOG_ERR("Debug counter: %d", debug_counter);
                // debug_counter = 0;
            }
        }
    }

end:
    #ifdef CONFIG_USE_SYS_WORK_Q
        k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #else
        k_work_schedule_for_queue(&data->workq, &data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #endif
}

// TODO: Це треба винести в API

void shake_ignore_me(int secs)
{
    share_ignore_time = Secs(secs);
}

// static K_THREAD_STACK_DEFINE(workq_stack, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);

static const struct sensor_driver_api sensor_api = {
    .sample_fetch = fetch,
    .channel_get = get,
    .trigger_set = trigger_set,
	.attr_set = attr_set,
};

/* Options for the sequence sampling. */
const struct adc_sequence_options options = {
    .extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
    .interval_us = 2000,
};

static int sensor_init(const struct device *dev)
{
    int ret = 0;

    printk("==== shock_sensor_init(%s)\n", dev->name);

    const struct sensor_config *config = dev->config;
    struct sensor_data *data = dev->data;

    // self reference
    data->dev = dev;

    data->warn_handler = NULL;
    data->main_handler = NULL;
    data->treshold_warn = __TEMP_SENSOR_SHAKE_WARN_ZONE;
    data->treshold_main = __TEMP_SENSOR_SHAKE_MAIN_ZONE;

    // Перші 10 секунд треба пропустити спрацювання за для стабілізації датчика
    shake_main = 10 * 1000 / config->sensor.sampling_period_ms / CONFIG_SEQUENCE_SAMPLES;
    shake_warn = shake_main;

    // Середина живлення (десь 522513). Якось треба врахувати розрядність ADC. Поки це 12 біт.
    adc_centered_value = MULTIPLIER * 2048;

    /* Default value to use if `power-gpios` does not exist */
    data->earliest_sample = K_TIMEOUT_ABS_TICKS(0);

    if (!adc_is_ready_dt(&config->sensor.port)) {
        LOG_ERR("ADC is not ready");
        return -ENODEV;
    }

    if (config->gpio_power.port != NULL) {
        if (!gpio_is_ready_dt(&config->gpio_power)) {
            LOG_ERR("Power GPIO is not ready");
            return -ENODEV;
        }
    }

    ret = adc_channel_setup_dt(&config->sensor.port);
    if (ret != 0) {
        LOG_ERR("setup: %d", ret);
        return ret;
    }

    ret = adc_sequence_init_dt(&config->sensor.port, &data->sequence);
    if (ret != 0) {
        LOG_ERR("sequence init: %d", ret);
        return ret;
    }

    data->sequence.buffer = &data->raw[0];
    data->sequence.buffer_size = sizeof(data->raw); /* buffer size in bytes, not number of samples */
    LOG_ERR("Buffer size: %d", data->sequence.buffer_size);
    // data->sequence.resolution = 12;
    data->sequence.options = &options;


    // Use k_sys_work_q for the work queue
    data->workq = k_sys_work_q;

    printk("==== shock_sensor period: %d ms\n", config->sensor.sampling_period_ms);

    #ifdef CONFIG_USE_SYS_WORK_Q
        k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
        k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #else
        // Init signal
        // k_poll_signal_init(&data->async_sig);

        struct k_work_queue_config workq_cfg = {
            .name = "shock_sensor:WQ",
            // .no_yield = true,
        };

        k_work_queue_init(&data->workq);
        /* initialize the sensor work queue */
        k_work_queue_start(&data->workq,
            config->work_q_stack, config->work_q_stack_size,
            // workq_stack, K_KERNEL_STACK_SIZEOF(workq_stack),
            // data->workq_stack,
            // K_KERNEL_STACK_SIZEOF(data->workq_stack),
            CONFIG_SENSOR_SHOCK_THREAD_PRIORITY,
            // K_PRIO_COOP(7),
            &workq_cfg);
        // k_thread_name_set(&data->workq.thread, "shock_sensor:WQ");
        k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
        k_work_schedule_for_queue(&data->workq, &data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #endif

    // k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
    // // Перший запуск зробимо з системної черги
    // k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
            

    #ifdef CONFIG_PM_DEVICE_RUNTIME
        return pm_device_driver_init(dev, pm_action);
    #else
        data->mode = SHOCK_SENSOR_MODE_DISARMED;
        set_warn_zones(dev);
        set_zones(dev, 0, 0);

        data->last_tap_time_warn = k_uptime_get();
        data->last_tap_time_main = k_uptime_get();
        
        k_timer_init(&data->reset_timer_warn, reset_timer_handler_warn, NULL);
        k_timer_user_data_set(&data->reset_timer_warn, (void *)dev);
        k_timer_init(&data->reset_timer_main, reset_timer_handler_main, NULL);
        k_timer_user_data_set(&data->reset_timer_main, (void *)dev);
        k_timer_init(&data->reset_timer_alarm, reset_timer_handler_alarm, NULL);
        k_timer_user_data_set(&data->reset_timer_alarm, (void *)dev);

        return 0;
    #endif
}

void reset_timer_handler_warn(struct k_timer *timer)
{
    struct device *dev = k_timer_user_data_get(timer);
    if (!dev) {
        printk("Device is NULL in timer handler!");
        return;
    }

    struct sensor_data *data = dev->data;

    if (data->mode != SHOCK_SENSOR_MODE_ARMED) return;

    // printk("Tap count warn: %d\n", data->warn_count);
    coarsering_warn(data, false);
}

void reset_timer_handler_main(struct k_timer *timer)
{
    struct device *dev = k_timer_user_data_get(timer);
    if (!dev) {
        printk("Device is NULL in timer handler!");
        return;
    }

    struct sensor_data *data = dev->data;

    if (data->mode != SHOCK_SENSOR_MODE_ARMED) return;
    // printk("Tap count main: %d\n", data->main_count);
    coarsering_main(data, false);
}

void reset_timer_handler_alarm(struct k_timer *timer)
{
    struct device *dev = k_timer_user_data_get(timer);
    if (!dev) {
        printk("Device is NULL in timer handler!");
        return;
    }

    struct sensor_data *data = dev->data;

    if (data->mode != SHOCK_SENSOR_MODE_ALARM) return;

    data->mode = SHOCK_SENSOR_MODE_ARMED;
    LOG_ERR("Sensor is armed\n");
}

void set_zones(const struct device *dev, int warn_zone, int main_zone)
{
    struct sensor_data *data = dev->data;
    
    data->selected_warn_zone = warn_zone;
    data->current_warn_zone = warn_zone;
    create_main_zones(dev, warn_zone);
    data->selected_main_zone = main_zone;
    data->current_main_zone = main_zone;
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[warn_zone], .val2 = data->main_zones[main_zone] });
}

void set_warn_zones(const struct device *dev)
{
    struct sensor_data *data = dev->data;
    for (int i = 0; i < 16; i++) {
        data->warn_zones[i] = warn_zones_initial[i]; 
    }
}

void create_main_zones(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    for (int i = 0; i < 16; i++) {
        data->main_zones[i] = (data->warn_zones[zone] + i) * 5; 
    }
}

void set_warn_zone(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    data->selected_warn_zone = zone;
    data->current_warn_zone = zone;
    create_main_zones(dev, zone);
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[zone], .val2 = data->main_zones[data->current_main_zone] });
}

void change_warn_zone(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    data->current_warn_zone = zone;
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[zone], .val2 = data->main_zones[data->current_main_zone] });
}

void set_main_zone(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    data->selected_main_zone = zone;
    data->current_main_zone = zone;
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[zone] });
}

void change_main_zone(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    data->current_main_zone = zone;
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[zone] });
}

void coarsering_warn(struct sensor_data *data, bool increase)
{
    if (k_uptime_get() - data->last_coarsering_time_warn < data->min_coarsering_interval) return;
    if (increase) {
        if (data->current_warn_zone == 15) 
        {
            data->last_coarsering_time_warn = k_uptime_get();
            return;
        }
        data->current_warn_zone++;
    } else {
        if (data->current_warn_zone == data->selected_warn_zone)
        {
            data->last_coarsering_time_warn = k_uptime_get();
            return;
        }
        data->current_warn_zone--;
        k_timer_start(&data->reset_timer_warn, K_SECONDS(data->max_tap_interval), K_NO_WAIT);
    }
    // printk("coarsering_warn: %d\n", data->current_warn_zone);
    sensor_attr_set(data->dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[data->current_main_zone] });
    data->last_coarsering_time_warn = k_uptime_get();
    data->warn_count = 0;
}

void coarsering_main(struct sensor_data *data, bool increase)
{
    if (k_uptime_get() - data->last_coarsering_time_main < data->min_coarsering_interval) return;
    if (increase) {
        if (data->current_main_zone == 15) 
        {
            data->last_coarsering_time_main = k_uptime_get();
            return;
        }
        data->current_main_zone++;
    } else {
        if (data->current_main_zone == data->selected_main_zone) 
        {
            data->last_coarsering_time_main = k_uptime_get();
            return;
        }
        data->current_main_zone--;
        k_timer_start(&data->reset_timer_main, K_SECONDS(data->max_tap_interval), K_NO_WAIT);
    }
    // printk("coarsering_main: %d\n", data->current_main_zone);
    sensor_attr_set(data->dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[data->current_main_zone] });
    data->last_coarsering_time_main = k_uptime_get();
    data->main_count = 0;
}

void register_tap_main(struct sensor_data *data)
{
    int64_t current_time = k_uptime_get();
    data->main_count++;
    if (current_time - data->last_tap_time_main < data->min_tap_interval) {
        printk("Warning: Possible abuse detected, taps too frequent\n");
    } else if (data->main_count > 1) {
        coarsering_main(data, true);
    }
    data->last_tap_time_main = current_time;
}

void register_tap_warn(struct sensor_data *data)
{
    int64_t current_time = k_uptime_get();
    data->warn_count++;
    if (current_time - data->last_tap_time_warn < data->min_tap_interval) {
        printk("Warning: Possible abuse detected, taps too frequent\n");
    } else if (data->warn_count > 1) {
        coarsering_warn(data, true);
    }
    data->last_tap_time_warn = current_time;
}



#define _INIT(inst)                                                                         \
    static struct sensor_data sensor_##inst##_data;                                       \
    static K_THREAD_STACK_DEFINE(work_q_stack_##_inst, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);       \
                                                                                            \
    static const struct sensor_config sensor_##inst##_config = {                          \
        .sensor = SHOCK_SENSOR_DT_SPEC_GET(DT_DRV_INST(inst)),                          \
        .gpio_power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),                     \
		.work_q_stack = (k_thread_stack_t *)&work_q_stack_##_inst,                         \
		.work_q_stack_size = K_THREAD_STACK_SIZEOF(work_q_stack_##_inst),                  \
    };                                                                                      \
                                                                                            \
    PM_DEVICE_DT_INST_DEFINE(inst, pm_action);                                              \
                                                                                            \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, &sensor_init, PM_DEVICE_DT_INST_GET(inst),          \
                &sensor_##inst##_data, &sensor_##inst##_config, POST_KERNEL,              \
                CONFIG_SENSOR_INIT_PRIORITY, &sensor_api);

DT_INST_FOREACH_STATUS_OKAY(_INIT)
