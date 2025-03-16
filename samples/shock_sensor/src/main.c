#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SHOCK_NODE       DT_ALIAS(shock_sensor)
static const struct device *dev = DEVICE_DT_GET(SHOCK_NODE);

void sensor_warn_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	printk("Tap (Warn)!\n");
}

void sensor_main_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    printk("Tap (Main)!\n");
}

const struct sensor_trigger trig_warn = {
    .chan = SENSOR_CHAN_PROX,
    .type = SENSOR_TRIG_TAP,
};

const struct sensor_trigger trig_main = {
    .chan = SENSOR_CHAN_PROX,
    .type = SENSOR_TRIG_THRESHOLD,
};

static int shock_init()
{
    if(!device_is_ready(dev)) {
        printk("ADC VBUS controller device %s not ready\n", dev->name);
        return -ENODEV;
    }
 
    //задаємо максимуми для порогів зон
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_THRESHHOLDS_WARN_MAIN_MAX, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = 120, .val2 = 120 });
    //задаємо на скільки відсотків підвищувати пороги зон за одне загрублення
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_COARSING_WARN_MAIN_PERCENRS, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = 20, .val2 = 5 });
    //задаємо мінімальний інтервал між спрацьовуваннями (мс) і максимальний інтервал між спрацьовуваннями (с) при якому скидається загрублення
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_TAP_MIN_MAX_INTERVALS, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = 5000, .val2 = 30 });
    //задаємо мінімальний інтервал між загрубленнями (c)
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_MIN_COARSERING_INTERVAL, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = 30, .val2 = 0 });
    //задаємо початкові пороги зон
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = 10, .val2 = 60 });
    //задаємо початкові пороги зон (для загрублення)
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_TRESHHOLDS_INITIAL, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = 10, .val2 = 60 });
    //активуємо сенсор
    sensor_attr_set(dev, SHOCK_SENSOR_CHANNEL_ACTIVE, SHOCK_SENSOR_SPECIAL_ATTRS, &(struct sensor_value){ .val1 = SHOCK_SENSOR_ACTIVE, .val2 = 0 });

    int rc = sensor_trigger_set(dev, &trig_warn, sensor_warn_handler);
    rc = sensor_trigger_set(dev, &trig_main, sensor_main_handler);

    printk("Shock sensor controller device %s is ready\n", dev->name);
    return 0;
}

int main(void)
{
    printk("Hello World from minimal!\n");

    shock_init();

    for(;;) {
        k_msleep(1000);
    }

    return 0;
}