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

    int warn_zone = 10;
    int main_zone = 60;
    struct sensor_value val = { .val1 = warn_zone, .val2 = main_zone };
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &val);

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