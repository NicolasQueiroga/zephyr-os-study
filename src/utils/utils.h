#ifndef __UTILS_H__
#define __UTILS_H__

#include <zephyr/drivers/gpio.h>

#define SW0_NODE DT_ALIAS(sw0)
#define ECHO_NODE DT_ALIAS(d0)
#define TRIG_NODE DT_ALIAS(d1)

typedef void (*gpio_callback_handler_t)(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

int configure_input(const struct gpio_dt_spec *spec, int flags, int interrupt_flags);
void configure_input_cb(struct gpio_callback *cb, const struct gpio_dt_spec *spec, gpio_callback_handler_t handler);
int configure_output(const struct gpio_dt_spec *spec, int flags);

#endif // __UTILS_H__
