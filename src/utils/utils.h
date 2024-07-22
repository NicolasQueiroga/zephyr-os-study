#ifndef __UTILS_H__
#define __UTILS_H__

#include <zephyr/drivers/gpio.h>

#define SW0_NODE DT_ALIAS(sw0)
#define LED0_NODE DT_ALIAS(led0)

int configure_input(const struct gpio_dt_spec *spec, int flags, int interrupt_flags);
void configure_input_cb(struct gpio_callback *cb, const struct gpio_dt_spec *spec);
int configure_output(const struct gpio_dt_spec *spec);

#endif // __UTILS_H__
