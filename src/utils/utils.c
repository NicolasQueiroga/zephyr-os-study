#include "utils.h"
#include "distance.h"

int configure_input(const struct gpio_dt_spec *spec, int flags, int interrupt_flags)
{
    if (!gpio_is_ready_dt(spec))
    {
        printk("Error: button device %s is not ready\n", spec->port->name);
        return 0;
    }
    if (gpio_pin_configure_dt(spec, flags))
    {
        printk("Error: failed to configure %s pin %d\n", spec->port->name, spec->pin);
        return 0;
    }
    if (gpio_pin_interrupt_configure_dt(spec, interrupt_flags))
    {
        printk("Error: failed to configure interrupt on %s pin %d\n", spec->port->name, spec->pin);
        return 0;
    }
    return 1;
}

void configure_input_cb(struct gpio_callback *cb, const struct gpio_dt_spec *spec, gpio_callback_handler_t handler)
{
    gpio_init_callback(cb, handler, BIT(spec->pin));
    gpio_add_callback(spec->port, cb);
}

int configure_output(const struct gpio_dt_spec *spec, int flags)
{
    if (!gpio_is_ready_dt(spec))
    {
        printk("Error: LED device %s is not ready\n", spec->port->name);
        return 0;
    }
    if (gpio_pin_configure_dt(spec, flags))
    {
        printk("Error: failed to configure %s pin %d\n", spec->port->name, spec->pin);
        return 0;
    }
    return 1;
}
