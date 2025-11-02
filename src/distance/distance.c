#include "distance.h"

K_THREAD_STACK_DEFINE(distance_calculator_thread_stack_area, STACKSIZE);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    btn_callback_data_t *data = CONTAINER_OF(cb, btn_callback_data_t, cb);

    gpio_pin_set(data->trig->port, data->trig->pin, 1);
    k_busy_wait(10);
    gpio_pin_set(data->trig->port, data->trig->pin, 0);

    printk("button_pressed: button pressed\n");
}

void echo_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    echo_callback_data_t *data = CONTAINER_OF(cb, echo_callback_data_t, cb);
    if (gpio_pin_get(data->echo->port, data->echo->pin))
    {
        *data->timer = k_cycle_get_32();
    }
    else
    {
        uint32_t end_time = k_cycle_get_32();
        uint32_t cycles_spent = end_time - *data->timer;
        *data->timer = 0;
        uint32_t microseconds_spent = k_cyc_to_us_ceil32(cycles_spent);
        double distance = (double)microseconds_spent / 58.0; // Assuming speed of sound in air 343 m/s
        if (k_msgq_put(data->distance_queue, &distance, K_NO_WAIT) != 0)
        {
            printk("echo_callback: failed to put distance into message queue\n");
        }
    }
}

void distance_calculator_thread(void *queue, void *dummy2, void *dummy3)
{
    struct k_msgq *distance_queue = (struct k_msgq *)queue;
    ARG_UNUSED(dummy2);
    ARG_UNUSED(dummy3);

    double distance;

    while (1)
    {
        if (k_msgq_get(distance_queue, &distance, K_FOREVER) == 0)
        {
            printk("distance_calculator_thread: distance: %f\n", distance);
        }
    }
}
