#include "distance.h"

K_THREAD_STACK_DEFINE(distance_calculator_thread_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(time_of_flight_thread_stack_area, STACKSIZE);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    btn_callback_data_t *data = CONTAINER_OF(cb, btn_callback_data_t, cb);
    const struct gpio_dt_spec *led = data->led;

    gpio_pin_set(led->port, led->pin, 1);
    k_thread_start(data->time_of_flight_thread_data);
    k_busy_wait(10);
    gpio_pin_set(led->port, led->pin, 0);
}

void time_of_flight_thread(void *distance_calculator_thread_data, void *dummy2, void *dummy3)
{
    struct k_thread *distance_calculator_thread = (struct k_thread *)distance_calculator_thread_data;
    ARG_UNUSED(dummy2);
    ARG_UNUSED(dummy3);

    printk("thread_a: thread started \n");

    printk("thread_a: waiting for thread_b to complete \n");
    k_thread_join(distance_calculator_thread, K_FOREVER); // wait forever until thread_b returns

    while (1)
    {
        printk("thread_a: thread loop \n");
        k_msleep(SLEEPTIME);
    }
}

void distance_calculator_thread(void *dummy1, void *dummy2, void *dummy3)
{
    ARG_UNUSED(dummy1);
    ARG_UNUSED(dummy2);
    ARG_UNUSED(dummy3);

    printk("thread_b: thread started \n");

    k_msleep(SLEEPTIME);

    printk("thread_b: returning to thread_a \n");
}
