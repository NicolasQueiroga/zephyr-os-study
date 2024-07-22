#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#include <inttypes.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define STACKSIZE 1024
#define PRIORITY 7
#define SLEEPTIME 2000

extern K_THREAD_STACK_DEFINE(distance_calculator_thread_stack_area, STACKSIZE);
extern K_THREAD_STACK_DEFINE(time_of_flight_thread_stack_area, STACKSIZE);

typedef struct btn_callabck_data
{
    struct gpio_callback cb;
    struct gpio_dt_spec *led;
    struct k_thread *time_of_flight_thread_data;
} btn_callback_data_t;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void time_of_flight_thread(void *distance_calculator_thread_data, void *dummy2, void *dummy3);
void distance_calculator_thread(void *dummy1, void *dummy2, void *dummy3);

#endif // __DISTANCE_H__
