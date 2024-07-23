#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#include <inttypes.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define STACKSIZE 1024
#define QUEUE_SIZE 10
#define PRIORITY 7
#define SLEEPTIME 500
#define FREQUENCY 1 / (2 * 0.000058)
#define SND_SPEED 340.0

extern K_THREAD_STACK_DEFINE(distance_calculator_thread_stack_area, STACKSIZE);

typedef struct btn_callabck_data
{
    struct gpio_callback cb;
    struct gpio_dt_spec *trig;
} btn_callback_data_t;

typedef struct echo_callabck_data
{
    struct gpio_callback cb;
    struct gpio_dt_spec *echo;
    struct k_msgq *distance_queue;
    uint32_t *timer;
} echo_callback_data_t;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void echo_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void distance_calculator_thread(void *time_of_flight_thread_data, void *echo, void *dummy3);

#endif // __DISTANCE_H__
