#include "utils.h"
#include "distance.h"

int main(void)
{
    static struct k_msgq timer_queue;
    static char timer_queue_buffer[QUEUE_SIZE * sizeof(double)];
    static struct k_msgq distance_queue;
    static char distance_queue_buffer[QUEUE_SIZE * sizeof(double)];
    static struct k_thread time_of_flight_thread_data;
    static struct k_thread distance_calculator_thread_data;
    static struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
    static struct gpio_dt_spec echo = GPIO_DT_SPEC_GET_OR(ECHO_NODE, gpios, {0});
    static struct gpio_dt_spec trig = GPIO_DT_SPEC_GET_OR(TRIG_NODE, gpios, {0});
    static btn_callback_data_t button_cb_data = {
        .trig = &trig,
    };
    static uint32_t timer = 0;
    static echo_callback_data_t echo_cb_data = {
        .echo = &echo,
        .distance_queue = &distance_queue,
        .timer = &timer,
    };

    configure_output(&trig, GPIO_OUTPUT);
    gpio_pin_set(trig.port, trig.pin, 0);
    configure_input(&echo, GPIO_INPUT | GPIO_PULL_DOWN, GPIO_INT_EDGE_BOTH);
    configure_input_cb(&echo_cb_data.cb, &echo, echo_callback);
    configure_input(&button, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE);
    configure_input_cb(&button_cb_data.cb, &button, button_pressed);

    k_msgq_init(&timer_queue, timer_queue_buffer, sizeof(double), QUEUE_SIZE);
    k_msgq_init(&distance_queue, distance_queue_buffer, sizeof(double), QUEUE_SIZE);

    k_thread_create(&distance_calculator_thread_data, distance_calculator_thread_stack_area,
                    K_THREAD_STACK_SIZEOF(distance_calculator_thread_stack_area),
                    distance_calculator_thread, &distance_queue, NULL, NULL,
                    PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&distance_calculator_thread_data, "calculate distance thread");
    k_thread_start(&distance_calculator_thread_data);

    return 0;
}
