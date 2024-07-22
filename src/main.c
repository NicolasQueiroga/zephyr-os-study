#include "utils.h"
#include "distance.h"

int main(void)
{
    static struct k_thread time_of_flight_thread_data;
    static struct k_thread distance_calculator_thread_data;
    static struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
    static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});
    static btn_callback_data_t button_cb_data = {
        .led = &led,
        .time_of_flight_thread_data = &time_of_flight_thread_data,
    };

    configure_output(&led);
    configure_input(&button, GPIO_INPUT, GPIO_INT_EDGE_TO_ACTIVE);
    configure_input_cb(&button_cb_data.cb, &button);

    k_thread_create(&time_of_flight_thread_data, time_of_flight_thread_stack_area,
                    K_THREAD_STACK_SIZEOF(time_of_flight_thread_stack_area),
                    time_of_flight_thread, &distance_calculator_thread_data, NULL, NULL,
                    PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&time_of_flight_thread_data, "get time of flight thread");

    k_thread_create(&distance_calculator_thread_data, distance_calculator_thread_stack_area,
                    K_THREAD_STACK_SIZEOF(distance_calculator_thread_stack_area),
                    distance_calculator_thread, NULL, NULL, NULL,
                    PRIORITY + 1, 0, K_FOREVER);
    k_thread_name_set(&distance_calculator_thread_data, "calculate distance thread");

    // k_thread_start(&time_of_flight_thread_data);
    k_thread_start(&distance_calculator_thread_data);

    return 0;
}
