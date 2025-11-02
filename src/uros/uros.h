#ifndef UROS_H
#define UROS_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize micro-ROS
 * @return 0 on success, negative on error
 */
int uros_init(void);

/**
 * @brief Start micro-ROS ping-pong example
 * This creates a publisher and subscriber for testing
 */
void uros_ping_pong_start(void);

/**
 * @brief Spin micro-ROS executor
 * Call this periodically or in a thread
 */
void uros_spin(void);

#endif // UROS_H
