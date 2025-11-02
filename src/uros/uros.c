#include "uros.h"
#include <zephyr/kernel.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Macro for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ROS 2 objects
static rcl_publisher_t ping_publisher;
static rcl_subscription_t pong_subscriber;
static std_msgs__msg__Int32 ping_msg;
static std_msgs__msg__Int32 pong_msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

// Ping counter
static int ping_count = 0;

/**
 * @brief Subscription callback - receives pong messages
 */
static void pong_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    printk("Pong received: %d\n", msg->data);
}

/**
 * @brief Timer callback - publishes ping messages
 */
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL) {
        ping_msg.data = ping_count++;
        rcl_ret_t ret = rcl_publish(&ping_publisher, &ping_msg, NULL);
        if (ret == RCL_RET_OK) {
            printk("Ping sent: %d\n", ping_msg.data);
        } else {
            printk("Error publishing ping: %d\n", ret);
        }
    }
}

int uros_init(void)
{
    printk("Initializing micro-ROS...\n");

    // Initialize allocator
    allocator = rcl_get_default_allocator();

    // Create init options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK) {
        printk("Failed to initialize init options: %d\n", ret);
        return -1;
    }

    // Initialize rclc support
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (ret != RCL_RET_OK) {
        printk("Failed to initialize rclc support: %d\n", ret);
        return -1;
    }

    // Create node
    ret = rclc_node_init_default(&node, "zephyr_microros_node", "", &support);
    if (ret != RCL_RET_OK) {
        printk("Failed to create node: %d\n", ret);
        return -1;
    }

    printk("micro-ROS initialized successfully\n");
    return 0;
}

void uros_ping_pong_start(void)
{
    printk("Starting micro-ROS ping-pong example...\n");

    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        &ping_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ping"));

    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &pong_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pong"));

    // Create timer (1 Hz = 1000ms)
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &pong_msg,
                                           &pong_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    printk("Ping-pong example started. Publishing to 'ping' and subscribing to 'pong'\n");
}

void uros_spin(void)
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
