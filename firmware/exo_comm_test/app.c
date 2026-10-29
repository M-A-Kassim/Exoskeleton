
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define DIAG_LED GPIO_NUM_2

enum connection_state {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static rclc_executor_t executor;

static std_msgs__msg__Int32 received_msg;
static std_msgs__msg__Int32 reply_msg;

static bool support_created = false;
static bool node_created = false;
static bool publisher_created = false;
static bool subscriber_created = false;
static bool executor_created = false;

static void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *incoming =
        (const std_msgs__msg__Int32 *)msgin;

    reply_msg.data = incoming->data + 1;

    rcl_publish(&publisher, &reply_msg, NULL);
}

static void destroy_entities(void)
{
    if (executor_created) {
        rclc_executor_fini(&executor);
        executor_created = false;
    }

    if (subscriber_created) {
        rcl_subscription_fini(&subscriber, &node);
        subscriber_created = false;
    }

    if (publisher_created) {
        rcl_publisher_fini(&publisher, &node);
        publisher_created = false;
    }

    if (node_created) {
        rcl_node_fini(&node);
        node_created = false;
    }

    if (support_created) {
        rclc_support_fini(&support);
        support_created = false;
    }

    // Reset the ROS objects so they can be initialized again.
    executor = rclc_executor_get_zero_initialized_executor();
    subscriber = rcl_get_zero_initialized_subscription();
    publisher = rcl_get_zero_initialized_publisher();
    node = rcl_get_zero_initialized_node();
    support = (rclc_support_t){0};

    gpio_set_level(DIAG_LED, 0);
}

static bool create_entities(void)
{
    support = (rclc_support_t){0};

    if (rclc_support_init(
            &support, 0, NULL, &allocator) != RCL_RET_OK) {
        goto error;
    }
    support_created = true;

    node = rcl_get_zero_initialized_node();

    if (rclc_node_init_default(
            &node, "exo_comm_test", "", &support) != RCL_RET_OK) {
        goto error;
    }
    node_created = true;

    publisher = rcl_get_zero_initialized_publisher();

    if (rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/esp_test_rx") != RCL_RET_OK) {
        goto error;
    }
    publisher_created = true;

    subscriber = rcl_get_zero_initialized_subscription();

    if (rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/esp_test_tx") != RCL_RET_OK) {
        goto error;
    }
    subscriber_created = true;

    executor = rclc_executor_get_zero_initialized_executor();

    if (rclc_executor_init(
            &executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        goto error;
    }
    executor_created = true;

    if (rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &received_msg,
            &subscription_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        goto error;
    }

    gpio_set_level(DIAG_LED, 1);
    return true;

error:
    destroy_entities();
    return false;
}

void appMain(void *arg)
{
    (void)arg;

    gpio_reset_pin(DIAG_LED);
    gpio_set_direction(DIAG_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(DIAG_LED, 0);

    allocator = rcl_get_default_allocator();

    enum connection_state state = WAITING_AGENT;

    while (1) {
        switch (state) {

        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                state = AGENT_CONNECTED;
            } else {
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                state = AGENT_DISCONNECTED;
            } else {
                rclc_executor_spin_some(
                    &executor, RCL_MS_TO_NS(100));
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
