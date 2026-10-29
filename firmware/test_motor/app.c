#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/can.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <exo_interfaces/srv/motor_cmd.h>

// ---------------- CAN Configuration ----------------
#define CAN_TX_PIN      GPIO_NUM_21
#define CAN_RX_PIN      GPIO_NUM_22

#define MOTOR_ID        0x06
#define MIT_CONTROL_ID  8

#define P_MIN  (-12.56f)
#define P_MAX  ( 12.56f)
#define V_MIN  (-60.0f)
#define V_MAX  ( 60.0f)
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN  (-12.0f)
#define T_MAX  ( 12.0f)

#define PI_F 3.14159265f

#define FEEDBACK_TIMEOUT_MS 300

// ---------------- micro-ROS State ----------------
static exo_interfaces__srv__MotorCmd_Request req_msg;
static exo_interfaces__srv__MotorCmd_Response res_msg;

static rcl_service_t motor_service;

// ---------------- Feedback State ----------------
static float fb_abs = 0.0f;
static float fb_cur = 0.0f;

static int8_t fb_temp = 0;
static uint8_t fb_err = 0;

static bool fb_seen = false;
static uint32_t fb_time = 0;

// ---------------- Time ----------------
static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// ---------------- Float to Integer ----------------
static uint32_t float_to_uint(
    float value,
    float min_value,
    float max_value,
    uint8_t bits)
{
    if (!isfinite(value)) {
        value = min_value;
    }

    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;

    uint32_t max_int = (1UL << bits) - 1UL;

    return (uint32_t)(
        (value - min_value) *
        ((float)max_int / (max_value - min_value))
    );
}

// ---------------- Send MIT Command ----------------
static bool send_mit(
    float position,
    float velocity,
    float kp,
    float kd,
    float torque)
{
    uint32_t p_int  = float_to_uint(position, P_MIN, P_MAX, 16);
    uint32_t v_int  = float_to_uint(velocity, V_MIN, V_MAX, 12);
    uint32_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t t_int  = float_to_uint(torque, T_MIN, T_MAX, 12);

    can_message_t message;
    memset(&message, 0, sizeof(message));

    message.identifier = ((uint32_t)MIT_CONTROL_ID << 8) | MOTOR_ID;
    message.extd = 1;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = (uint8_t)(kp_int >> 4);
    message.data[1] = (uint8_t)(((kp_int & 0x0F) << 4) | (kd_int >> 8));
    message.data[2] = (uint8_t)(kd_int & 0xFF);

    message.data[3] = (uint8_t)(p_int >> 8);
    message.data[4] = (uint8_t)(p_int & 0xFF);

    message.data[5] = (uint8_t)(v_int >> 4);
    message.data[6] = (uint8_t)(((v_int & 0x0F) << 4) | (t_int >> 8));
    message.data[7] = (uint8_t)(t_int & 0xFF);

    return can_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK;
}

// ---------------- Feedback ----------------
static void drain_can(void)
{
    can_message_t message;

    while (can_receive(&message, 0) == ESP_OK) {
        if (message.rtr ||
            !message.extd ||
            message.data_length_code != 8) {
            continue;
        }

        if ((message.identifier & 0xFF) != MOTOR_ID) {
            continue;
        }

        int16_t position_raw =
            (int16_t)(((uint16_t)message.data[0] << 8) |
                      message.data[1]);

        int16_t current_raw =
            (int16_t)(((uint16_t)message.data[4] << 8) |
                      message.data[5]);

        fb_abs = ((float)position_raw * 0.1f) * PI_F / 180.0f;
        fb_cur = (float)current_raw * 0.01f;
        fb_temp = (int8_t)message.data[6];
        fb_err = message.data[7];

        fb_seen = true;
        fb_time = now_ms();
    }
}

static bool feedback_ok(void)
{
    uint32_t now = now_ms();

    return fb_seen &&
           (now - fb_time <= FEEDBACK_TIMEOUT_MS) &&
           (fb_err == 0);
}

// ---------------- Service Callback ----------------
static void motor_service_callback(
    const void *request_msg,
    void *response_msg)
{
    const exo_interfaces__srv__MotorCmd_Request *request = request_msg;
    exo_interfaces__srv__MotorCmd_Response *response = response_msg;

    response->success = false;
    rosidl_runtime_c__String__assign(
        &response->message, "Command rejected");

    // Validate all inputs before sending anything.
    if (!isfinite(request->kp) ||
        !isfinite(request->kd) ||
        !isfinite(request->pos) ||
        !isfinite(request->vel) ||
        !isfinite(request->torque)) {

        rosidl_runtime_c__String__assign(
            &response->message, "Inputs must be finite numbers");
        return;
    }

    if (request->kp < KP_MIN || request->kp > KP_MAX ||
        request->kd < KD_MIN || request->kd > KD_MAX ||
        request->pos < P_MIN || request->pos > P_MAX ||
        request->vel < V_MIN || request->vel > V_MAX ||
        request->torque < T_MIN || request->torque > T_MAX) {

        rosidl_runtime_c__String__assign(
            &response->message, "Command outside allowed range");
        return;
    }

    // Require recent feedback before accepting a motor command.
    if (!feedback_ok()) {
        rosidl_runtime_c__String__assign(
            &response->message, "No valid motor feedback");
        return;
    }

    // Send one MIT CAN frame for this service request.
    if (!send_mit(
            request->pos,
            request->vel,
            request->kp,
            request->kd,
            request->torque)) {

        rosidl_runtime_c__String__assign(
            &response->message, "CAN transmit failed");
        return;
    }

    response->success = true;
    rosidl_runtime_c__String__assign(
        &response->message, "MIT CAN frame transmitted");
}

// ---------------- Error Handling ----------------
#define RCCHECK(fn)                                      \
    do {                                                 \
        rcl_ret_t rc = (fn);                             \
        if (rc != RCL_RET_OK) {                          \
            vTaskDelete(NULL);                           \
            return;                                      \
        }                                                \
    } while (0)

// ---------------- Main Application ----------------
void appMain(void *arg)
{
    (void)arg;

    // Start CAN.
    can_general_config_t general_config =
        CAN_GENERAL_CONFIG_DEFAULT(
            CAN_TX_PIN,
            CAN_RX_PIN,
            CAN_MODE_NORMAL);

    general_config.rx_queue_len = 64;
    general_config.tx_queue_len = 16;

    can_timing_config_t timing_config = CAN_TIMING_CONFIG_1MBITS();
    can_filter_config_t filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    if (can_driver_install(
            &general_config,
            &timing_config,
            &filter_config) != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }

    if (can_start() != ESP_OK) {
        can_driver_uninstall();
        vTaskDelete(NULL);
        return;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;
    RCCHECK(rclc_support_init(
        &support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(
        &node,
        "esp32_motor",
        "",
        &support));

    RCCHECK(rclc_service_init_default(
        &motor_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(
            exo_interfaces, srv, MotorCmd),
        "/motor_cmd"));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator));

    RCCHECK(rclc_executor_add_service(
        &executor,
        &motor_service,
        &req_msg,
        &res_msg,
        motor_service_callback));

    while (1) {
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(5));

        drain_can();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
//ros2 service call /motor_cmd exo_interfaces/srv/MotorCmd "{kp: 500.0, kd: 0.29, pos: 0.0, vel: 0.0, torque: 0.0}"