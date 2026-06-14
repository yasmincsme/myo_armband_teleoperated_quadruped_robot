#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/range.h>

#include "pico_uart_transports.h"
#include "aux/pmw3901.h"
#include "aux/mpu9250.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN 25

/* ── Sensor state (written from IRQ/DMA callbacks, read from timer_callback) ─ */

typedef struct {
    pmw3901_t pmw;
    mpu9250_t imu;
    float     ir_distance;
    float     us_distance;
    float     x;
    float     y;
    float     theta;
} sensor_snapshot_t;

static volatile sensor_snapshot_t _sensors;
static hcsr04_t                   _hcsr04;

/* Accumulated pose – updated inside the PMW3901 DMA callback chain */
static float _x     = 0.0f;
static float _y     = 0.0f;
static float _theta = 0.0f;

/* ── Sensor read pipeline: pmw → mag → snapshot ──────────────────────────── */

static pmw3901_t _pmw_buf;
static mpu9250_t _imu_buf;

static void on_mag_done(void);
static void on_ag_done(void);
static void on_pmw_done(void);

static void on_mag_done(void) {
    /* All sensor data collected – take atomic snapshot */
    uint32_t saved = save_and_disable_interrupts();

    float gz     = _imu_buf.gyro[2];
    _theta      += gz * DT;
    float c      = cosf(_theta);
    float s      = sinf(_theta);
    _x          += _pmw_buf.dx_m * c - _pmw_buf.dy_m * s;
    _y          += _pmw_buf.dx_m * s + _pmw_buf.dy_m * c;

    _sensors.pmw         = _pmw_buf;
    _sensors.imu         = _imu_buf;
    _sensors.ir_distance = sharpIR_read(NULL, NULL);
    _sensors.us_distance = _hcsr04.distance;
    _sensors.x           = _x;
    _sensors.y           = _y;
    _sensors.theta       = _theta;

    restore_interrupts(saved);
}

static void on_ag_done(void) {
    mpu9250_read_mag_async(&_imu_buf, on_mag_done);
}

static void on_pmw_done(void) {
    mpu9250_read_accel_gyro_async(&_imu_buf, on_ag_done);
}

/* ── Repeating timer – kicks off sensor read pipeline every 50 ms ─────────── */

static struct repeating_timer _sensor_timer;

static bool sensor_timer_cb(struct repeating_timer *t) {
    (void)t;
    pmw3901_read_async(&_pmw_buf, on_pmw_done);
    return true;
}

/* ── micro-ROS publishers ─────────────────────────────────────────────────── */

static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rcl_timer_t      timer;
static rclc_executor_t  executor;

static rcl_publisher_t odom_publisher;
static rcl_publisher_t tf_publisher;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t mag_publisher;
static rcl_publisher_t ir_publisher;
static rcl_publisher_t us_publisher;

static nav_msgs__msg__Odometry         odom_msg;
static tf2_msgs__msg__TFMessage        tf_msg;
static sensor_msgs__msg__Imu           imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static sensor_msgs__msg__Range         ir_msg;
static sensor_msgs__msg__Range         us_msg;

/* ── Timer callback – publishes snapshot ──────────────────────────────────── */

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (!timer) return;

    /* Atomic copy */
    uint32_t saved = save_and_disable_interrupts();
    sensor_snapshot_t snap = *(sensor_snapshot_t *)&_sensors;
    restore_interrupts(saved);

    float half = snap.theta * 0.5f;
    float qw   = cosf(half);
    float qz   = sinf(half);

    int64_t  time_ns = rmw_uros_epoch_nanos();
    int32_t  sec     = (int32_t)(time_ns / 1000000000LL);
    uint32_t nsec    = (uint32_t)(time_ns % 1000000000LL);

    odom_msg.header.stamp.sec                      = sec;
    odom_msg.header.stamp.nanosec                  = nsec;
    tf_msg.transforms.data[0].header.stamp.sec     = sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = nsec;
    tf_msg.transforms.data[1].header.stamp.sec     = sec;
    tf_msg.transforms.data[1].header.stamp.nanosec = nsec;
    tf_msg.transforms.data[2].header.stamp.sec     = sec;
    tf_msg.transforms.data[2].header.stamp.nanosec = nsec;
    imu_msg.header.stamp.sec                       = sec;
    imu_msg.header.stamp.nanosec                   = nsec;
    mag_msg.header.stamp.sec                       = sec;
    mag_msg.header.stamp.nanosec                   = nsec;
    ir_msg.header.stamp.sec                        = sec;
    ir_msg.header.stamp.nanosec                    = nsec;
    us_msg.header.stamp.sec                        = sec;
    us_msg.header.stamp.nanosec                    = nsec;

    odom_msg.pose.pose.position.x    = snap.x;
    odom_msg.pose.pose.position.y    = snap.y;
    odom_msg.pose.pose.position.z    = 0.0f;
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.twist.twist.linear.x    = snap.pmw.dx_m / DT;
    odom_msg.twist.twist.linear.y    = snap.pmw.dy_m / DT;
    odom_msg.twist.twist.linear.z    = 0.0f;
    odom_msg.twist.twist.angular.x   = 0.0f;
    odom_msg.twist.twist.angular.y   = 0.0f;
    odom_msg.twist.twist.angular.z   = snap.imu.gyro[2];

    tf_msg.transforms.data[0].transform.translation.x = snap.x;
    tf_msg.transforms.data[0].transform.translation.y = snap.y;
    tf_msg.transforms.data[0].transform.translation.z = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.w    = qw;
    tf_msg.transforms.data[0].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.z    = qz;

    tf_msg.transforms.data[1].transform.translation.x = 0.10f;
    tf_msg.transforms.data[1].transform.translation.y = 0.0f;
    tf_msg.transforms.data[1].transform.translation.z = 0.05f;
    tf_msg.transforms.data[1].transform.rotation.w    = 1.0f;
    tf_msg.transforms.data[1].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.z    = 0.0f;

    tf_msg.transforms.data[2].transform.translation.x = 0.10f;
    tf_msg.transforms.data[2].transform.translation.y = 0.0f;
    tf_msg.transforms.data[2].transform.translation.z = 0.05f;
    tf_msg.transforms.data[2].transform.rotation.w    = 1.0f;
    tf_msg.transforms.data[2].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.z    = 0.0f;

    imu_msg.linear_acceleration.x = snap.imu.accel[0];
    imu_msg.linear_acceleration.y = snap.imu.accel[1];
    imu_msg.linear_acceleration.z = snap.imu.accel[2];
    imu_msg.angular_velocity.x    = snap.imu.gyro[0];
    imu_msg.angular_velocity.y    = snap.imu.gyro[1];
    imu_msg.angular_velocity.z    = snap.imu.gyro[2];
    imu_msg.orientation.w         = qw;
    imu_msg.orientation.x         = 0.0f;
    imu_msg.orientation.y         = 0.0f;
    imu_msg.orientation.z         = qz;

    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]         = 0.0f;
        imu_msg.angular_velocity_covariance[i]    = 0.0f;
        imu_msg.linear_acceleration_covariance[i] = 0.0f;
    }
    imu_msg.orientation_covariance[0]         = 0.01f;
    imu_msg.orientation_covariance[4]         = 0.01f;
    imu_msg.orientation_covariance[8]         = 0.01f;
    imu_msg.angular_velocity_covariance[0]    = 0.01f;
    imu_msg.angular_velocity_covariance[4]    = 0.01f;
    imu_msg.angular_velocity_covariance[8]    = 0.01f;
    imu_msg.linear_acceleration_covariance[0] = 0.1f;
    imu_msg.linear_acceleration_covariance[4] = 0.1f;
    imu_msg.linear_acceleration_covariance[8] = 0.1f;

    mag_msg.magnetic_field.x = snap.imu.mag[0];
    mag_msg.magnetic_field.y = snap.imu.mag[1];
    mag_msg.magnetic_field.z = snap.imu.mag[2];

    ir_msg.range = snap.ir_distance / 100.0f;
    us_msg.range = snap.us_distance / 100.0f;

    rcl_publish(&tf_publisher,   &tf_msg,   NULL);
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&imu_publisher,  &imu_msg,  NULL);
    rcl_publish(&mag_publisher,  &mag_msg,  NULL);
    rcl_publish(&ir_publisher,   &ir_msg,   NULL);
    rcl_publish(&us_publisher,   &us_msg,   NULL);

    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

/* ── micro-ROS entity lifecycle ──────────────────────────────────────────── */

void destroy_entities(void);

bool create_entities(void) {
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pico_node", "", &support) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }

    rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,    msg, Odometry),      "odom");
    rclc_publisher_init_default(&tf_publisher,   &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs,    msg, TFMessage),     "/tf");
    rclc_publisher_init_default(&imu_publisher,  &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),           "imu/data");
    rclc_publisher_init_default(&mag_publisher,  &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag");
    rclc_publisher_init_default(&ir_publisher,   &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),         "ir_range");
    rclc_publisher_init_default(&us_publisher,   &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),         "us_range");

    nav_msgs__msg__Odometry__init(&odom_msg);
    tf2_msgs__msg__TFMessage__init(&tf_msg);
    sensor_msgs__msg__Imu__init(&imu_msg);
    sensor_msgs__msg__MagneticField__init(&mag_msg);
    sensor_msgs__msg__Range__init(&ir_msg);
    sensor_msgs__msg__Range__init(&us_msg);

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,  "base_link");
    rosidl_runtime_c__String__assign(&ir_msg.header.frame_id,   "ir_link");
    rosidl_runtime_c__String__assign(&us_msg.header.frame_id,   "us_link");

    tf_msg.transforms.data = malloc(sizeof(geometry_msgs__msg__TransformStamped) * 3);
    if (!tf_msg.transforms.data) { destroy_entities(); return false; }
    tf_msg.transforms.size     = 3;
    tf_msg.transforms.capacity = 3;

    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[0]);
    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[1]);
    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[2]);

    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[0].header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[0].child_frame_id,  "base_link");
    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[1].header.frame_id, "base_link");
    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[1].child_frame_id,  "ir_link");
    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[2].header.frame_id, "base_link");
    rosidl_runtime_c__String__assign(&tf_msg.transforms.data[2].child_frame_id,  "us_link");

    ir_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    ir_msg.field_of_view  = 0.26f;
    ir_msg.min_range      = 0.10f;
    ir_msg.max_range      = 0.80f;

    us_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    us_msg.field_of_view  = 0.5f;
    us_msg.min_range      = 0.02f;
    us_msg.max_range      = 4.0f;

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    rmw_uros_sync_session(1000);
    return true;
}

void destroy_entities(void) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&tf_publisher,   &node);
    rcl_publisher_fini(&imu_publisher,  &node);
    rcl_publisher_fini(&mag_publisher,  &node);
    rcl_publisher_fini(&ir_publisher,   &node);
    rcl_publisher_fini(&us_publisher,   &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    nav_msgs__msg__Odometry__fini(&odom_msg);
    tf2_msgs__msg__TFMessage__fini(&tf_msg);
    sensor_msgs__msg__Imu__fini(&imu_msg);
    sensor_msgs__msg__MagneticField__fini(&mag_msg);
    sensor_msgs__msg__Range__fini(&ir_msg);
    sensor_msgs__msg__Range__fini(&us_msg);
}

/* ── main ────────────────────────────────────────────────────────────────── */

static volatile bool _pmw_ready = false;
static volatile bool _mpu_ready = false;

static void on_pmw_init_done(void) { _pmw_ready = true; }
static void on_mpu_init_done(void) { _mpu_ready = true; }

int main(void) {
    stdio_init_all();
    allocator = rcl_get_default_allocator();

    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Sensor init (non-blocking state machines) */
    pmw3901_init(on_pmw_init_done);
    mpu9250_init(on_mpu_init_done);
    sharpIR_init();
    hcsr04_init(&_hcsr04);

    while (!_pmw_ready || !_mpu_ready) {
        pmw3901_init_poll();
        mpu9250_init_poll();
        tight_loop_contents();
    }

    /* Start sensor pipeline timer (50 ms period) */
    add_repeating_timer_ms(50, sensor_timer_cb, NULL, &_sensor_timer);

    typedef enum {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } agent_state_t;

    agent_state_t state = WAITING_AGENT;

    while (true) {
        switch (state) {
            case WAITING_AGENT: {
                static int64_t t = -1;
                if (t < 0) t = (int64_t)(time_us_64() / 1000);
                if ((int64_t)(time_us_64() / 1000) - t > 500) {
                    state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                            ? AGENT_AVAILABLE : WAITING_AGENT;
                    t = -1;
                }
                break;
            }
            case AGENT_AVAILABLE:
                state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) destroy_entities();
                break;

            case AGENT_CONNECTED: {
                static int64_t t = -1;
                if (t < 0) t = (int64_t)(time_us_64() / 1000);
                if ((int64_t)(time_us_64() / 1000) - t > 500) {
                    state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                            ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                    t = -1;
                }
                if (state == AGENT_CONNECTED)
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                break;
            }
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
        }
    }
}
