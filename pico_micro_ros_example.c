#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <stdlib.h>

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
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>

#include "pico_uart_transports.h"
#include "aux/pmw3901.h"
#include "aux/mpu9250.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN 25

/* ── Odometry state ─────────────────────────────────────────────────────── */
static float x     = 0.0f;
static float y     = 0.0f;
static float theta = 0.0f;

pmw3901_t pmw3901;
mpu9250_t mpu9250;
hcsr04_t  hcsr04;

static volatile float _ir_range = 0.0f;

/* ── micro-ROS entities ─────────────────────────────────────────────────── */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rcl_timer_t      timer;
static rclc_executor_t  executor;

static rcl_publisher_t odom_publisher;
static nav_msgs__msg__Odometry odom_msg;

static rcl_publisher_t tf_publisher;
static tf2_msgs__msg__TFMessage tf_msg;

static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;

static rcl_publisher_t mag_publisher;
static sensor_msgs__msg__MagneticField mag_msg;

static rcl_publisher_t ir_publisher;
static sensor_msgs__msg__Range ir_msg;

static rcl_publisher_t us_publisher;
static sensor_msgs__msg__Range us_msg;

/* ── Sensor pipeline callbacks (DMA/alarm IRQ context) ─────────────────── */
static void on_mag_done(void *u) {
    (void)u;
    uint16_t raw; float voltage;
    _ir_range = sharpIR_read(&raw, &voltage);
}

static void on_ag_done(void *u) {
    (void)u;
    mpu9250_read_mag_async(&mpu9250, on_mag_done, NULL);
}

/* ── ROS timer callback: snapshot → publish → kick IMU read ────────────── */
void timer_callback(rcl_timer_t *t, int64_t last_call_time) {
    (void)last_call_time;
    if (t == NULL) return;

    /* Integrate odometry (uses values written by the previous async read) */
    float gz = mpu9250.gyro[2];
    theta += gz * DT;

    float c = cosf(theta);
    float s = sinf(theta);
    x += pmw3901.dx_m * c - pmw3901.dy_m * s;
    y += pmw3901.dx_m * s + pmw3901.dy_m * c;

    float half = theta * 0.5f;
    float qw   = cosf(half);
    float qz   = sinf(half);

    int64_t  time_ns = rmw_uros_epoch_nanos();
    int32_t  sec     = (int32_t)(time_ns / 1000000000LL);
    uint32_t nsec    = (uint32_t)(time_ns % 1000000000LL);

    /* Odom */
    odom_msg.header.stamp.sec     = sec;
    odom_msg.header.stamp.nanosec = nsec;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0f;
    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.twist.twist.linear.x  = pmw3901.dx_m / DT;
    odom_msg.twist.twist.linear.y  = pmw3901.dy_m / DT;
    odom_msg.twist.twist.linear.z  = 0.0f;
    odom_msg.twist.twist.angular.x = 0.0f;
    odom_msg.twist.twist.angular.y = 0.0f;
    odom_msg.twist.twist.angular.z = gz;

    /* TF odom→base_link */
    tf_msg.transforms.data[0].header.stamp.sec     = sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = nsec;
    tf_msg.transforms.data[0].transform.translation.x = x;
    tf_msg.transforms.data[0].transform.translation.y = y;
    tf_msg.transforms.data[0].transform.translation.z = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.w    = qw;
    tf_msg.transforms.data[0].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.z    = qz;

    /* TF base_link→ir_link */
    tf_msg.transforms.data[1].header.stamp.sec     = sec;
    tf_msg.transforms.data[1].header.stamp.nanosec = nsec;
    tf_msg.transforms.data[1].transform.translation.x = 0.10f;
    tf_msg.transforms.data[1].transform.translation.y = 0.0f;
    tf_msg.transforms.data[1].transform.translation.z = 0.05f;
    tf_msg.transforms.data[1].transform.rotation.w    = 1.0f;
    tf_msg.transforms.data[1].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.z    = 0.0f;

    /* TF base_link→us_link */
    tf_msg.transforms.data[2].header.stamp.sec     = sec;
    tf_msg.transforms.data[2].header.stamp.nanosec = nsec;
    tf_msg.transforms.data[2].transform.translation.x = 0.10f;
    tf_msg.transforms.data[2].transform.translation.y = 0.0f;
    tf_msg.transforms.data[2].transform.translation.z = 0.05f;
    tf_msg.transforms.data[2].transform.rotation.w    = 1.0f;
    tf_msg.transforms.data[2].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.z    = 0.0f;

    /* IMU */
    imu_msg.header.stamp.sec     = sec;
    imu_msg.header.stamp.nanosec = nsec;
    imu_msg.linear_acceleration.x = mpu9250.accel[0];
    imu_msg.linear_acceleration.y = mpu9250.accel[1];
    imu_msg.linear_acceleration.z = mpu9250.accel[2];
    imu_msg.angular_velocity.x    = mpu9250.gyro[0];
    imu_msg.angular_velocity.y    = mpu9250.gyro[1];
    imu_msg.angular_velocity.z    = mpu9250.gyro[2];
    imu_msg.orientation.w         = qw;
    imu_msg.orientation.x         = 0.0f;
    imu_msg.orientation.y         = 0.0f;
    imu_msg.orientation.z         = qz;
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]         = 0.0;
        imu_msg.angular_velocity_covariance[i]    = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    imu_msg.linear_acceleration_covariance[0] = 0.1;
    imu_msg.linear_acceleration_covariance[4] = 0.1;
    imu_msg.linear_acceleration_covariance[8] = 0.1;

    /* Mag */
    mag_msg.header.stamp.sec     = sec;
    mag_msg.header.stamp.nanosec = nsec;
    mag_msg.magnetic_field.x = mpu9250.mag[0];
    mag_msg.magnetic_field.y = mpu9250.mag[1];
    mag_msg.magnetic_field.z = mpu9250.mag[2];

    /* Range */
    ir_msg.header.stamp.sec     = sec;
    ir_msg.header.stamp.nanosec = nsec;
    ir_msg.range = _ir_range;
    us_msg.range = hcsr04.distance / 100.0f;

    rcl_publish(&tf_publisher,   &tf_msg,   NULL);
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&imu_publisher,  &imu_msg,  NULL);
    rcl_publish(&mag_publisher,  &mag_msg,  NULL);
    rcl_publish(&ir_publisher,   &ir_msg,   NULL);
    rcl_publish(&us_publisher,   &us_msg,   NULL);

    gpio_put(LED_PIN, !gpio_get(LED_PIN));

    /* Kick sensor reads for next cycle (non-blocking, complete well before next timer fire) */
    pmw3901_read_async(&pmw3901, NULL);
    mpu9250_read_accel_gyro_async(&mpu9250, on_ag_done, NULL);
}

/* ── create / destroy entities ──────────────────────────────────────────── */
static bool create_entities(void) {
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pico_pmw3901", "", &support) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }

    rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
    rclc_publisher_init_default(&tf_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf");
    rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
    rclc_publisher_init_default(&mag_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag");
    rclc_publisher_init_default(&ir_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "ir_range");
    rclc_publisher_init_default(&us_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "us_range");

    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,  "base_link");

    tf2_msgs__msg__TFMessage__init(&tf_msg);
    tf_msg.transforms.data =
        malloc(sizeof(geometry_msgs__msg__TransformStamped) * 3);
    if (!tf_msg.transforms.data) { rclc_support_fini(&support); return false; }
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

    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link");

    sensor_msgs__msg__MagneticField__init(&mag_msg);
    rosidl_runtime_c__String__assign(&mag_msg.header.frame_id, "base_link");

    sensor_msgs__msg__Range__init(&ir_msg);
    rosidl_runtime_c__String__assign(&ir_msg.header.frame_id, "ir_link");
    ir_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    ir_msg.field_of_view  = 0.26f;
    ir_msg.min_range      = 0.10f;
    ir_msg.max_range      = 0.80f;

    sensor_msgs__msg__Range__init(&us_msg);
    rosidl_runtime_c__String__assign(&us_msg.header.frame_id, "us_link");
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

static void destroy_entities(void) {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

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
    free(tf_msg.transforms.data);
    tf_msg.transforms.data     = NULL;
    tf_msg.transforms.size     = 0;
    tf_msg.transforms.capacity = 0;
    sensor_msgs__msg__Imu__fini(&imu_msg);
    sensor_msgs__msg__MagneticField__fini(&mag_msg);
    sensor_msgs__msg__Range__fini(&ir_msg);
    sensor_msgs__msg__Range__fini(&us_msg);
}

/* ── main ───────────────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    allocator = rcl_get_default_allocator();

    /* I2C and other sensors (blocking, independent of SPI bringup) */
    i2c_activate();
    mpu9250_init(INTERFACE_I2C);
    sharpIR_init();
    hcsr04_init(&hcsr04);

    /* PMW3901 init: blocking phase-1 writes (~15 ms) + 100 ms async alarm + phase-2 writes. */
    gpio_put(LED_PIN, 1);
    pmw3901_init(NULL);
    while (!pmw3901_init_done())
        pmw3901_init_poll();
    gpio_put(LED_PIN, 0);

    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

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
                /* Slow 1 Hz blink while searching for agent */
                gpio_put(LED_PIN, (time_us_64() / 500000) & 1);
                static int64_t t = -1;
                if (t < 0) t = (int64_t)(time_us_64() / 1000);
                if ((int64_t)(time_us_64() / 1000) - t > 500) {
                    state = (rmw_uros_ping_agent(500, 1) == RMW_RET_OK)
                            ? AGENT_AVAILABLE : WAITING_AGENT;
                    t = (int64_t)(time_us_64() / 1000);
                }
                break;
            }
            case AGENT_AVAILABLE:
                gpio_put(LED_PIN, 0);
                if (create_entities()) {
                    state = AGENT_CONNECTED;
                } else {
                    /* create_entities() cleans up partial init internally */
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED: {
                static int64_t t = -1;
                if (t < 0) t = (int64_t)(time_us_64() / 1000);
                if ((int64_t)(time_us_64() / 1000) - t > 500) {
                    state = (rmw_uros_ping_agent(500, 1) == RMW_RET_OK)
                            ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                    t = (int64_t)(time_us_64() / 1000);
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
