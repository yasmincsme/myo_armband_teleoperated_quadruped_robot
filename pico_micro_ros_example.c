#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
#include "aux/i2c_dma.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN  25
#define DT       0.05f

/* ── micro-ROS agent state machine (RESTRIÇÃO 7) ─────────────── */
typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} agent_state_t;

/* ── Hardware objects (persist across reconnects) ─────────────── */
static mpu9250_t mpu9250;
static hcsr04_t  hcsr04;

float x = 0.0f, y = 0.0f, theta = 0.0f;

/* ── ROS entities ─────────────────────────────────────────────── */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rcl_timer_t      ros_timer;
static rclc_executor_t  executor;

static rcl_publisher_t  odom_pub, tf_pub, imu_pub, mag_pub, ir_pub, us_pub;

static nav_msgs__msg__Odometry         odom_msg;
static tf2_msgs__msg__TFMessage        tf_msg;
static sensor_msgs__msg__Imu           imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static sensor_msgs__msg__Range         ir_msg, us_msg;

static bool _entities_ok = false;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    /* Snapshot atômico do PMW3901 — devolve deslocamento acumulado
       desde a última chamada (em frame do corpo) e reseta o acumulador. */
    float dx_m, dy_m, vel_x, vel_y;
    pmw3901_snapshot(&dx_m, &dy_m, &vel_x, &vel_y);

    uint16_t raw;
    float voltage;
    float distance_cm = sharpIR_read(&raw, &voltage);
    float distance_m = distance_cm / 100.0f;

    //

    float gz = mpu9250.gyro[2];
    theta += gz * DT;

    x += dx_m;
    y += dy_m;

    float half = theta * 0.5f;
    float qw = cosf(half);
    float qz = sinf(half);

    //

    int64_t time_ns = rmw_uros_epoch_nanos();
    int32_t sec = (int32_t)(time_ns / 1000000000LL);
    uint32_t nsec = (uint32_t)(time_ns % 1000000000LL);

    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nsec;

    tf_msg.transforms.data[0].header.stamp.sec = sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = nsec;

    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nsec;

    mag_msg.header.stamp.sec = sec;
    mag_msg.header.stamp.nanosec = nsec;

    ir_msg.header.stamp.sec = sec;
    ir_msg.header.stamp.nanosec = nsec;

    us_msg.header.stamp.sec = sec;
    us_msg.header.stamp.nanosec = nsec;

    //Odom

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;

    odom_msg.twist.twist.linear.x = vel_x;
    odom_msg.twist.twist.linear.y = vel_y;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = gz;

    //TF

    tf_msg.transforms.data[0].transform.translation.x = x;
    tf_msg.transforms.data[0].transform.translation.y = y;
    tf_msg.transforms.data[0].transform.translation.z = 0.0;

    tf_msg.transforms.data[0].transform.rotation.w = 1.0;
    tf_msg.transforms.data[0].transform.rotation.x = 0.0;
    tf_msg.transforms.data[0].transform.rotation.y = 0.0;
    tf_msg.transforms.data[0].transform.rotation.z = 0.0;

    //Imu

    imu_msg.linear_acceleration.x = mpu9250.accel[0];
    imu_msg.linear_acceleration.y = mpu9250.accel[1];
    imu_msg.linear_acceleration.z = mpu9250.accel[2];

    imu_msg.angular_velocity.x = mpu9250.gyro[0];
    imu_msg.angular_velocity.y = mpu9250.gyro[1];
    imu_msg.angular_velocity.z = mpu9250.gyro[2];

    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = qz;

    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
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

    //Mag

    mag_msg.magnetic_field.x = mpu9250.mag[0];
    mag_msg.magnetic_field.y = mpu9250.mag[1];
    mag_msg.magnetic_field.z = mpu9250.mag[2];

    //Range

    ir_msg.range = distance_m;
    us_msg.range = hcsr04.distance / 100.0f;

    //

    rcl_publish(&tf_pub,   &tf_msg,   NULL);
    rcl_publish(&odom_pub, &odom_msg, NULL);
    rcl_publish(&imu_pub,  &imu_msg,  NULL);
    rcl_publish(&mag_pub,  &mag_msg,  NULL);
    rcl_publish(&ir_pub,   &ir_msg,   NULL);
    rcl_publish(&us_pub,   &us_msg,   NULL);

    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

/* ── Entity lifecycle ─────────────────────────────────────────── */
static bool create_entities(void) {
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&node, "pico_node", "", &support) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }

#define MKPUB(pub, pkg, mod, type, topic) \
    rclc_publisher_init_default(&(pub), &node, \
        ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, mod, type), topic)

    MKPUB(odom_pub, nav_msgs,    msg, Odometry,     "odom");
    MKPUB(tf_pub,   tf2_msgs,    msg, TFMessage,    "/tf");
    MKPUB(imu_pub,  sensor_msgs, msg, Imu,          "imu/data");
    MKPUB(mag_pub,  sensor_msgs, msg, MagneticField, "imu/mag");
    MKPUB(ir_pub,   sensor_msgs, msg, Range,         "ir_range");
    MKPUB(us_pub,   sensor_msgs, msg, Range,         "us_range");
#undef MKPUB

    /* Odometry */
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,  "base_link");

    /* TF: one transform only (odom → base_link) */
    tf2_msgs__msg__TFMessage__init(&tf_msg);
    tf_msg.transforms.data =
        malloc(sizeof(geometry_msgs__msg__TransformStamped));
    if (!tf_msg.transforms.data) {
        rclc_support_fini(&support);
        return false;
    }
    tf_msg.transforms.size     = 1;
    tf_msg.transforms.capacity = 1;
    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[0]);
    rosidl_runtime_c__String__assign(
        &tf_msg.transforms.data[0].header.frame_id, "odom");
    rosidl_runtime_c__String__assign(
        &tf_msg.transforms.data[0].child_frame_id,  "base_link");

    /* IMU / Mag */
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link");
    sensor_msgs__msg__MagneticField__init(&mag_msg);
    rosidl_runtime_c__String__assign(&mag_msg.header.frame_id, "base_link");

    /* Range sensors — both anchored to base_link to avoid TF expiry */
    sensor_msgs__msg__Range__init(&ir_msg);
    rosidl_runtime_c__String__assign(&ir_msg.header.frame_id, "base_link");
    ir_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    ir_msg.field_of_view  = 0.26f;
    ir_msg.min_range      = 0.10f;
    ir_msg.max_range      = 0.80f;

    sensor_msgs__msg__Range__init(&us_msg);
    rosidl_runtime_c__String__assign(&us_msg.header.frame_id, "base_link");
    us_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    us_msg.field_of_view  = 0.5f;
    us_msg.min_range      = 0.02f;
    us_msg.max_range      = 4.0f;

    /* Timer and executor */
    if (rclc_timer_init_default(
            &ros_timer, &support,
            RCL_MS_TO_NS(50), timer_callback) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    if (rclc_executor_init(
            &executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        rcl_timer_fini(&ros_timer);
        rclc_support_fini(&support);
        return false;
    }
    rclc_executor_add_timer(&executor, &ros_timer);
    return true;
}

static void destroy_entities(void) {
    if (!_entities_ok) return;
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&tf_pub,   &node);
    rcl_publisher_fini(&imu_pub,  &node);
    rcl_publisher_fini(&mag_pub,  &node);
    rcl_publisher_fini(&ir_pub,   &node);
    rcl_publisher_fini(&us_pub,   &node);
    rcl_timer_fini(&ros_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    /* Free TF data manually to avoid double-free */
    if (tf_msg.transforms.data) {
        geometry_msgs__msg__TransformStamped__fini(
            &tf_msg.transforms.data[0]);
        free(tf_msg.transforms.data);
        tf_msg.transforms.data     = NULL;
        tf_msg.transforms.size     = 0;
        tf_msg.transforms.capacity = 0;
    }
    nav_msgs__msg__Odometry__fini(&odom_msg);
    sensor_msgs__msg__Imu__fini(&imu_msg);
    sensor_msgs__msg__MagneticField__fini(&mag_msg);
    sensor_msgs__msg__Range__fini(&ir_msg);
    sensor_msgs__msg__Range__fini(&us_msg);

    _entities_ok = false;
}

/* ── PMW3901 bringup callback ─────────────────────────────────── */
static volatile bool _pmw_failed = false;

static void on_pmw_done(bool ok) {
    if (!ok) {
        _pmw_failed = true;
        return;
    }
    gpio_put(LED_PIN, 1);  /* solid ON: sensor ready */
}

/* ── Main ─────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* PMW3901 async bringup (starts SPI + DMA, non-blocking) */
    pmw3901_init(on_pmw_done);

    /* Other sensors */
    i2c_activate();
    i2c_dma_init(INTERFACE_I2C);
    mpu9250_init(INTERFACE_I2C);
    sharpIR_init();
    hcsr04_init(&hcsr04);

    /* Wait for PMW3901 bringup to complete (interrupts do the work) */
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (!pmw3901_ready()) {
        if (_pmw_failed ||
            to_ms_since_boot(get_absolute_time()) - t0 > 5000u) {
            /* Sensor failed or timed out — blink halt */
            while (1) {
                gpio_put(LED_PIN, 1); busy_wait_ms(100);
                gpio_put(LED_PIN, 0); busy_wait_ms(100);
            }
        }
        tight_loop_contents();
    }

    /* micro-ROS transport */
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    agent_state_t state     = WAITING_AGENT;
    uint64_t      last_ping = 0;

    while (1) {
        /* Service PMW3901 read chain every iteration (RESTRIÇÃO 5) */
        pmw3901_service();

        /* Kick async IMU read whenever the bus is free */
        if (!i2c_dma_busy())
            mpu9250_read_async(&mpu9250, NULL, NULL);

        switch (state) {

        /* Poll until an agent responds */
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                state = AGENT_AVAILABLE;
            break;

        /* Agent seen — sync time then create all ROS entities */
        case AGENT_AVAILABLE:
            rmw_uros_sync_session(1000);
            _entities_ok = create_entities();
            state = _entities_ok ? AGENT_CONNECTED : WAITING_AGENT;
            break;

        /* Normal operation: spin executor, ping every 500 ms */
        case AGENT_CONNECTED: {
            uint64_t now = time_us_64();
            if (now - last_ping >= 500000u) {
                last_ping = now;
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            break;
        }

        /* Agent lost — tear down entities and restart discovery */
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }
}
