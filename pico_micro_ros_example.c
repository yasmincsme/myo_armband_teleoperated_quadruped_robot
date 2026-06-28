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
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/u_int32.h>

#include "pico_uart_transports.h"
#include "hardware/uart.h"
#include "aux/pmw3901.h"
#include "aux/mpu9250.h"
#include "aux/i2c_dma.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN  25
#define DT       0.05f

/* ── Hardware objects ─────────────────────────────────────────── */
static mpu9250_t mpu9250;
static hcsr04_t  hcsr04;

float x = 0.0f, y = 0.0f, theta = 0.0f;

/* ── ROS entities ─────────────────────────────────────────────── */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rcl_timer_t      ros_timer;
static rclc_executor_t  executor;

static rcl_publisher_t  odom_pub, tf_pub, imu_pub, mag_pub, ir_pub, us_pub, fused_pub;
static rcl_publisher_t  teleop_lat_pub, obstacle_lat_pub;
static rcl_subscription_t        cmd_vel_sub;

static nav_msgs__msg__Odometry         odom_msg;
static tf2_msgs__msg__TFMessage        tf_msg;
static sensor_msgs__msg__Imu           imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static sensor_msgs__msg__Range         ir_msg, us_msg, fused_msg;
static geometry_msgs__msg__Twist       cmd_vel_msg;
static std_msgs__msg__UInt32           teleop_lat_msg, obstacle_lat_msg;

/* ── SSC-32 servo controller (UART0 / GPIO 0=TX, 1=RX) ───────── */
#define SSC_UART      uart1
#define SSC_TX_PIN    8
#define SSC_RX_PIN    9
#define SSC_BAUD      115200
#define OBSTACLE_M    0.15f
#define GAIT_QUERY_US 20000u

static const char REST_CMD[] =
    "#0 P1200 #1 P1100 #2 P1100 "
    "#14 P1800 #13 P1800 #12 P2000 "
    "#16 P1800 #17 P1800 #18 P1900 "
    "#30 P1300 #29 P1300 #28 P1000 T200";

#define GAIT_FWD_N   10
#define GAIT_LEFT_N  10
#define GAIT_RIGHT_N 10

static const char *GAIT_FWD[GAIT_FWD_N] = {
    "#0 P1200 #1 P1100 #2 P1100 #14 P1800 #13 P1700 #12 P2000 #16 P1800 #17 P1800 #18 P1400 #30 P1700 #29 P1400 #28 P900 T200",
    "#30 P1300 #29 P1100 #28 P1600 T100",
    "#30 P1300 #29 P1300 #28 P1600 T100",
    "#16 P1800 #17 P2000 #18 P1900 T100",
    "#16 P1600 #17 P1800 #18 P2000 T100",
    "#0 P1200 #1 P1100 #2 P1600 #14 P1600 #13 P1700 #12 P2100 #16 P1800 #17 P1800 #18 P1900 #30 P1300 #29 P1300 #28 P1000 T200",
    "#14 P1800 #13 P2000 #12 P1500 T100",
    "#14 P1800 #13 P1700 #12 P1500 T100",
    "#0 P1300 #1 P1000 #2 P1000 T100",
    "#0 P1500 #1 P1200 #2 P900 T100",
};

static const char *GAIT_LEFT[GAIT_LEFT_N] = {
    "#0 P1200 #1 P1100 #2 P1100 #14 P1800 #13 P1700 #12 P2000 #16 P1800 #17 P1800 #18 P1900 #30 P1300 #29 P1300 #28 P1000 T200",
    "#30 P1300 #29 P1100 #28 P1600 T100",
    "#30 P1300 #29 P1300 #28 P1600 T100",
    "#0 P1200 #1 P1000 #2 P1600 T100",
    "#0 P1200 #1 P1200 #2 P1600 T100",
    "#0 P1200 #1 P1100 #2 P1100 #14 P1800 #13 P1700 #12 P2000 #16 P1800 #17 P1800 #18 P1900 #30 P1300 #29 P1300 #28 P1000 T200",
    "#16 P1800 #17 P2000 #18 P2200 T100",
    "#16 P1800 #17 P1800 #18 P2200 T100",
    "#14 P1800 #13 P2000 #12 P2300 T100",
    "#14 P1800 #13 P1700 #12 P2300 T100",
};

static const char *GAIT_RIGHT[GAIT_RIGHT_N] = {
    "#0 P1200 #1 P1100 #2 P1100 #14 P1800 #13 P1700 #12 P2000 #16 P1800 #17 P1800 #18 P1900 #30 P1300 #29 P1300 #28 P1000 T200",
    "#30 P1300 #29 P1100 #28 P700 T100",
    "#30 P1300 #29 P1300 #28 P700 T100",
    "#0 P1200 #1 P1000 #2 P800 T100",
    "#0 P1200 #1 P1200 #2 P800 T100",
    "#0 P1200 #1 P1100 #2 P1100 #14 P1800 #13 P1700 #12 P2000 #16 P1800 #17 P1800 #18 P1900 #30 P1300 #29 P1300 #28 P1000 T200",
    "#16 P1800 #17 P2000 #18 P1400 T100",
    "#16 P1800 #17 P1800 #18 P1400 T100",
    "#14 P1800 #13 P2000 #12 P1500 T100",
    "#14 P1800 #13 P1700 #12 P1500 T100",
};

typedef enum { MOTION_NONE, MOTION_FORWARD, MOTION_LEFT, MOTION_RIGHT } motion_mode_t;
typedef enum { GAIT_IDLE, GAIT_MOVING, GAIT_QUERYING } gait_fsm_t;

static volatile motion_mode_t _motion    = MOTION_NONE;
static gait_fsm_t             _gait_fsm  = GAIT_IDLE;
static int                    _gait_step = 0;
static uint64_t               _last_q_us = 0;

/* ── Latency measurement ──────────────────────────────────────── */
static volatile uint32_t _teleop_latency_us  = 0;
static volatile uint64_t _obs_start_us        = 0;
static volatile bool     _obs_measuring        = false;
static volatile uint32_t _obstacle_latency_us  = 0;

/* ── Kalman filter state for IR + sonar fusion ────────────────── */
#define KF_Q          0.001f  /* process noise variance (m²)                  */
#define KF_R_IR_NEAR  0.003f  /* IR noise at close range  → more trusted      */
#define KF_R_IR_FAR   0.020f  /* IR noise at far range    → less trusted      */
#define KF_R_US_NEAR  0.020f  /* sonar noise at close range → less trusted    */
#define KF_R_US_FAR   0.003f  /* sonar noise at far range   → more trusted    */
static float _kf_x = 0.5f;   /* state estimate (m)                            */
static float _kf_P = 1.0f;   /* error covariance                              */

/* ── SSC-32 helpers ───────────────────────────────────────────── */
static void ssc_send(const char *cmd) {
    uart_puts(SSC_UART, cmd);
    uart_putc(SSC_UART, '\r');
}

static void ssc_step_send(void) {
    const char **steps;
    int n;
    switch (_motion) {
        case MOTION_FORWARD: steps = GAIT_FWD;   n = GAIT_FWD_N;   break;
        case MOTION_LEFT:    steps = GAIT_LEFT;  n = GAIT_LEFT_N;  break;
        case MOTION_RIGHT:   steps = GAIT_RIGHT; n = GAIT_RIGHT_N; break;
        default: return;
    }
    ssc_send(steps[_gait_step % n]);
}

/* Called every main-loop iteration; advances gait only when SSC-32
   confirms the previous move is done AND no obstacle is within 15 cm. */
static void gait_service(void) {
    if (_gait_fsm == GAIT_IDLE) return;

    uint64_t now = time_us_64();

    if (_gait_fsm == GAIT_MOVING) {
        if (now - _last_q_us >= GAIT_QUERY_US) {
            uart_puts(SSC_UART, "Q\r");
            _last_q_us = now;
            _gait_fsm  = GAIT_QUERYING;
        }
        return;
    }

    /* GAIT_QUERYING: wait for SSC-32 response byte */
    if (!uart_is_readable(SSC_UART)) return;

    char c = (char)uart_getc(SSC_UART);
    if (c == '+') {
        /* Still moving — re-query after interval */
        _gait_fsm = GAIT_MOVING;
    } else if (c == '.') {
        /* Move complete — check obstacle before advancing */
        if (_kf_x <= OBSTACLE_M) {
            ssc_send(REST_CMD);
            if (_obs_measuring) {
                _obstacle_latency_us = (uint32_t)(time_us_64() - _obs_start_us);
                _obs_measuring = false;
            }
            _gait_fsm = GAIT_IDLE;
            _motion   = MOTION_NONE;
        } else {
            _gait_step++;
            ssc_step_send();
            _last_q_us = time_us_64();
            _gait_fsm  = GAIT_MOVING;
        }
    }
    /* other chars (echoes, CR/LF): ignore */
}

static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = msgin;
    float lx = msg->linear.x;
    float az = msg->angular.z;

    motion_mode_t m;
    if (fabsf(lx) < 0.05f && fabsf(az) < 0.05f)
        m = MOTION_NONE;
    else if (fabsf(az) > 0.05f)
        m = (az > 0.0f) ? MOTION_LEFT : MOTION_RIGHT;
    else
        m = MOTION_FORWARD;

    if (m == _motion) return;

    uint64_t _t0 = time_us_64();

    _motion    = m;
    _gait_step = 0;
    if (m == MOTION_NONE) {
        ssc_send(REST_CMD);
        _gait_fsm = GAIT_IDLE;
    } else {
        ssc_step_send();
        _last_q_us = time_us_64();
        _gait_fsm  = GAIT_MOVING;
    }

    _teleop_latency_us = (uint32_t)(time_us_64() - _t0);
}

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

    x += dx_m * cosf(theta) - dy_m * sinf(theta);
    y += dx_m * sinf(theta) + dy_m * cosf(theta);

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

    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = qz;

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

    tf_msg.transforms.data[0].transform.rotation.w = qw;
    tf_msg.transforms.data[0].transform.rotation.x = 0.0;
    tf_msg.transforms.data[0].transform.rotation.y = 0.0;
    tf_msg.transforms.data[0].transform.rotation.z = qz;

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

    float us_m = hcsr04.distance / 100.0f;
    ir_msg.range = distance_m;
    us_msg.range = us_m;

    //Kalman fusion

    _kf_P += KF_Q;

    /* t in [0,1]: 0 = close (0.02m), 1 = far (4.0m) */
    float t = fmaxf(0.0f, fminf(1.0f, _kf_x / 4.0f));
    float r_ir_dyn = KF_R_IR_NEAR + t * (KF_R_IR_FAR - KF_R_IR_NEAR);
    float r_us_dyn = KF_R_US_NEAR + t * (KF_R_US_FAR - KF_R_US_NEAR);

    float r_ir = (distance_m >= 0.10f && distance_m <= 0.80f) ? r_ir_dyn : 100.0f;
    float k_ir = _kf_P / (_kf_P + r_ir);
    _kf_x += k_ir * (distance_m - _kf_x);
    _kf_P *= (1.0f - k_ir);

    if (us_m >= 0.02f && us_m <= 4.0f) {
        float k_us = _kf_P / (_kf_P + r_us_dyn);
        _kf_x += k_us * (us_m - _kf_x);
        _kf_P *= (1.0f - k_us);
    }

    fused_msg.header.stamp.sec     = sec;
    fused_msg.header.stamp.nanosec = nsec;
    fused_msg.range = _kf_x;

    /* Marca o instante em que x̂ cruza o limiar pela primeira vez */
    if (_kf_x <= OBSTACLE_M) {
        if (!_obs_measuring) {
            _obs_start_us  = time_us_64();
            _obs_measuring = true;
        }
    } else {
        _obs_measuring = false;
    }

    //

    teleop_lat_msg.data   = _teleop_latency_us;
    obstacle_lat_msg.data = _obstacle_latency_us;

    rcl_publish(&tf_pub,           &tf_msg,           NULL);
    rcl_publish(&odom_pub,         &odom_msg,         NULL);
    rcl_publish(&imu_pub,          &imu_msg,          NULL);
    rcl_publish(&mag_pub,          &mag_msg,          NULL);
    rcl_publish(&ir_pub,           &ir_msg,           NULL);
    rcl_publish(&us_pub,           &us_msg,           NULL);
    rcl_publish(&fused_pub,        &fused_msg,        NULL);
    rcl_publish(&teleop_lat_pub,   &teleop_lat_msg,   NULL);
    rcl_publish(&obstacle_lat_pub, &obstacle_lat_msg, NULL);

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

    MKPUB(odom_pub,         nav_msgs,    msg, Odometry,      "odom");
    MKPUB(tf_pub,           tf2_msgs,    msg, TFMessage,     "/tf");
    MKPUB(imu_pub,          sensor_msgs, msg, Imu,           "imu/data");
    MKPUB(mag_pub,          sensor_msgs, msg, MagneticField, "imu/mag");
    MKPUB(ir_pub,           sensor_msgs, msg, Range,         "ir_range");
    MKPUB(us_pub,           sensor_msgs, msg, Range,         "us_range");
    MKPUB(fused_pub,        sensor_msgs, msg, Range,         "fused_range");
    MKPUB(teleop_lat_pub,   std_msgs,    msg, UInt32,        "teleop_latency_us");
    MKPUB(obstacle_lat_pub, std_msgs,    msg, UInt32,        "obstacle_latency_us");
#undef MKPUB

    std_msgs__msg__UInt32__init(&teleop_lat_msg);
    std_msgs__msg__UInt32__init(&obstacle_lat_msg);

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

    sensor_msgs__msg__Range__init(&fused_msg);
    rosidl_runtime_c__String__assign(&fused_msg.header.frame_id, "base_link");
    fused_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    fused_msg.field_of_view  = 0.26f;
    fused_msg.min_range      = 0.02f;
    fused_msg.max_range      = 4.0f;

    /* /cmd_vel subscriber */
    rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

    /* Timer and executor */
    if (rclc_timer_init_default(
            &ros_timer, &support,
            RCL_MS_TO_NS(50), timer_callback) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    if (rclc_executor_init(
            &executor, &support.context, 2, &allocator) != RCL_RET_OK) {
        rcl_timer_fini(&ros_timer);
        rclc_support_fini(&support);
        return false;
    }
    rclc_executor_add_timer(&executor, &ros_timer);
    memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        cmd_vel_callback, ON_NEW_DATA);
    return true;
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

    /* SSC-32 servo controller */
    uart_init(SSC_UART, SSC_BAUD);
    gpio_set_function(SSC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SSC_RX_PIN, GPIO_FUNC_UART);
    ssc_send(REST_CMD);

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

    rmw_uros_sync_session(1000);
    create_entities();

    while (1) {
        pmw3901_service();
        gait_service();
        if (!i2c_dma_busy())
            mpu9250_read_async(&mpu9250, NULL, NULL);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
