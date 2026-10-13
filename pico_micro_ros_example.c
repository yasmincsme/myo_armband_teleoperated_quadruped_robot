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

#include "pico_uart_transports.h"
#include "hardware/uart.h"
#include "aux/pmw3901.h"
#include "aux/mpu9250.h"
#include "aux/i2c_dma.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN  25
#define DT       0.05f

/* ── TESTE ESTRATÉGICO: ativação de sensores por partes ───────────
   Mude cada flag para 1 para religar aquele sensor/periférico. Tudo
   começa em 0 (só a comunicação com o host roda). Religue um de cada
   vez, recompile, grave, confirme estabilidade, e só então ligue o
   próximo. */
#ifndef ENABLE_PMW3901
#define ENABLE_PMW3901 1
#endif
#ifndef ENABLE_MPU9250
#define ENABLE_MPU9250 0   /* sensor queimado */
#endif
#ifndef ENABLE_SHARPIR
#define ENABLE_SHARPIR 1
#endif
#ifndef ENABLE_HCSR04
#define ENABLE_HCSR04 1
#endif
#ifndef ENABLE_SSC32
#define ENABLE_SSC32 1
#endif

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
static rcl_subscription_t        cmd_vel_sub;

static nav_msgs__msg__Odometry         odom_msg;
static tf2_msgs__msg__TFMessage        tf_msg;
static sensor_msgs__msg__Imu           imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static sensor_msgs__msg__Range         ir_msg, us_msg, fused_msg;
static geometry_msgs__msg__Twist       cmd_vel_msg;

/* ── Kalman filter state for IR + sonar fusion ────────────────── */
#define KF_Q          0.001f  /* process noise variance (m²)                  */
#define KF_R_IR_NEAR  0.003f  /* IR noise at close range  → more trusted      */
#define KF_R_IR_FAR   0.020f  /* IR noise at far range    → less trusted      */
#define KF_R_US_NEAR  0.020f  /* sonar noise at close range → less trusted    */
#define KF_R_US_FAR   0.003f  /* sonar noise at far range   → more trusted    */
static float _kf_x = 0.5f;   /* state estimate (m)                            */
static float _kf_P = 1.0f;   /* error covariance                              */

#if ENABLE_SSC32
/* ── SSC-32 servo controller (UART1 / GPIO 8=TX, 9=RX) ─────────
   Leg channel map: coxa / femur / tibia
     A = 26 / 27 / 28     B = 21 / 20 / 19
     C = 10 / 11 / 12     D =  5 /  4 /  3                        */
#define SSC_UART      uart1
#define SSC_TX_PIN    8
#define SSC_RX_PIN    9
#define SSC_BAUD      115200
/* Obstacle gate. Two thresholds instead of one: stopping at 0.15 m but only
   resuming above 0.20 m keeps sensor noise at the boundary from toggling the
   robot between walking and parked. */
#define OBSTACLE_STOP_M   0.15f
#define OBSTACLE_CLEAR_M  0.20f
#define GAIT_QUERY_US 20000u

static const char REST_CMD[] =
    "#28 P1200 #27 P1300 #26 P1200 "
    "#21 P2000 #20 P1300 #19 P1250 "
    "#10 P1800 #11 P1700 #12 P1900 "
    "#5 P1000 #4 P1600 #3 P1700 T100";

#define GAIT_FWD_N   10
#define GAIT_LEFT_N  10
#define GAIT_RIGHT_N 10

/* Leg order D, C, body shift, B, A, body shift */
static const char *GAIT_FWD[GAIT_FWD_N] = {
    "#28 P1200 #27 P1300 #26 P1200 #21 P2000 #20 P1300 #19 P1250 #10 P1500 #11 P1700 #12 P1900 #5 P800 #4 P1600 #3 P1700 T100",
    "#5 P1500 #4 P1800 #3 P1700 T100",
    "#5 P1500 #4 P1600 #3 P1700 T100",
    "#10 P2000 #11 P1900 #12 P1900 T100",
    "#10 P2000 #11 P1700 #12 P1800 T100",
    "#26 P1500 #27 P1300 #28 P1200 #21 P2100 #20 P1300 #19 P1250 #10 P1800 #11 P1700 #12 P1900 #5 P1000 #4 P1600 #3 P1700 T100",
    "#21 P1500 #20 P1100 #19 P1250 T100",
    "#21 P1500 #20 P1300 #19 P1250 T100",
    "#26 P1000 #27 P1100 #28 P1200 T100",
    "#26 P1000 #27 P1300 #28 P1200 T100",
};

/* Turn-in-place: same leg order as the creep gait, but every coxa swings the
   same way on the servo (mirrored sides => the body rotates). The PC code's
   TURN_AMP=250 offset is pre-applied here: +250 for left, -250 for right. */
static const char *GAIT_LEFT[GAIT_LEFT_N] = {
    "#5 P1250 #4 P1800 #3 P1700 T100",
    "#5 P1250 #4 P1600 #3 P1700 T100",
    "#10 P2050 #11 P1900 #12 P1900 T100",
    "#10 P2050 #11 P1700 #12 P1900 T100",
    "#26 P950 #27 P1300 #28 P1200 #21 P1750 #20 P1300 #19 P1250 #10 P1800 #11 P1700 #12 P1900 #5 P1000 #4 P1600 #3 P1700 T100",
    "#21 P2250 #20 P1100 #19 P1250 T100",
    "#21 P2250 #20 P1300 #19 P1250 T100",
    "#26 P1450 #27 P1100 #28 P1200 T100",
    "#26 P1450 #27 P1300 #28 P1200 T100",
    "#26 P1200 #27 P1300 #28 P1200 #21 P2000 #20 P1300 #19 P1250 #10 P1550 #11 P1700 #12 P1900 #5 P750 #4 P1600 #3 P1700 T100",
};

static const char *GAIT_RIGHT[GAIT_RIGHT_N] = {
    "#5 P750 #4 P1800 #3 P1700 T100",
    "#5 P750 #4 P1600 #3 P1700 T100",
    "#10 P1550 #11 P1900 #12 P1900 T100",
    "#10 P1550 #11 P1700 #12 P1900 T100",
    "#26 P1450 #27 P1300 #28 P1200 #21 P2250 #20 P1300 #19 P1250 #10 P1800 #11 P1700 #12 P1900 #5 P1000 #4 P1600 #3 P1700 T100",
    "#21 P1750 #20 P1100 #19 P1250 T100",
    "#21 P1750 #20 P1300 #19 P1250 T100",
    "#26 P950 #27 P1100 #28 P1200 T100",
    "#26 P950 #27 P1300 #28 P1200 T100",
    "#26 P1200 #27 P1300 #28 P1200 #21 P2000 #20 P1300 #19 P1250 #10 P2050 #11 P1700 #12 P1900 #5 P1250 #4 P1600 #3 P1700 T100",
};

typedef enum { MOTION_NONE, MOTION_FORWARD, MOTION_LEFT, MOTION_RIGHT } motion_mode_t;
typedef enum { GAIT_IDLE, GAIT_MOVING, GAIT_QUERYING } gait_fsm_t;

static volatile motion_mode_t _motion    = MOTION_NONE;  /* operator intent */
static gait_fsm_t             _gait_fsm  = GAIT_IDLE;
static int                    _gait_step = 0;
static uint64_t               _last_q_us = 0;
static bool                   _blocked   = false;        /* obstacle gate */

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
    /* Re-evaluated every iteration, not just when a step finishes, so an
       obstacle appearing mid-stride stops the robot at the next step boundary
       rather than after the whole cycle. */
    if (!_blocked && _kf_x <= OBSTACLE_STOP_M)
        _blocked = true;
    else if (_blocked && _kf_x >= OBSTACLE_CLEAR_M)
        _blocked = false;

    if (_blocked) {
        /* Park and stay parked regardless of what was commanded. _motion is
           deliberately preserved: it is the operator's intent, and clearing it
           here is what previously let the next /cmd_vel restart the gait
           without ever consulting the obstacle gate. */
        if (_gait_fsm != GAIT_IDLE) {
            ssc_send(REST_CMD);
            _gait_fsm  = GAIT_IDLE;
            _gait_step = 0;
        }
        return;
    }

    /* Path clear: start, or resume, whatever motion currently stands. This is
       the only place a gait is started, so the obstacle gate above cannot be
       bypassed. */
    if (_gait_fsm == GAIT_IDLE) {
        if (_motion == MOTION_NONE) return;
        ssc_step_send();
        _last_q_us = time_us_64();
        _gait_fsm  = GAIT_MOVING;
        return;
    }

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
        _gait_step++;
        ssc_step_send();
        _last_q_us = time_us_64();
        _gait_fsm  = GAIT_MOVING;
    }
    /* other chars (echoes, CR/LF): ignore */
}
#endif /* ENABLE_SSC32 */

static void cmd_vel_callback(const void *msgin) {
#if ENABLE_SSC32
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

    _motion    = m;
    _gait_step = 0;

    if (m == MOTION_NONE) {
        ssc_send(REST_CMD);
        _gait_fsm = GAIT_IDLE;
    }
    /* Starting a gait is left entirely to gait_service(), which owns the
       obstacle gate. Commanding the servos from here is exactly how an
       incoming /cmd_vel used to walk the robot into an obstacle. */
#else
    /* SSC-32 desligada: apenas recebe a mensagem, sem tocar na UART. */
    (void)msgin;
#endif
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    /* Sensores desligados publicam valores neutros/fora-de-alcance, para o
       Kalman simplesmente ignorar aquela fonte em vez de tratar 0 como uma
       leitura válida. */
    float dx_m = 0.0f, dy_m = 0.0f, vel_x = 0.0f, vel_y = 0.0f;
#if ENABLE_PMW3901
    pmw3901_snapshot(&dx_m, &dy_m, &vel_x, &vel_y);
#endif

    float distance_m = 100.0f;
#if ENABLE_SHARPIR
    uint16_t raw;
    float voltage;
    float distance_cm = sharpIR_read(&raw, &voltage);
    distance_m = distance_cm / 100.0f;
#endif

    //

#if ENABLE_MPU9250
    float gz = mpu9250.gyro[2];
#else
    float gz = 0.0f;
#endif
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

#if ENABLE_MPU9250
    imu_msg.linear_acceleration.x = mpu9250.accel[0];
    imu_msg.linear_acceleration.y = mpu9250.accel[1];
    imu_msg.linear_acceleration.z = mpu9250.accel[2];

    imu_msg.angular_velocity.x = mpu9250.gyro[0];
    imu_msg.angular_velocity.y = mpu9250.gyro[1];
    imu_msg.angular_velocity.z = mpu9250.gyro[2];
#else
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = gz;
#endif

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

#if ENABLE_MPU9250
    mag_msg.magnetic_field.x = mpu9250.mag[0];
    mag_msg.magnetic_field.y = mpu9250.mag[1];
    mag_msg.magnetic_field.z = mpu9250.mag[2];
#else
    mag_msg.magnetic_field.x = 0.0;
    mag_msg.magnetic_field.y = 0.0;
    mag_msg.magnetic_field.z = 0.0;
#endif

    //Range

#if ENABLE_HCSR04
    float us_m = hcsr04.distance / 100.0f;
#else
    float us_m = 100.0f; /* fora de alcance → Kalman ignora */
#endif
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

    //

    rcl_publish(&tf_pub,    &tf_msg,    NULL);
    rcl_publish(&odom_pub,  &odom_msg,  NULL);
    rcl_publish(&imu_pub,   &imu_msg,   NULL);
    rcl_publish(&mag_pub,   &mag_msg,   NULL);
    rcl_publish(&ir_pub,    &ir_msg,    NULL);
    rcl_publish(&us_pub,    &us_msg,    NULL);
    rcl_publish(&fused_pub, &fused_msg, NULL);

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
    MKPUB(ir_pub,    sensor_msgs, msg, Range, "ir_range");
    MKPUB(us_pub,    sensor_msgs, msg, Range, "us_range");
    MKPUB(fused_pub, sensor_msgs, msg, Range, "fused_range");
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

#if ENABLE_PMW3901
/* ── PMW3901 bringup callback ─────────────────────────────────── */
static volatile bool _pmw_failed = false;

static void on_pmw_done(bool ok) {
    if (!ok) {
        _pmw_failed = true;
        return;
    }
    gpio_put(LED_PIN, 1);  /* solid ON: sensor ready */
}
#endif /* ENABLE_PMW3901 */

/* ── Main ─────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#if ENABLE_PMW3901
    /* PMW3901 async bringup (starts SPI + DMA, non-blocking) */
    pmw3901_init(on_pmw_done);
#endif

#if ENABLE_MPU9250
    i2c_activate();
    i2c_dma_init(INTERFACE_I2C);
    mpu9250_init(INTERFACE_I2C);
#endif
#if ENABLE_SHARPIR
    sharpIR_init();
#endif
#if ENABLE_HCSR04
    hcsr04_init(&hcsr04);
#endif

#if ENABLE_SSC32
    /* SSC-32 servo controller */
    uart_init(SSC_UART, SSC_BAUD);
    gpio_set_function(SSC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SSC_RX_PIN, GPIO_FUNC_UART);
    ssc_send(REST_CMD);
#endif

#if ENABLE_PMW3901
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
#endif

    /* ── Rotina de comunicação com o host ROS2 (NÃO comentada) ──── */
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    create_entities();

    /* Must come AFTER create_entities(): the time sync needs the XRCE session
       that rclc_support_init() establishes. Called before it, the sync fails
       and rmw_uros_epoch_nanos() returns time since boot, which makes RViz
       drop every message as older than its transform cache. */
    rmw_uros_sync_session(1000);

    while (1) {
#if ENABLE_PMW3901
        pmw3901_service();
#endif
#if ENABLE_SSC32
        gait_service();
#endif
#if ENABLE_MPU9250
        /* 14 bytes at 100 kHz take ~1.4 ms; 5 ms means the slave never ACKed
           and the completion IRQ will never arrive. Abort so the chain can be
           rearmed, instead of staying busy forever and freezing the readings. */
        if (i2c_dma_stalled(5000))
            i2c_dma_abort();
        if (!i2c_dma_busy())
            mpu9250_read_async(&mpu9250, NULL, NULL);
#endif
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
