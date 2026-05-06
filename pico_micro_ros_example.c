#include <stdio.h>
//#include "pico/stdlib.h"
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
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>

#include "pico_uart_transports.h"
#include "aux/pmw3901.h"
#include "aux/mpu9250.h"
#include "aux/sharpIR.h"
#include "aux/hcsr04.h"

#define LED_PIN 25

pmw3901_t pmw3901;
mpu9250_t mpu9250;
hcsr04_t hcsr04;

float x = 0.0;
float y = 0.0;
float theta = 0.0;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_msg;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t mag_publisher;
sensor_msgs__msg__MagneticField mag_msg;

rcl_publisher_t ir_publisher;
sensor_msgs__msg__Range ir_msg;

rcl_publisher_t us_publisher;
sensor_msgs__msg__Range us_msg;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if(timer == NULL) return;

    pmw3901_update_values(&pmw3901);
    mpu9250_update_values(&mpu9250);

    uint16_t raw;
    float voltage;
    float distance_cm = sharpIR_read(&raw, &voltage);
    float distance_m = distance_cm / 100.0f;

    //

    float gz = mpu9250.gyro[2];
    theta += gz * DT;

    float c = cos(theta);
    float s = sin(theta);

    x += pmw3901.dx_m * c - pmw3901.dy_m * s;
    y += pmw3901.dx_m * s + pmw3901.dy_m * c;

    float half = theta * 0.5;
    float qw = cos(half);
    float qz = sin(half);

    //

    int64_t time_ns = rmw_uros_epoch_nanos();
    int32_t sec = (int32_t)(time_ns / 1000000000);
    uint32_t nsec = (uint32_t)(time_ns % 1000000000);

    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nsec;

    // Timestamp atual
    tf_msg.transforms.data[0].header.stamp.sec = sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = nsec;

    tf_msg.transforms.data[1].header.stamp.sec = sec;
    tf_msg.transforms.data[1].header.stamp.nanosec = nsec;

    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nsec;

    mag_msg.header.stamp.sec = sec;
    mag_msg.header.stamp.nanosec = nsec;

    ir_msg.header.stamp.sec = sec;
    ir_msg.header.stamp.nanosec = nsec;

    //Odom

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.w = qw;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = qz;

    odom_msg.twist.twist.linear.x = pmw3901.dx_m / DT;
    odom_msg.twist.twist.linear.y = pmw3901.dy_m / DT;
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

    tf_msg.transforms.data[1].transform.translation.x = 0.10f;
    tf_msg.transforms.data[1].transform.translation.y = 0.0f;
    tf_msg.transforms.data[1].transform.translation.z = 0.05f;

    // tf_msg.transforms.data[1].transform.rotation.w = 1.0f;
    // tf_msg.transforms.data[1].transform.rotation.x = 0.0f;
    // tf_msg.transforms.data[1].transform.rotation.y = 0.0f;
    // tf_msg.transforms.data[1].transform.rotation.z = 0.0f;

    float yaw_ir = 0.0f; // ajuste aqui
    float half_ir = yaw_ir * 0.5f;

    tf_msg.transforms.data[1].transform.rotation.w = cosf(half_ir);
    tf_msg.transforms.data[1].transform.rotation.x = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.y = 0.0f;
    tf_msg.transforms.data[1].transform.rotation.z = sinf(half_ir);

    tf_msg.transforms.data[2].transform.translation.x = 0.10f;
    tf_msg.transforms.data[2].transform.translation.y = 0.0f;
    tf_msg.transforms.data[2].transform.translation.z = 0.05f;

    tf_msg.transforms.data[2].transform.rotation.w = 1.0f;
    tf_msg.transforms.data[2].transform.rotation.x = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.y = 0.0f;
    tf_msg.transforms.data[2].transform.rotation.z = 0.0f;

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
    us_msg.range = hcsr04.distance / 100.0f; // se estiver em cm  

    //

    rcl_publish(&tf_publisher, &tf_msg, NULL);
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&imu_publisher, &imu_msg, NULL);
    rcl_publish(&mag_publisher, &mag_msg, NULL);
    rcl_publish(&ir_publisher, &ir_msg, NULL);
    rcl_publish(&us_publisher, &us_msg, NULL);

    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

int main() {
    stdio_init_all();

    init_spi();
    i2c_activate();
    mpu9250_init(INTERFACE_I2C);
    pmw3901_init_registers();
    sharpIR_init();
    hcsr04_init(&hcsr04);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    if(rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        return 1;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_timer_t timer;
    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_pmw3901", "", &support);
    
    //Publishers
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    );

    rclc_publisher_init_default(
        &tf_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf"
    );

    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );

    rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    );

    rclc_publisher_init_default(
        &ir_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ir_range"
    );

    rclc_publisher_init_default(
        &us_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "us_range"
    );

    nav_msgs__msg__Odometry__init(&odom_msg);
    tf2_msgs__msg__TFMessage__init(&tf_msg);   
    sensor_msgs__msg__Imu__init(&imu_msg);
    sensor_msgs__msg__MagneticField__init(&mag_msg);
    sensor_msgs__msg__Range__init(&ir_msg); 
    sensor_msgs__msg__Range__init(&us_msg); 

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
    rosidl_runtime_c__String__assign(&ir_msg.header.frame_id, "ir_link");
    rosidl_runtime_c__String__assign(&us_msg.header.frame_id, "us_link");

    // tf_msg.transforms.size = 2;
    // tf_msg.transforms.capacity = 2;
    tf_msg.transforms.size = 3;
    tf_msg.transforms.capacity = 3;
    tf_msg.transforms.data = malloc(sizeof(geometry_msgs__msg__TransformStamped) * 3);

    ir_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    ir_msg.field_of_view = 0.26;   // ~15 graus (ajuste se quiser)
    ir_msg.min_range = 0.10;     
    ir_msg.max_range = 0.80;  

    us_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    us_msg.field_of_view = 0.5;     // ~30 graus típico HC-SR04
    us_msg.min_range = 0.02;        // 2 cm
    us_msg.max_range = 4.0;         // 4 m

    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[0]);
    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[1]);
    geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[2]);

    tf_msg.transforms.data[0].header.frame_id.data = "odom";
    tf_msg.transforms.data[0].header.frame_id.size = 4;
    tf_msg.transforms.data[0].header.frame_id.capacity = 5;

    tf_msg.transforms.data[0].child_frame_id.data = "base_link";
    tf_msg.transforms.data[0].child_frame_id.size = 9;
    tf_msg.transforms.data[0].child_frame_id.capacity = 10;

    tf_msg.transforms.data[1].header.frame_id.data = "base_link";
    tf_msg.transforms.data[1].header.frame_id.size = 9;
    tf_msg.transforms.data[1].header.frame_id.capacity = 10;

    tf_msg.transforms.data[1].child_frame_id.data = "ir_link";
    tf_msg.transforms.data[1].child_frame_id.size = 7;
    tf_msg.transforms.data[1].child_frame_id.capacity = 8;

    tf_msg.transforms.data[2].header.frame_id.data = "base_link";
    tf_msg.transforms.data[2].header.frame_id.size = 9;
    tf_msg.transforms.data[2].header.frame_id.capacity = 10;

    tf_msg.transforms.data[2].child_frame_id.data = "us_link";
    tf_msg.transforms.data[2].child_frame_id.size = 7;
    tf_msg.transforms.data[2].child_frame_id.capacity = 8;

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
