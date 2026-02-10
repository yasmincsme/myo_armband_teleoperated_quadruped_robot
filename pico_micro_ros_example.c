#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h> //comentar
#include <geometry_msgs/msg/vector3_stamped.h> //comentar
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h> 

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/spi.h"

const uint LED_PIN = 25;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

const double PMW_SCALE = 0.001; //calibrar depois de acordo com datasheet
const double DT = 0.05;

//Pinout for Pico 2
#define PIN_MISO 16 
#define PIN_MOSI 19
#define PIN_SCK 18
#define PIN_CS 17
#define INTERFACE_SPI spi0

//Register List
#define PRODUCT_ID          0x00
#define REVISION_ID         0x01
#define MOTION              0x02
#define DELTA_X_L           0x03
#define DELTA_X_H           0x04
#define DELTA_Y_L           0x05
#define DELTA_Y_H           0x06
#define SQUAL               0x07
#define RAWDATA_SUM         0x08
#define MAXIMUM_RAWDATA     0x09
#define MINIMUM_RAWDATA     0x0A
#define SHUTTER_LOWER       0x0B
#define SHUTTER_UPPER       0x0C
#define OBSERVATION         0x15
#define MOTION_BURST        0x16
#define POWER_UP_RESET      0x3A
#define SHUTDOWN            0x3B
#define RAWDATA_GRAB        0x58
#define RAWDATA_GRAB_STATUS 0x59
#define INVERSE_PRODUCT_ID  0x5F

//Reset Register List
#define PRODUCT_ID_RST          0x49
#define REVISION_ID_RST         0x00
#define MOTION_RST              0x00
#define DELTA_X_L_RST           0x00
#define DELTA_X_H_RST           0x00
#define DELTA_Y_L_RST           0x00
#define DELTA_Y_H_RST           0x00
#define SQUAL_RST               0x00
#define RAWDATA_SUM_RST         0x00
#define MAXIMUM_RAWDATA_RST     0x00
#define MINIMUM_RAWDATA_RST     0x00
#define SHUTTER_LOWER_RST       0x00
#define SHUTTER_UPPER_RST       0x00
#define OBSERVATION_RST         0x00
#define MOTION_BURST_RST        0x00
#define RAWDATA_GRAB_RST        0x00
#define RAWDATA_GRAB_STATUS_RST 0x00
#define INVERSE_PRODUCT_ID_RST  0xB6

static inline void cs_activate() {
    asm volatile("nop \n nop \n nop"); 
    gpio_put(PIN_CS, 0); 
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deactivate() {
    asm volatile("nop \n nop \n nop"); 
    gpio_put(PIN_CS, 1); 
    asm volatile("nop \n nop \n nop");
}

void init_spi() {
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    spi_init(spi0, 2000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
}

void pmw3901_write(uint8_t reg, uint8_t data) {
    uint8_t buffer[] = {reg | 0x80, data}; //Máscara para escrita 1000 000 
    
    cs_activate();
    spi_write_blocking(spi0, buffer, 2);
    cs_deactivate();
    sleep_us(50);
}

uint8_t pmw3901_read(uint8_t reg) {
    uint8_t addr = reg & 0x7F; //Máscara para leitura 0111 1111
    uint8_t value = 0;

    cs_activate();
    spi_write_blocking(spi0, &addr, 1);
    sleep_us(50);
    spi_read_blocking(spi0, 0, &value, 1);
    cs_deactivate();

    return value;
}

void pmw3901_init_registers() {
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x61, 0xAD);
    pmw3901_write(0x7F, 0x03);
    pmw3901_write(0x40, 0x00);
    pmw3901_write(0x7F, 0x05);
    pmw3901_write(0x41, 0xB3);
    pmw3901_write(0x43, 0xF1);
    pmw3901_write(0x45, 0x14);
    pmw3901_write(0x5B, 0x32);
    pmw3901_write(0x5F, 0x34);
    pmw3901_write(0x7B, 0x08);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x44, 0x1B);
    pmw3901_write(0x40, 0xBF);
    pmw3901_write(0x4E, 0x3F);
    pmw3901_write(0x7F, 0x08);
    pmw3901_write(0x65, 0x20);
    pmw3901_write(0x6A, 0x18);
    pmw3901_write(0x7F, 0x09);
    pmw3901_write(0x4F, 0xAF);
    pmw3901_write(0x5F, 0x40);
    pmw3901_write(0x48, 0x80);
    pmw3901_write(0x49, 0x80);
    pmw3901_write(0x57, 0x77);
    pmw3901_write(0x60, 0x78);
    pmw3901_write(0x61, 0x78);
    pmw3901_write(0x62, 0x08);
    pmw3901_write(0x63, 0x50);
    pmw3901_write(0x7F, 0x0A);
    pmw3901_write(0x45, 0x60);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x4D, 0x11);
    pmw3901_write(0x55, 0x80);
    pmw3901_write(0x74, 0x1F);
    pmw3901_write(0x75, 0x1F);
    pmw3901_write(0x4A, 0x78);
    pmw3901_write(0x4B, 0x78);
    pmw3901_write(0x44, 0x08);
    pmw3901_write(0x45, 0x50);
    pmw3901_write(0x64, 0xFF);
    pmw3901_write(0x65, 0x1F);
    pmw3901_write(0x7F, 0x14);
    pmw3901_write(0x65, 0x60);
    pmw3901_write(0x66, 0x08);
    pmw3901_write(0x63, 0x78);
    pmw3901_write(0x7F, 0x15);
    pmw3901_write(0x48, 0x58);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x41, 0x0D);
    pmw3901_write(0x43, 0x14);
    pmw3901_write(0x4B, 0x0E);
    pmw3901_write(0x45, 0x0F);
    pmw3901_write(0x44, 0x42);
    pmw3901_write(0x4C, 0x80);
    pmw3901_write(0x7F, 0x10);
    pmw3901_write(0x5B, 0x02);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x41);
    pmw3901_write(0x70, 0x00);

    sleep_ms(100);
    pmw3901_write(0x32, 0x44);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x40);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x62, 0xf0);
    pmw3901_write(0x63, 0x00);
    pmw3901_write(0x7F, 0x0D);
    pmw3901_write(0x48, 0xC0);
    pmw3901_write(0x6F, 0xd5);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x5B, 0xa0);
    pmw3901_write(0x4E, 0xA8);
    pmw3901_write(0x5A, 0x50);
    pmw3901_write(0x40, 0x80);
}

typedef struct {
    int16_t dx;
    int16_t dy;
} pmw3901_t;

pmw3901_t pmw3901_read_motion() {
    pmw3901_t pmw3901;
    uint8_t h, l;
    pmw3901_read(MOTION); //trava os valores de leitura

    h = pmw3901_read(DELTA_X_H);
    l = pmw3901_read(DELTA_X_L);
    pmw3901.dx = (int16_t) ((h << 8) | l);

    h = pmw3901_read(DELTA_Y_H);
    l = pmw3901_read(DELTA_Y_L);
    pmw3901.dy = (int16_t) ((h << 8) | l);

    return pmw3901;
}
//-----------------------------------------------------------

rcl_publisher_t publisher;
nav_msgs__msg__Odometry msg;

rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if(timer == NULL) return;

    pmw3901_t pmw3901 = pmw3901_read_motion();

    double dx = pmw3901.dx * PMW_SCALE;
    double dy = pmw3901.dy * PMW_SCALE;

    x += dx;
    y += dy;

    int64_t time_ns = rmw_uros_epoch_nanos();
    msg.header.stamp.sec = (int32_t) (time_ns / 1000000000);
    msg.header.stamp.nanosec = (uint32_t) (time_ns % 1000000000);

    msg.pose.pose.position.x = x; //Posição
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0; 

    msg.pose.pose.orientation.w = 1.0; //Quaternion
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;

    msg.twist.twist.linear.x = dx / DT;
    msg.twist.twist.linear.y = dy / DT;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.z = 0.0;

    tf.header.stamp = msg.header.stamp;

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.w = 1.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;

    rcl_publish(&tf_publisher, &tf_msg, NULL);
    rcl_publish(&publisher, &msg, NULL);

    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    
}

int main() {
    stdio_init_all();

    init_spi();
    pmw3901_init_registers();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    if(rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        return 1;
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_pmw3901", "", &support);
    
    rclc_publisher_init_default(
        &publisher,
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

    rosidl_runtime_c__String__assign(&msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&msg.child_frame_id, "base_link");

    rosidl_runtime_c__String__assign(&tf.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&tf.child_frame_id, "base_link");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

}
