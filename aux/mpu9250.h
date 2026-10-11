#ifndef MPU9250_H
#define MPU9250_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_dma.h"

extern i2c_inst_t *i2c_port;

#define MPU6500_ADDR 0x68
#define AK8963_ADDR  0x0C

#define INTERFACE_I2C i2c0 
#define PIN_SDA       4
#define PIN_SCL       5
#define BAUDRATE_I2C  100000

#define ACCEL_SCALE_2G   16384.0f
#define ACCEL_SCALE_4G   8192.0f
#define ACCEL_SCALE_8G   4096.0f
#define ACCEL_SCALE_16G  2048.0f

#define GYRO_SCALE_250DPS   131.0f
#define GYRO_SCALE_500DPS   65.5f
#define GYRO_SCALE_1000DPS  32.8f
#define GYRO_SCALE_2000DPS  16.4f

#define MAG_SCALE    0.15f
#define M_PI         3.14159265f

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_INT_PIN_CFG     0x37
#define REG_WHO_AM_I        0x75

#define AK8963_WHO_AM_I     0x00
#define AK8963_CNTL1        0x0A
#define AK8963_ST1          0x02
#define AK8963_HXL          0x03
#define AK8963_ST2          0x09

typedef struct {

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];

    float accel[3];   // m/s²
    float gyro[3];    // rad/s
    float mag[3];     // Tesla

    float gyro_bias[3];
    float accel_bias[3];
    float mag_bias[3];

} mpu9250_t;

void i2c_activate();
uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t mpu9250_init(i2c_inst_t *i2c);
uint8_t mpu9250_update_values(mpu9250_t *data);

/* Non-blocking read: accel/gyro → (chain) → mag. cb is called from DMA_IRQ_1. */
void mpu9250_read_async(mpu9250_t *out, i2c_dma_done_cb_t cb, void *user);

#endif