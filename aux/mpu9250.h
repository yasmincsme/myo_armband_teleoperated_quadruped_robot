#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "i2c_dma.h"

#define MPU6500_ADDR 0x68
#define AK8963_ADDR  0x0C

#define INTERFACE_I2C i2c0
#define PIN_SDA       4
#define PIN_SCL       5
#define BAUDRATE_I2C  400000

#define ACCEL_SCALE_4G    8192.0f
#define GYRO_SCALE_500DPS 65.5f
#define MAG_SCALE         0.15f

#ifndef M_PI
#define M_PI 3.14159265f
#endif

/* MPU6500 registers */
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_INT_PIN_CFG  0x37

/* AK8963 registers */
#define AK8963_CNTL1 0x0A
#define AK8963_HXL   0x03

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    float   accel[3];
    float   gyro[3];
    float   mag[3];
    float   gyro_bias[3];
    float   accel_bias[3];
    float   mag_bias[3];
} mpu9250_t;

typedef void (*mpu9250_done_cb_t)(void);

/* Initialize I2C DMA engine and start non-blocking sensor config.
   on_done is called when the sensor is ready to read. */
void mpu9250_init(mpu9250_done_cb_t on_done);

/* Call in main loop until mpu9250_init_done() returns true. */
void mpu9250_init_poll(void);
bool mpu9250_init_done(void);

/* Start an async accel+gyro read (14 bytes from MPU6500).
   on_done fires from I2C IRQ when data is ready.
   Call mpu9250_get_accel_gyro() inside on_done to trigger the mag read. */
void mpu9250_read_accel_gyro_async(mpu9250_t *out, mpu9250_done_cb_t on_done);

/* Start an async magnetometer read (7 bytes from AK8963).
   Typically chained after the accel/gyro callback. */
void mpu9250_read_mag_async(mpu9250_t *out, mpu9250_done_cb_t on_done);

/* IRQ handler for I2C0 – call from I2C0_IRQ handler */
void mpu9250_i2c_irq_handler(void);

#endif
