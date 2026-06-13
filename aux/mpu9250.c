#include "mpu9250.h"
#include "i2c_dma.h"
#include <math.h>

static i2c_inst_t *i2c_port;

static uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = {reg, val};
    return (i2c_write_blocking(i2c_port, addr, buffer, 2, false) >= 0);
}

void i2c_activate(void) {
    i2c_dma_init(INTERFACE_I2C, PIN_SDA, PIN_SCL, BAUDRATE_I2C);
}

uint8_t mpu9250_init(i2c_inst_t *i2c) {
    i2c_port = i2c;
    sleep_ms(100);

    if (!i2c_write_reg(MPU6500_ADDR, REG_PWR_MGMT_1, 0x00))
        return 0;
    sleep_ms(10);

    i2c_write_reg(MPU6500_ADDR, REG_SMPLRT_DIV,  0x04); /* 200 Hz */
    i2c_write_reg(MPU6500_ADDR, REG_CONFIG,       0x03); /* DLPF 44 Hz */
    i2c_write_reg(MPU6500_ADDR, REG_GYRO_CONFIG,  0x08); /* ±500 dps */
    i2c_write_reg(MPU6500_ADDR, REG_ACCEL_CONFIG, 0x08); /* ±4 g */
    i2c_write_reg(MPU6500_ADDR, REG_INT_PIN_CFG,  0x02); /* bypass for AK8963 */
    sleep_ms(10);

    i2c_write_reg(AK8963_ADDR, AK8963_CNTL1, 0x00); /* power down */
    sleep_ms(10);
    i2c_write_reg(AK8963_ADDR, AK8963_CNTL1, 0x16); /* 16-bit, 100 Hz */
    sleep_ms(10);

    return 1;
}

/* ── Accel / Gyro async ── */
static mpu9250_t    *_ag_out;
static i2c_done_cb_t _ag_cb;
static void         *_ag_user;
static uint8_t       _ag_buf[14];

static void on_ag_dma_done(void *u) {
    (void)u;
    mpu9250_t *d = _ag_out;
    d->accel_raw[0] = (int16_t)((_ag_buf[0]  << 8) | _ag_buf[1]);
    d->accel_raw[1] = (int16_t)((_ag_buf[2]  << 8) | _ag_buf[3]);
    d->accel_raw[2] = (int16_t)((_ag_buf[4]  << 8) | _ag_buf[5]);
    /* bytes 6-7 = temperature, skip */
    d->gyro_raw[0]  = (int16_t)((_ag_buf[8]  << 8) | _ag_buf[9]);
    d->gyro_raw[1]  = (int16_t)((_ag_buf[10] << 8) | _ag_buf[11]);
    d->gyro_raw[2]  = (int16_t)((_ag_buf[12] << 8) | _ag_buf[13]);

    const float g = 9.80665f;
    for (int i = 0; i < 3; i++) {
        d->accel[i] = ((float)d->accel_raw[i] / ACCEL_SCALE_4G) * g - d->accel_bias[i];
        d->gyro[i]  = ((float)d->gyro_raw[i]  / GYRO_SCALE_500DPS) * (M_PI / 180.0f)
                      - d->gyro_bias[i];
    }

    if (_ag_cb) _ag_cb(_ag_user);
}

void mpu9250_read_accel_gyro_async(mpu9250_t *out, i2c_done_cb_t cb, void *user) {
    _ag_out  = out;
    _ag_cb   = cb;
    _ag_user = user;
    i2c_dma_read(MPU6500_ADDR, REG_ACCEL_XOUT_H, _ag_buf, 14, on_ag_dma_done, NULL);
}

/* ── Magnetometer async ── */
static mpu9250_t    *_mag_out;
static i2c_done_cb_t _mag_cb;
static void         *_mag_user;
static uint8_t       _mag_buf[7];

static void on_mag_dma_done(void *u) {
    (void)u;
    mpu9250_t *d = _mag_out;
    /* AK8963 is little-endian: L byte first, H byte second */
    d->mag_raw[0] = (int16_t)((_mag_buf[1] << 8) | _mag_buf[0]);
    d->mag_raw[1] = (int16_t)((_mag_buf[3] << 8) | _mag_buf[2]);
    d->mag_raw[2] = (int16_t)((_mag_buf[5] << 8) | _mag_buf[4]);

    for (int i = 0; i < 3; i++)
        d->mag[i] = (float)d->mag_raw[i] * MAG_SCALE - d->mag_bias[i];

    if (_mag_cb) _mag_cb(_mag_user);
}

void mpu9250_read_mag_async(mpu9250_t *out, i2c_done_cb_t cb, void *user) {
    _mag_out  = out;
    _mag_cb   = cb;
    _mag_user = user;
    i2c_dma_read(AK8963_ADDR, AK8963_HXL, _mag_buf, 7, on_mag_dma_done, NULL);
}
