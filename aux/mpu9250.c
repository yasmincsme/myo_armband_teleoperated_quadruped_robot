#include "mpu9250.h"
#include "i2c_dma.h"
#include <stdio.h>
#include <math.h>

i2c_inst_t *i2c_port;

void i2c_activate() {
    i2c_init(INTERFACE_I2C, BAUDRATE_I2C);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
}

uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = {reg, val};
    return (i2c_write_blocking(i2c_port, addr, buffer, 2, false) >= 0);
}

//O valor lido do barramento fica na variável buffer
//A função de escrita situa o ponteiro interno 
uint8_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (i2c_write_blocking(i2c_port, addr, &reg, 1, true) < 0)
        return 0;

    if (i2c_read_blocking(i2c_port, addr, buf, len, false) < 0)
        return 0;

    return 1;
}


uint8_t mpu9250_init(i2c_inst_t *i2c) {
    i2c_port = i2c; //salva ponteiro i2c
    sleep_ms(100);

    if(!i2c_write_reg(MPU6500_ADDR, REG_PWR_MGMT_1, 0X00)) { //Tira o sensor do modo sleep
        return 0;
    }
    sleep_ms(10);

    i2c_write_reg(MPU6500_ADDR, REG_SMPLRT_DIV, 0x04); //200HZ
    i2c_write_reg(MPU6500_ADDR, REG_CONFIG, 0x03); //DLPF 44HZ
    i2c_write_reg(MPU6500_ADDR, REG_GYRO_CONFIG, 0x08); //+-500 DPS
    i2c_write_reg(MPU6500_ADDR, REG_ACCEL_CONFIG, 0X08); //+-4g

    //activate bypass for magnometer
    i2c_write_reg(MPU6500_ADDR, REG_INT_PIN_CFG, 0x02); //Isso permite acessar o AK8963 diretamente pelo I2C externo.
    sleep_ms(10);

    // power down
    i2c_write_reg(AK8963_ADDR, AK8963_CNTL1, 0x00);
    sleep_ms(10);

    // 16bit + continuous mode 2 (100Hz)
    i2c_write_reg(AK8963_ADDR, AK8963_CNTL1, 0x16);
    sleep_ms(10);

    return 1;
}

uint8_t mpu9250_update_values(mpu9250_t *data) {
    uint8_t buf[14];

    if (!i2c_read_bytes(MPU6500_ADDR, REG_ACCEL_XOUT_H, buf, 14))
        return 0;

    data->accel_raw[0] = (buf[0] << 8) | buf[1];
    data->accel_raw[1] = (buf[2] << 8) | buf[3];
    data->accel_raw[2] = (buf[4] << 8) | buf[5];

    data->gyro_raw[0]  = (buf[8] << 8) | buf[9];
    data->gyro_raw[1]  = (buf[10] << 8) | buf[11];
    data->gyro_raw[2]  = (buf[12] << 8) | buf[13];

    const float g = 9.80665f;

    for (int i = 0; i < 3; i++)
    {
        // Acelerômetro (±4g configurado no init)
        data->accel[i] =
            ((float)data->accel_raw[i] / ACCEL_SCALE_4G) * g
            - data->accel_bias[i];

        // Giroscópio (±500 dps configurado no init)
        data->gyro[i] =
            ((float)data->gyro_raw[i] / GYRO_SCALE_500DPS)
            * (M_PI / 180.0f)
            - data->gyro_bias[i];
    }

    uint8_t mag[7];

    if (!i2c_read_bytes(AK8963_ADDR, AK8963_HXL, mag, 7))
        return 0;

    data->mag_raw[0] = (mag[1] << 8) | mag[0];
    data->mag_raw[1] = (mag[3] << 8) | mag[2];
    data->mag_raw[2] = (mag[5] << 8) | mag[4];

    for (int i = 0; i < 3; i++)
    {
        data->mag[i] =
            (float)data->mag_raw[i] * MAG_SCALE
            - data->mag_bias[i];
    }

    return 1;
}

/* ── Async read chain (DMA_IRQ_1 context) ─────────────────────── */

static uint8_t           _ag_buf[14];
static uint8_t           _mag_buf[7];
static mpu9250_t        *_async_out;
static i2c_dma_done_cb_t _async_cb;
static void             *_async_user;

static void on_mag_done(void *u) {
    (void)u;
    mpu9250_t *d = _async_out;

    d->mag_raw[0] = (int16_t)((_mag_buf[1] << 8) | _mag_buf[0]);
    d->mag_raw[1] = (int16_t)((_mag_buf[3] << 8) | _mag_buf[2]);
    d->mag_raw[2] = (int16_t)((_mag_buf[5] << 8) | _mag_buf[4]);

    for (int i = 0; i < 3; i++)
        d->mag[i] = (float)d->mag_raw[i] * MAG_SCALE - d->mag_bias[i];

    if (_async_cb) _async_cb(_async_user);
}

static void on_ag_done(void *u) {
    (void)u;
    mpu9250_t *d = _async_out;

    d->accel_raw[0] = (int16_t)((_ag_buf[0]  << 8) | _ag_buf[1]);
    d->accel_raw[1] = (int16_t)((_ag_buf[2]  << 8) | _ag_buf[3]);
    d->accel_raw[2] = (int16_t)((_ag_buf[4]  << 8) | _ag_buf[5]);
    d->gyro_raw[0]  = (int16_t)((_ag_buf[8]  << 8) | _ag_buf[9]);
    d->gyro_raw[1]  = (int16_t)((_ag_buf[10] << 8) | _ag_buf[11]);
    d->gyro_raw[2]  = (int16_t)((_ag_buf[12] << 8) | _ag_buf[13]);

    const float g = 9.80665f;
    for (int i = 0; i < 3; i++) {
        d->accel[i] = ((float)d->accel_raw[i] / ACCEL_SCALE_4G) * g
                      - d->accel_bias[i];
        d->gyro[i]  = ((float)d->gyro_raw[i] / GYRO_SCALE_500DPS)
                      * (M_PI / 180.0f) - d->gyro_bias[i];
    }

    /* Wait for STOP before issuing the next transaction. */
    i2c_dma_wait_idle();
    i2c_dma_read(AK8963_ADDR, AK8963_HXL, _mag_buf, 7, on_mag_done, NULL);
}

void mpu9250_read_async(mpu9250_t *out, i2c_dma_done_cb_t cb, void *user) {
    _async_out  = out;
    _async_cb   = cb;
    _async_user = user;
    i2c_dma_read(MPU6500_ADDR, REG_ACCEL_XOUT_H, _ag_buf, 14, on_ag_done, NULL);
}

