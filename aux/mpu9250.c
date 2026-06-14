#include "mpu9250.h"
#include "pico/time.h"
#include <math.h>

static i2c_dma_t _i2c;

/* ── Non-blocking init state machine ──────────────────────────────────────── */
/* Odd  stages: waiting for alarm
   Even stages: alarm fired, poll should act
     1 → wait 100ms after wakeup
     2 → write config, schedule 10ms → 3
     3 → wait 10ms
     4 → power down mag, schedule 10ms → 5
     5 → wait 10ms
     6 → continuous mode, schedule 10ms → 7
     7 → wait 10ms
     8 → done                                                                 */

static volatile uint8_t  _stage   = 0;
static mpu9250_done_cb_t _done_cb = NULL;

static int64_t _alarm_cb(alarm_id_t id, void *user_data) {
    (void)id; (void)user_data;
    _stage++;
    return 0;
}

#define WREG(addr, reg, val) i2c_dma_write_reg_blocking(&_i2c, addr, reg, val)

void mpu9250_init(mpu9250_done_cb_t on_done) {
    _done_cb = on_done;
    i2c_dma_init(&_i2c, INTERFACE_I2C, BAUDRATE_I2C, PIN_SDA, PIN_SCL);

    irq_set_exclusive_handler(I2C0_IRQ, mpu9250_i2c_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    WREG(MPU6500_ADDR, REG_PWR_MGMT_1, 0x00);
    _stage = 1;
    add_alarm_in_ms(100, _alarm_cb, NULL, false);
}

void mpu9250_init_poll(void) {
    switch (_stage) {
        case 2:
            WREG(MPU6500_ADDR, REG_SMPLRT_DIV,   0x04);
            WREG(MPU6500_ADDR, REG_CONFIG,        0x03);
            WREG(MPU6500_ADDR, REG_GYRO_CONFIG,   0x08);
            WREG(MPU6500_ADDR, REG_ACCEL_CONFIG,  0x08);
            WREG(MPU6500_ADDR, REG_INT_PIN_CFG,   0x02);
            _stage = 3;
            add_alarm_in_ms(10, _alarm_cb, NULL, false);
            break;
        case 4:
            WREG(AK8963_ADDR, AK8963_CNTL1, 0x00);
            _stage = 5;
            add_alarm_in_ms(10, _alarm_cb, NULL, false);
            break;
        case 6:
            WREG(AK8963_ADDR, AK8963_CNTL1, 0x16);
            _stage = 7;
            add_alarm_in_ms(10, _alarm_cb, NULL, false);
            break;
        case 8:
            _stage = 9;
            if (_done_cb) _done_cb();
            break;
        default:
            break;
    }
}

bool mpu9250_init_done(void) {
    return _stage == 9;
}

void mpu9250_i2c_irq_handler(void) {
    i2c_dma_irq_handler(&_i2c);
}

/* ── Async accel + gyro read ──────────────────────────────────────────────── */

static mpu9250_t        *_out_ptr    = NULL;
static mpu9250_done_cb_t _ag_done_cb = NULL;
static uint8_t           _ag_buf[14];

static void _ag_raw_done(void *user_data, bool ok) {
    (void)user_data;
    if (!ok || !_out_ptr) return;

    mpu9250_t *d = _out_ptr;
    d->accel_raw[0] = (int16_t)((_ag_buf[0]  << 8) | _ag_buf[1]);
    d->accel_raw[1] = (int16_t)((_ag_buf[2]  << 8) | _ag_buf[3]);
    d->accel_raw[2] = (int16_t)((_ag_buf[4]  << 8) | _ag_buf[5]);
    d->gyro_raw[0]  = (int16_t)((_ag_buf[8]  << 8) | _ag_buf[9]);
    d->gyro_raw[1]  = (int16_t)((_ag_buf[10] << 8) | _ag_buf[11]);
    d->gyro_raw[2]  = (int16_t)((_ag_buf[12] << 8) | _ag_buf[13]);

    const float g = 9.80665f;
    for (int i = 0; i < 3; i++) {
        d->accel[i] = ((float)d->accel_raw[i] / ACCEL_SCALE_4G) * g - d->accel_bias[i];
        d->gyro[i]  = ((float)d->gyro_raw[i]  / GYRO_SCALE_500DPS) * (M_PI / 180.0f)
                      - d->gyro_bias[i];
    }

    if (_ag_done_cb) _ag_done_cb();
}

void mpu9250_read_accel_gyro_async(mpu9250_t *out, mpu9250_done_cb_t on_done) {
    _out_ptr    = out;
    _ag_done_cb = on_done;
    i2c_dma_write_read(&_i2c, MPU6500_ADDR, REG_ACCEL_XOUT_H,
                       _ag_buf, 14, _ag_raw_done, NULL);
}

/* ── Async magnetometer read ──────────────────────────────────────────────── */

static mpu9250_done_cb_t _mag_done_cb = NULL;
static uint8_t           _mag_buf[7];

static void _mag_raw_done(void *user_data, bool ok) {
    (void)user_data;
    if (!ok || !_out_ptr) return;

    mpu9250_t *d = _out_ptr;
    d->mag_raw[0] = (int16_t)((_mag_buf[1] << 8) | _mag_buf[0]);
    d->mag_raw[1] = (int16_t)((_mag_buf[3] << 8) | _mag_buf[2]);
    d->mag_raw[2] = (int16_t)((_mag_buf[5] << 8) | _mag_buf[4]);

    for (int i = 0; i < 3; i++)
        d->mag[i] = (float)d->mag_raw[i] * MAG_SCALE - d->mag_bias[i];

    if (_mag_done_cb) _mag_done_cb();
}

void mpu9250_read_mag_async(mpu9250_t *out, mpu9250_done_cb_t on_done) {
    _out_ptr     = out;
    _mag_done_cb = on_done;
    i2c_dma_write_read(&_i2c, AK8963_ADDR, AK8963_HXL,
                       _mag_buf, 7, _mag_raw_done, NULL);
}
