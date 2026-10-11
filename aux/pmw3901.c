#include "pmw3901.h"
#include "spi_async.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/sync.h"

/* ── Register map ─────────────────────────────────────────────── */
#define REG_PRODUCT_ID     0x00
#define REG_INV_PRODUCT_ID 0x5F
#define REG_MOTION         0x02
#define REG_DELTA_X_L      0x03
#define REG_DELTA_X_H      0x04
#define REG_DELTA_Y_L      0x05
#define REG_DELTA_Y_H      0x06
#define REG_SQUAL          0x07
#define REG_SHUTTER_L      0x0B
#define REG_SHUTTER_U      0x0C
#define REG_POWER_UP_RESET 0x3A

#define PID_EXPECTED     0x49u
#define INV_PID_EXPECTED 0xB6u

/* ── Init table (RESTRIÇÃO 2) ─────────────────────────────────── */
typedef enum { I_WRITE, I_DELAY_MS } init_kind_t;
typedef struct {
    init_kind_t kind;
    uint8_t     reg;
    uint8_t     val;
    uint32_t    ms;
} init_item_t;

static const init_item_t INIT_TABLE[] = {
    {I_WRITE, 0x7F, 0x00, 0}, {I_WRITE, 0x61, 0xAD, 0},
    {I_WRITE, 0x7F, 0x03, 0}, {I_WRITE, 0x40, 0x00, 0},
    {I_WRITE, 0x7F, 0x05, 0}, {I_WRITE, 0x41, 0xB3, 0},
    {I_WRITE, 0x43, 0xF1, 0}, {I_WRITE, 0x45, 0x14, 0},
    {I_WRITE, 0x5B, 0x32, 0}, {I_WRITE, 0x5F, 0x34, 0},
    {I_WRITE, 0x7B, 0x08, 0}, {I_WRITE, 0x7F, 0x06, 0},
    {I_WRITE, 0x44, 0x1B, 0}, {I_WRITE, 0x40, 0xBF, 0},
    {I_WRITE, 0x4E, 0x3F, 0}, {I_WRITE, 0x7F, 0x08, 0},
    {I_WRITE, 0x65, 0x20, 0}, {I_WRITE, 0x6A, 0x18, 0},
    {I_WRITE, 0x7F, 0x09, 0}, {I_WRITE, 0x4F, 0xAF, 0},
    {I_WRITE, 0x5F, 0x40, 0}, {I_WRITE, 0x48, 0x80, 0},
    {I_WRITE, 0x49, 0x80, 0}, {I_WRITE, 0x57, 0x77, 0},
    {I_WRITE, 0x60, 0x78, 0}, {I_WRITE, 0x61, 0x78, 0},
    {I_WRITE, 0x62, 0x08, 0}, {I_WRITE, 0x63, 0x50, 0},
    {I_WRITE, 0x7F, 0x0A, 0}, {I_WRITE, 0x45, 0x60, 0},
    {I_WRITE, 0x7F, 0x00, 0}, {I_WRITE, 0x4D, 0x11, 0},
    {I_WRITE, 0x55, 0x80, 0}, {I_WRITE, 0x74, 0x21, 0},
    {I_WRITE, 0x75, 0x1F, 0}, {I_WRITE, 0x4A, 0x78, 0},
    {I_WRITE, 0x4B, 0x78, 0}, {I_WRITE, 0x44, 0x08, 0},
    {I_WRITE, 0x45, 0x50, 0}, {I_WRITE, 0x64, 0xFF, 0},
    {I_WRITE, 0x65, 0x1F, 0}, {I_WRITE, 0x7F, 0x14, 0},
    {I_WRITE, 0x65, 0x67, 0}, {I_WRITE, 0x66, 0x08, 0},
    {I_WRITE, 0x63, 0x70, 0}, {I_WRITE, 0x7F, 0x15, 0},
    {I_WRITE, 0x48, 0x48, 0}, {I_WRITE, 0x7F, 0x07, 0},
    {I_WRITE, 0x41, 0x0D, 0}, {I_WRITE, 0x43, 0x14, 0},
    {I_WRITE, 0x4B, 0x0E, 0}, {I_WRITE, 0x45, 0x0F, 0},
    {I_WRITE, 0x44, 0x42, 0}, {I_WRITE, 0x4C, 0x80, 0},
    {I_WRITE, 0x7F, 0x10, 0}, {I_WRITE, 0x5B, 0x02, 0},
    {I_WRITE, 0x7F, 0x07, 0}, {I_WRITE, 0x40, 0x41, 0},
    {I_WRITE, 0x70, 0x00, 0},
    {I_DELAY_MS, 0x00, 0x00, 100},
    {I_WRITE, 0x7F, 0x00, 0}, {I_WRITE, 0x32, 0x00, 0},
    {I_WRITE, 0x7F, 0x07, 0}, {I_WRITE, 0x40, 0x40, 0},
    {I_WRITE, 0x7F, 0x06, 0}, {I_WRITE, 0x68, 0xF0, 0},
    {I_WRITE, 0x69, 0x00, 0}, {I_WRITE, 0x7F, 0x0D, 0},
    {I_WRITE, 0x48, 0xC0, 0}, {I_WRITE, 0x6F, 0xD5, 0},
    {I_WRITE, 0x7F, 0x00, 0}, {I_WRITE, 0x5B, 0xA0, 0},
    {I_WRITE, 0x4E, 0xA8, 0}, {I_WRITE, 0x5A, 0x50, 0},
    {I_WRITE, 0x40, 0x80, 0},
};
#define INIT_LEN ((int)(sizeof(INIT_TABLE) / sizeof(INIT_TABLE[0])))

/* ── Bringup state machine (RESTRIÇÃO 3) ──────────────────────── */
typedef enum {
    BU_PROBE_PID,
    BU_PROBE_INV_PID,
    BU_POWER_RESET,
    BU_WAIT_RESET,
    BU_DUMMY_READS,
    BU_INIT_TABLE,
    BU_DONE,
    BU_FAILED,
} bringup_state_t;

static bringup_state_t   _bu = BU_PROBE_PID;
static int               _dummy_idx;
static int               _init_idx;
static pmw3901_done_cb_t _init_done;
static repeating_timer_t _poll_timer;

static const uint8_t DUMMY_REGS[] = { 0x02, 0x03, 0x04, 0x05, 0x06 };
#define N_DUMMY ((int)(sizeof(DUMMY_REGS)))

/* ── Sensor state (shared with ISR) ──────────────────────────── */
static volatile float _disp_x = 0.0f, _disp_y = 0.0f;
static volatile float _vel_dx  = 0.0f, _vel_dy  = 0.0f;
static volatile bool  _mot_pending     = false;
static volatile bool  _read_chain_busy = false;
static volatile bool  _ready           = false;

static absolute_time_t _last_read_ts;
static uint64_t        _chain_start_us = 0;
#define CHAIN_TIMEOUT_US 200000u

/* read chain intermediates — written only from alarm IRQ */
static uint8_t _rd_dxL, _rd_dxH;
static uint8_t _rd_dyL, _rd_dyH;
static uint8_t _rd_squal;
static uint8_t _rd_shL;

/* ── Forward declarations ─────────────────────────────────────── */
static void    bringup_step    (void *u, uint8_t val);
static void    init_advance    (void *u, uint8_t val);
static int64_t alarm_rst_done  (alarm_id_t id, void *u);
static int64_t alarm_init_dly  (alarm_id_t id, void *u);
static bool    poll_timer_cb   (repeating_timer_t *rt);

static void rd_done_motion(void *u, uint8_t val);
static void rd_done_dxL   (void *u, uint8_t val);
static void rd_done_dxH   (void *u, uint8_t val);
static void rd_done_dyL   (void *u, uint8_t val);
static void rd_done_dyH   (void *u, uint8_t val);
static void rd_done_squal (void *u, uint8_t val);
static void rd_done_shL   (void *u, uint8_t val);
static void rd_done_shH   (void *u, uint8_t val);

/* ── Bringup ──────────────────────────────────────────────────── */
static void fail_bringup(void) {
    _bu = BU_FAILED;
    if (_init_done) _init_done(false);
}

static void bringup_step(void *u, uint8_t val) {
    (void)u;
    switch (_bu) {
    case BU_PROBE_PID:
        if (val != PID_EXPECTED) { fail_bringup(); return; }
        _bu = BU_PROBE_INV_PID;
        spi_async_read(REG_INV_PRODUCT_ID, bringup_step, NULL);
        break;
    case BU_PROBE_INV_PID:
        if (val != INV_PID_EXPECTED) { fail_bringup(); return; }
        _bu = BU_POWER_RESET;
        spi_async_write(REG_POWER_UP_RESET, 0x5A, bringup_step, NULL);
        break;
    case BU_POWER_RESET:
        _bu = BU_WAIT_RESET;
        add_alarm_in_ms(5, alarm_rst_done, NULL, true);
        break;
    case BU_WAIT_RESET:
        _dummy_idx = 0;
        _bu = BU_DUMMY_READS;
        spi_async_read(DUMMY_REGS[_dummy_idx++], bringup_step, NULL);
        break;
    case BU_DUMMY_READS:
        if (_dummy_idx < N_DUMMY) {
            spi_async_read(DUMMY_REGS[_dummy_idx++], bringup_step, NULL);
        } else {
            _bu = BU_INIT_TABLE;
            _init_idx = 0;
            init_advance(NULL, 0);
        }
        break;
    default:
        break;
    }
}

static int64_t alarm_rst_done(alarm_id_t id, void *u) {
    (void)id; (void)u;
    bringup_step(NULL, 0);
    return 0;
}

/* ── Init table sequencer (RESTRIÇÃO 2) ───────────────────────── */
static void init_advance(void *u, uint8_t val) {
    (void)u; (void)val;
    if (_init_idx >= INIT_LEN) {
        _bu    = BU_DONE;
        _ready = true;
        _last_read_ts = get_absolute_time();
        add_repeating_timer_ms(10, poll_timer_cb, NULL, &_poll_timer);
        if (_init_done) _init_done(true);
        return;
    }
    const init_item_t *it = &INIT_TABLE[_init_idx++];
    if (it->kind == I_WRITE) {
        spi_async_write(it->reg, it->val, init_advance, NULL);
    } else {
        add_alarm_in_ms(it->ms, alarm_init_dly, NULL, true);
    }
}

static int64_t alarm_init_dly(alarm_id_t id, void *u) {
    (void)id; (void)u;
    init_advance(NULL, 0);
    return 0;
}

/* ── Repeating timer (RESTRIÇÃO 5) ───────────────────────────── */
static bool poll_timer_cb(repeating_timer_t *rt) {
    (void)rt;
    _mot_pending = true;
    return true;
}

/* ── Read chain (RESTRIÇÃO 4) ────────────────────────────────── */
static void rd_done_motion(void *u, uint8_t val) {
    (void)u;
    if (!(val & 0x80)) {           /* no motion: abandon chain */
        _read_chain_busy = false;
        return;
    }
    spi_async_read(REG_DELTA_X_L, rd_done_dxL, NULL);
}

static void rd_done_dxL(void *u, uint8_t val) {
    (void)u; _rd_dxL = val;
    spi_async_read(REG_DELTA_X_H, rd_done_dxH, NULL);
}

static void rd_done_dxH(void *u, uint8_t val) {
    (void)u; _rd_dxH = val;
    spi_async_read(REG_DELTA_Y_L, rd_done_dyL, NULL);
}

static void rd_done_dyL(void *u, uint8_t val) {
    (void)u; _rd_dyL = val;
    spi_async_read(REG_DELTA_Y_H, rd_done_dyH, NULL);
}

static void rd_done_dyH(void *u, uint8_t val) {
    (void)u; _rd_dyH = val;
    spi_async_read(REG_SQUAL, rd_done_squal, NULL);
}

static void rd_done_squal(void *u, uint8_t val) {
    (void)u; _rd_squal = val;
    spi_async_read(REG_SHUTTER_L, rd_done_shL, NULL);
}

static void rd_done_shL(void *u, uint8_t val) {
    (void)u; _rd_shL = val;
    spi_async_read(REG_SHUTTER_U, rd_done_shH, NULL);
}

/*
 * Final callback: apply 3-stage filter (RESTRIÇÃO 4) and integrate.
 * Runs in alarm IRQ context — keep brief; no blocking calls.
 */
static void rd_done_shH(void *u, uint8_t val) {
    (void)u; (void)val;

    absolute_time_t now = get_absolute_time();
    float dt = (float)absolute_time_diff_us(_last_read_ts, now) * 1e-6f;
    _last_read_ts = now;

    /* Filter 1: freeze position and zero velocity when quality is low */
    if (_rd_squal < SQUAL_MIN) {
        _vel_dx = 0.0f;
        _vel_dy = 0.0f;
        _read_chain_busy = false;
        return;
    }

    /* Filter 2: skip integration if dt is out of bounds */
    if (dt <= 0.0f || dt >= 0.2f) {
        _read_chain_busy = false;
        return;
    }

    /* Filter 3: integrate only on good quality */
    int16_t dx = (int16_t)(((uint16_t)_rd_dxH << 8) | _rd_dxL);
    int16_t dy = (int16_t)(((uint16_t)_rd_dyH << 8) | _rd_dyL);

    float fx = (float)dx * PMW_SCALE;
    float fy = (float)dy * PMW_SCALE;

    _disp_x += fx;
    _disp_y += fy;
    _vel_dx  = fx / dt;
    _vel_dy  = fy / dt;

    _read_chain_busy = false;
}

/* ── Public API ───────────────────────────────────────────────── */
void pmw3901_init(pmw3901_done_cb_t on_done) {
    _init_done = on_done;

    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 0);
    busy_wait_ms(10);
    gpio_put(PIN_RESET, 1);
    busy_wait_ms(5);

    spi_async_init(spi0, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS, 2000000);
    _bu = BU_PROBE_PID;
    spi_async_read(REG_PRODUCT_ID, bringup_step, NULL);
}

bool pmw3901_ready(void) { return _ready; }

void pmw3901_service(void) {
    if (_read_chain_busy) {
        if (time_us_64() - _chain_start_us > CHAIN_TIMEOUT_US) {
            spi_async_abort();
            _read_chain_busy = false;
        }
        return;
    }
    if (_mot_pending) {
        _mot_pending     = false;
        _read_chain_busy = true;
        _chain_start_us  = time_us_64();
        spi_async_read(REG_MOTION, rd_done_motion, NULL);
    }
}

void pmw3901_snapshot(float *dx, float *dy, float *vdx, float *vdy) {
    uint32_t save = save_and_disable_interrupts();
    *dx  = _disp_x; _disp_x = 0.0f;
    *dy  = _disp_y; _disp_y = 0.0f;
    *vdx = _vel_dx;
    *vdy = _vel_dy;
    restore_interrupts(save);
}
