#include "pmw3901.h"
#include "pico/time.h"

static spi_dma_t _spi;

/* ── Blocking helpers (init only) ─────────────────────────────────────────── */

static void _write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg | 0x80, val};
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
    spi_write_blocking(INTERFACE_SPI, buf, 2);
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    sleep_us(180); /* T_SWW */
}

uint8_t pmw3901_read_reg(uint8_t reg) {
    uint8_t addr  = reg & 0x7F;
    uint8_t value = 0;
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
    spi_write_blocking(INTERFACE_SPI, &addr, 1);
    sleep_us(35); /* T_SRAD */
    spi_read_blocking(INTERFACE_SPI, 0, &value, 1);
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    sleep_us(200); /* T_SCLK_NCS_READ */
    return value;
}

/* ── Non-blocking init state machine ──────────────────────────────────────── */

static volatile uint8_t     _stage   = 0;
static pmw3901_done_cb_t    _done_cb = NULL;

static int64_t _alarm_cb(alarm_id_t id, void *user_data) {
    (void)id; (void)user_data;
    _stage++;
    return 0;
}

void pmw3901_init(pmw3901_done_cb_t on_done) {
    _done_cb = on_done;

    spi_dma_init(&_spi, INTERFACE_SPI, PIN_CS,
                 2000 * 1000,
                 PIN_MISO, PIN_MOSI, PIN_SCK,
                 SPI_CPOL_1, SPI_CPHA_1);

    /* Phase 1 register writes */
    _write_reg(0x7F, 0x00); _write_reg(0x61, 0xAD); _write_reg(0x7F, 0x03);
    _write_reg(0x40, 0x00); _write_reg(0x7F, 0x05); _write_reg(0x41, 0xB3);
    _write_reg(0x43, 0xF1); _write_reg(0x45, 0x14); _write_reg(0x5B, 0x32);
    _write_reg(0x5F, 0x34); _write_reg(0x7B, 0x08); _write_reg(0x7F, 0x06);
    _write_reg(0x44, 0x1B); _write_reg(0x40, 0xBF); _write_reg(0x4E, 0x3F);
    _write_reg(0x7F, 0x08); _write_reg(0x65, 0x20); _write_reg(0x6A, 0x18);
    _write_reg(0x7F, 0x09); _write_reg(0x4F, 0xAF); _write_reg(0x5F, 0x40);
    _write_reg(0x48, 0x80); _write_reg(0x49, 0x80); _write_reg(0x57, 0x77);
    _write_reg(0x60, 0x78); _write_reg(0x61, 0x78); _write_reg(0x62, 0x08);
    _write_reg(0x63, 0x50); _write_reg(0x7F, 0x0A); _write_reg(0x45, 0x60);
    _write_reg(0x7F, 0x00); _write_reg(0x4D, 0x11); _write_reg(0x55, 0x80);
    _write_reg(0x74, 0x1F); _write_reg(0x75, 0x1F); _write_reg(0x4A, 0x78);
    _write_reg(0x4B, 0x78); _write_reg(0x44, 0x08); _write_reg(0x45, 0x50);
    _write_reg(0x64, 0xFF); _write_reg(0x65, 0x1F); _write_reg(0x7F, 0x14);
    _write_reg(0x65, 0x60); _write_reg(0x66, 0x08); _write_reg(0x63, 0x78);
    _write_reg(0x7F, 0x15); _write_reg(0x48, 0x58); _write_reg(0x7F, 0x07);
    _write_reg(0x41, 0x0D); _write_reg(0x43, 0x14); _write_reg(0x4B, 0x0E);
    _write_reg(0x45, 0x0F); _write_reg(0x44, 0x42); _write_reg(0x4C, 0x80);
    _write_reg(0x7F, 0x10); _write_reg(0x5B, 0x02); _write_reg(0x7F, 0x07);
    _write_reg(0x40, 0x41); _write_reg(0x70, 0x00);

    _stage = 1;
    add_alarm_in_ms(100, _alarm_cb, NULL, false);
}

void pmw3901_init_poll(void) {
    if (_stage != 2) return;

    _write_reg(0x32, 0x44); _write_reg(0x7F, 0x07); _write_reg(0x40, 0x40);
    _write_reg(0x7F, 0x06); _write_reg(0x62, 0xF0); _write_reg(0x63, 0x00);
    _write_reg(0x7F, 0x0D); _write_reg(0x48, 0xC0); _write_reg(0x6F, 0xD5);
    _write_reg(0x7F, 0x00); _write_reg(0x5B, 0xA0); _write_reg(0x4E, 0xA8);
    _write_reg(0x5A, 0x50); _write_reg(0x40, 0x80);

    _stage = 3;
    if (_done_cb) _done_cb();
}

bool pmw3901_init_done(void) {
    return _stage == 3;
}

/* ── Async burst read ─────────────────────────────────────────────────────── */

/* TX: send register address byte; remaining bytes are 0 (clock in RX data).
   RX byte 0 = dummy (address echo), bytes 1-5 = MOTION, DX_L, DX_H, DY_L, DY_H */
#define BURST_LEN 6

static uint8_t          _tx_buf[BURST_LEN];
static uint8_t          _rx_buf[BURST_LEN];
static pmw3901_t       *_out_ptr   = NULL;
static pmw3901_done_cb_t _read_cb  = NULL;

static void _burst_done(void *user_data) {
    (void)user_data;

    /* _rx_buf[0] = dummy; [1]=MOTION [2]=DX_L [3]=DX_H [4]=DY_L [5]=DY_H */
    uint8_t motion = _rx_buf[1];
    _out_ptr->motion = (motion & 0x80) ? 1 : 0;

    if (_out_ptr->motion) {
        _out_ptr->dx   = (int16_t)((_rx_buf[3] << 8) | _rx_buf[2]);
        _out_ptr->dy   = (int16_t)((_rx_buf[5] << 8) | _rx_buf[4]);
        _out_ptr->dx_m = _out_ptr->dx * PMW_SCALE;
        _out_ptr->dy_m = _out_ptr->dy * PMW_SCALE;
    } else {
        _out_ptr->dx = _out_ptr->dy = 0;
        _out_ptr->dx_m = _out_ptr->dy_m = 0.0f;
    }

    if (_read_cb) _read_cb();
}

void pmw3901_read_async(pmw3901_t *out, pmw3901_done_cb_t on_done) {
    _out_ptr = out;
    _read_cb = on_done;

    /* Address byte for MOTION register (read) followed by 5 dummy bytes */
    _tx_buf[0] = PMW_REG_MOTION & 0x7F;
    for (int i = 1; i < BURST_LEN; i++) _tx_buf[i] = 0x00;

    spi_dma_transfer(&_spi, _tx_buf, _rx_buf, BURST_LEN, _burst_done, NULL);
}
