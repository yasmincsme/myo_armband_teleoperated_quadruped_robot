#include "i2c_dma.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/regs/i2c.h"

/*
 * I2C DMA driver for RP2350.
 *
 * Protocol per transaction:
 *   TX DMA (DMA_SIZE_16): streams 16-bit IC_DATA_CMD words.
 *     _cmd_buf[0]          = register address   (write, no STOP)
 *     _cmd_buf[1]          = READ | RESTART      (first read byte)
 *     _cmd_buf[2..len-1]   = READ
 *     _cmd_buf[len]        = READ | STOP         (last byte)
 *
 *   RX DMA (DMA_SIZE_8): reads the low byte of IC_DATA_CMD into caller's buffer.
 *     Each read command in TX causes one byte to appear in the RX FIFO.
 *
 * IRQ fires on DMA_IRQ_1 (RX channel) when all bytes are received.
 */

#define MAX_LEN 16u   /* ≥14 for accel/gyro, ≥7 for mag */

/* IC_DATA_CMD control-bit positions */
#define CMD_READ     (1u << 8)
#define CMD_STOP     (1u << 9)
#define CMD_RESTART  (1u << 10)

static i2c_inst_t        *_i2c;
static int                _ch_tx, _ch_rx;

static uint16_t           _cmd_buf[MAX_LEN + 1]; /* write + N reads */
static volatile bool      _busy = false;
static i2c_dma_done_cb_t  _done_cb;
static void               *_done_user;

/* ── IRQ ──────────────────────────────────────────────────────── */
static void irq_handler(void) {
    if (!dma_channel_get_irq1_status(_ch_rx)) return;
    dma_channel_acknowledge_irq1(_ch_rx);

    _busy = false;
    i2c_dma_done_cb_t cb = _done_cb;
    void *u = _done_user;
    _done_cb = NULL;
    if (cb) cb(u);
}

/* ── Public API ───────────────────────────────────────────────── */
void i2c_dma_init(i2c_inst_t *i2c) {
    _i2c    = i2c;
    _busy   = false;
    _done_cb = NULL;

    _ch_tx = dma_claim_unused_channel(true);
    _ch_rx = dma_claim_unused_channel(true);

    dma_channel_set_irq1_enabled(_ch_rx, true);
    irq_set_exclusive_handler(DMA_IRQ_1, irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

bool i2c_dma_busy(void) { return _busy; }

void i2c_dma_wait_idle(void) {
    /* Spin until the I2C master finishes generating the STOP condition (~10 µs at 100 kHz). */
    while (i2c_get_hw(_i2c)->status & I2C_IC_STATUS_ACTIVITY_BITS)
        tight_loop_contents();
}

void i2c_dma_read(uint8_t addr, uint8_t reg,
                  uint8_t *dst, size_t len,
                  i2c_dma_done_cb_t cb, void *user) {
    _done_cb   = cb;
    _done_user = user;
    _busy      = true;

    /* Reconfigure I2C target address (disable → set → enable flushes FIFOs). */
    i2c_get_hw(_i2c)->enable = 0;
    i2c_get_hw(_i2c)->tar   = addr;
    i2c_get_hw(_i2c)->enable = 1;

    /* Build command buffer. */
    _cmd_buf[0] = reg;           /* write phase: register address */
    for (size_t i = 0; i < len; i++) {
        _cmd_buf[1 + i] = CMD_READ;
        if (i == 0)       _cmd_buf[1 + i] |= CMD_RESTART;
        if (i == len - 1) _cmd_buf[1 + i] |= CMD_STOP;
    }

    /* TX DMA: 16-bit command words → IC_DATA_CMD */
    dma_channel_config c = dma_channel_get_default_config(_ch_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, i2c_get_dreq(_i2c, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(_ch_tx, &c,
        &i2c_get_hw(_i2c)->data_cmd, _cmd_buf, len + 1, false);

    /* RX DMA: 8-bit data bytes ← IC_DATA_CMD[7:0] directly into caller's buffer */
    c = dma_channel_get_default_config(_ch_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, i2c_get_dreq(_i2c, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(_ch_rx, &c,
        dst, &i2c_get_hw(_i2c)->data_cmd, len, false);

    dma_start_channel_mask((1u << _ch_tx) | (1u << _ch_rx));
}
