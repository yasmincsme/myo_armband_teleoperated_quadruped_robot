#include "i2c_dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

/* Maximum bytes in one async read (MPU9250 accel+gyro = 14 bytes) */
#define MAX_I2C_LEN 16

static i2c_inst_t *_i2c;
static int         _ch_tx, _ch_rx;
static uint        _dreq_tx, _dreq_rx;

static volatile bool  _busy = false;
static uint8_t       *_dst;
static uint8_t        _len;
static i2c_done_cb_t  _done_cb;
static void          *_done_user;

/* TX: command words sent to IC_DATA_CMD (32-bit: bit9=STOP, bit8=CMD, bits7:0=data) */
static uint32_t _cmd_buf[1 + MAX_I2C_LEN];
/* RX: 32-bit words read from IC_DATA_CMD; received byte is in bits [7:0] */
static uint32_t _rx_buf[MAX_I2C_LEN];

static void _set_target(uint8_t addr) {
    /* IC_TAR can only be changed while the controller is disabled */
    i2c_get_hw(_i2c)->enable = 0;
    i2c_get_hw(_i2c)->tar   = addr;
    i2c_get_hw(_i2c)->enable = 1;
}

static void i2c_dma_irq_handler(void) {
    dma_hw->ints1 = 1u << _ch_rx;

    for (int i = 0; i < _len; i++)
        _dst[i] = (uint8_t)(_rx_buf[i] & 0xFF);

    _busy = false;
    i2c_done_cb_t cb = _done_cb;
    void         *u  = _done_user;
    _done_cb = NULL;
    if (cb) cb(u);
}

void i2c_dma_init(i2c_inst_t *i2c, uint sda, uint scl, uint baud) {
    _i2c = i2c;

    i2c_init(i2c, baud);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    _ch_tx   = dma_claim_unused_channel(true);
    _ch_rx   = dma_claim_unused_channel(true);
    _dreq_tx = (i2c == i2c0) ? DREQ_I2C0_TX : DREQ_I2C1_TX;
    _dreq_rx = (i2c == i2c0) ? DREQ_I2C0_RX : DREQ_I2C1_RX;

    dma_channel_set_irq1_enabled(_ch_rx, true);
    irq_set_exclusive_handler(DMA_IRQ_1, i2c_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

bool i2c_dma_busy(void) { return _busy; }

void i2c_dma_read(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len,
                  i2c_done_cb_t cb, void *user) {
    _dst       = dst;
    _len       = len;
    _done_cb   = cb;
    _done_user = user;
    _busy      = true;

    _set_target(addr);

    /* Command buffer: write reg address, then len read commands.
       Last read command carries the STOP bit (bit9). */
    _cmd_buf[0] = reg & 0xFF;              /* write, no STOP */
    for (int i = 1; i < len; i++)
        _cmd_buf[i] = 0x100;               /* read, no STOP */
    _cmd_buf[len] = 0x300;                 /* read + STOP */

    dma_channel_config c;

    /* Arm RX before TX so no bytes are missed */
    c = dma_channel_get_default_config(_ch_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, _dreq_rx);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(_ch_rx, &c, _rx_buf,
                          &i2c_get_hw(_i2c)->data_cmd, len, true);

    c = dma_channel_get_default_config(_ch_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, _dreq_tx);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(_ch_tx, &c,
                          &i2c_get_hw(_i2c)->data_cmd, _cmd_buf, 1 + len, true);
}
