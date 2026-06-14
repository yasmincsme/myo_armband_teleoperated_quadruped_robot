#include "i2c_dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

/* IC_DATA_CMD bit fields (RP2350 / RP2040 I2C register) */
#define IC_DATA_CMD_CMD      (1u << 8)   /* 1 = read */
#define IC_DATA_CMD_STOP     (1u << 9)
#define IC_DATA_CMD_RESTART  (1u << 10)

/* Maximum transaction size: 1 reg write + restart + 14 bytes read (MPU burst) */
#define TX_WORD_BUF_LEN 32
static uint32_t _tx_words[TX_WORD_BUF_LEN];

void i2c_dma_init(i2c_dma_t  *dev,
                  i2c_inst_t *i2c,
                  uint        baud,
                  uint        pin_sda,
                  uint        pin_scl)
{
    dev->i2c       = i2c;
    dev->busy      = false;
    dev->on_done   = NULL;
    dev->user_data = NULL;

    i2c_init(i2c, baud);
    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);

    dev->ch_tx = dma_claim_unused_channel(true);

    /* Enable STOP_DET interrupt from the I2C peripheral */
    i2c_get_hw(i2c)->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS;
}

/* ── Internal: start DMA TX of pre-built _tx_words array ──────────────────── */
static void _start_tx(i2c_dma_t *dev, size_t word_count) {
    dma_channel_config c = dma_channel_get_default_config(dev->ch_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, i2c_get_dreq(dev->i2c, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(dev->ch_tx, &c,
        &i2c_get_hw(dev->i2c)->data_cmd,
        _tx_words,
        word_count, true);
}

void i2c_dma_write(i2c_dma_t        *dev,
                   uint8_t           addr,
                   const uint8_t    *data,
                   size_t            len,
                   i2c_dma_done_cb_t cb,
                   void             *user_data)
{
    dev->addr      = addr;
    dev->on_done   = cb;
    dev->user_data = user_data;
    dev->busy      = true;
    dev->rx_buf    = NULL;
    dev->rx_len    = 0;
    dev->rx_idx    = 0;

    i2c_get_hw(dev->i2c)->enable = 0;
    i2c_get_hw(dev->i2c)->tar   = addr;
    i2c_get_hw(dev->i2c)->enable = 1;

    for (size_t i = 0; i < len; i++) {
        _tx_words[i] = data[i]
            | (i == len - 1 ? IC_DATA_CMD_STOP : 0);
    }
    _start_tx(dev, len);
}

void i2c_dma_write_read(i2c_dma_t        *dev,
                         uint8_t           addr,
                         uint8_t           reg,
                         uint8_t          *rx_buf,
                         size_t            rx_len,
                         i2c_dma_done_cb_t cb,
                         void             *user_data)
{
    dev->addr      = addr;
    dev->on_done   = cb;
    dev->user_data = user_data;
    dev->busy      = true;
    dev->rx_buf    = rx_buf;
    dev->rx_len    = rx_len;
    dev->rx_idx    = 0;

    i2c_get_hw(dev->i2c)->enable = 0;
    i2c_get_hw(dev->i2c)->tar   = addr;
    i2c_get_hw(dev->i2c)->enable = 1;

    /* Word 0: write the register address (no STOP – repeated start follows) */
    _tx_words[0] = reg;

    /* Words 1..rx_len: read commands with RESTART on first, STOP on last */
    for (size_t i = 0; i < rx_len; i++) {
        _tx_words[1 + i] = IC_DATA_CMD_CMD
            | (i == 0         ? IC_DATA_CMD_RESTART : 0)
            | (i == rx_len - 1 ? IC_DATA_CMD_STOP    : 0);
    }

    _start_tx(dev, 1 + rx_len);
}

void i2c_dma_irq_handler(i2c_dma_t *dev) {
    i2c_hw_t *hw = i2c_get_hw(dev->i2c);

    if (!(hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS)) return;
    hw->clr_stop_det; /* clear by read */

    bool ok = true;

    /* Drain RX FIFO into user buffer */
    if (dev->rx_buf && dev->rx_len > 0) {
        while (hw->status & I2C_IC_STATUS_RFNE_BITS) {
            if (dev->rx_idx < dev->rx_len)
                dev->rx_buf[dev->rx_idx++] = (uint8_t)(hw->data_cmd & 0xFF);
            else
                (void)hw->data_cmd; /* discard extra */
        }
        ok = (dev->rx_idx == dev->rx_len);
    }

    dev->busy = false;
    if (dev->on_done) dev->on_done(dev->user_data, ok);
}

/* ── Blocking helpers ─────────────────────────────────────────────────────── */

bool i2c_dma_write_reg_blocking(i2c_dma_t *dev, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (i2c_write_timeout_us(dev->i2c, addr, buf, 2, false, 5000) >= 0);
}

bool i2c_dma_read_bytes_blocking(i2c_dma_t *dev, uint8_t addr, uint8_t reg,
                                  uint8_t *buf, size_t len)
{
    if (i2c_write_timeout_us(dev->i2c, addr, &reg, 1, true,  5000) < 0) return false;
    if (i2c_read_timeout_us( dev->i2c, addr, buf,  len, false, 5000) < 0) return false;
    return true;
}
