#include "spi_async.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/time.h"

/* PMW3901 SPI timing (µs) */
#define T_SRAD_US  35u
#define T_SWW_US   50u

typedef enum {
    ST_IDLE,
    ST_TX_ADDR,       /* read: sent address byte, waiting tSRAD  */
    ST_WAIT_TSRAD,
    ST_RX_DATA,       /* read: receiving data byte                */
    ST_TX_WRITE,      /* write: sent reg+val bytes                */
    ST_WAIT_TSWW,
} spi_state_t;

static spi_inst_t          *_spi;
static uint                 _cs_pin;
static int                  _ch_tx, _ch_rx;
static volatile spi_state_t _state = ST_IDLE;

static spi_done_cb_t    _cb;
static void            *_cb_user;
static volatile uint8_t _read_val;

static uint8_t _tx_buf[2];
static uint8_t _dummy_tx = 0x00;
static uint8_t _dummy_rx;

/* ── DMA helpers ──────────────────────────────────────────────── */
static void start_dma(const void *tx, bool tx_inc,
                      void *rx,       bool rx_inc, size_t n)
{
    dma_channel_config c;

    c = dma_channel_get_default_config(_ch_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(_spi, true));
    channel_config_set_read_increment(&c, tx_inc);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(_ch_tx, &c,
        &spi_get_hw(_spi)->dr, tx, n, false);

    c = dma_channel_get_default_config(_ch_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(_spi, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, rx_inc);
    dma_channel_configure(_ch_rx, &c,
        rx, &spi_get_hw(_spi)->dr, n, false);

    dma_start_channel_mask((1u << _ch_tx) | (1u << _ch_rx));
}

/* ── Alarm callbacks ──────────────────────────────────────────── */
static int64_t alarm_tsrad(alarm_id_t id, void *u) {
    (void)id; (void)u;
    _state = ST_RX_DATA;
    start_dma(&_dummy_tx, false, (void *)&_read_val, false, 1);
    return 0;
}

static int64_t alarm_tsww(alarm_id_t id, void *u) {
    (void)id; (void)u;
    _state = ST_IDLE;
    spi_done_cb_t cb = _cb;
    void         *ud = _cb_user;
    _cb = NULL;
    if (cb) cb(ud, _read_val);
    return 0;
}

/* ── DMA RX IRQ (DMA_IRQ_0) ──────────────────────────────────── */
static void dma_irq_handler(void) {
    if (!dma_channel_get_irq0_status(_ch_rx)) return;
    dma_channel_acknowledge_irq0(_ch_rx);

    /* RX DMA completion guarantees SPI shift register is done:
       in full-duplex SPI, a byte appears in RX FIFO only after the
       corresponding TX clock cycle completes — no spin needed. */
    switch (_state) {
    case ST_TX_ADDR:
        _state = ST_WAIT_TSRAD;
        add_alarm_in_us(T_SRAD_US, alarm_tsrad, NULL, true);
        break;
    case ST_RX_DATA:
        gpio_put(_cs_pin, 1);
        _state = ST_WAIT_TSWW;
        add_alarm_in_us(T_SWW_US, alarm_tsww, NULL, true);
        break;
    case ST_TX_WRITE:
        gpio_put(_cs_pin, 1);
        _state = ST_WAIT_TSWW;
        add_alarm_in_us(T_SWW_US, alarm_tsww, NULL, true);
        break;
    default:
        break;
    }
}

/* ── Public API ───────────────────────────────────────────────── */
void spi_async_init(spi_inst_t *spi, uint pin_miso, uint pin_mosi,
                    uint pin_sck, uint pin_cs, uint baudrate)
{
    _spi    = spi;
    _cs_pin = pin_cs;
    _state  = ST_IDLE;
    _cb     = NULL;

    spi_init(spi, baudrate);
    spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pin_sck,  GPIO_FUNC_SPI);

    gpio_init(pin_cs);
    gpio_set_dir(pin_cs, GPIO_OUT);
    gpio_put(pin_cs, 1);

    _ch_tx = dma_claim_unused_channel(true);
    _ch_rx = dma_claim_unused_channel(true);

    dma_channel_set_irq0_enabled(_ch_rx, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void spi_async_read(uint8_t reg, spi_done_cb_t cb, void *user) {
    _cb        = cb;
    _cb_user   = user;
    _read_val  = 0;
    _tx_buf[0] = reg & 0x7F;
    gpio_put(_cs_pin, 0);
    _state = ST_TX_ADDR;
    start_dma(_tx_buf, true, &_dummy_rx, false, 1);
}

void spi_async_write(uint8_t reg, uint8_t val, spi_done_cb_t cb, void *user) {
    _cb        = cb;
    _cb_user   = user;
    _read_val  = 0;
    _tx_buf[0] = reg | 0x80;
    _tx_buf[1] = val;
    gpio_put(_cs_pin, 0);
    _state = ST_TX_WRITE;
    start_dma(_tx_buf, true, &_dummy_rx, false, 2);
}

bool spi_async_busy(void) { return _state != ST_IDLE; }

void spi_async_abort(void) {
    dma_channel_abort(_ch_tx);
    dma_channel_abort(_ch_rx);
    dma_channel_acknowledge_irq0(_ch_rx);
    dma_channel_acknowledge_irq0(_ch_tx);
    gpio_put(_cs_pin, 1);
    _state = ST_IDLE;
    _cb    = NULL;
}
