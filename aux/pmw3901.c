#include <time.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pmw3901.h"

static inline void cs_activate() {
    asm volatile("nop \n nop \n nop"); 
    gpio_put(PIN_CS, 0); 
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deactivate() {
    asm volatile("nop \n nop \n nop"); 
    gpio_put(PIN_CS, 1); 
    asm volatile("nop \n nop \n nop");
}

void init_spi() {
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    spi_init(spi0, 2000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
}

void pmw3901_write(uint8_t reg, uint8_t data) {
    uint8_t buffer[] = {reg | 0x80, data}; //Máscara para escrita 1000 0000 
    
    cs_activate();
    spi_write_blocking(spi0, buffer, 2);
    cs_deactivate();
    sleep_us(50);
}

uint8_t pmw3901_read(uint8_t reg) {
    uint8_t addr = reg & 0x7F; //Máscara para leitura 0111 1111
    uint8_t value = 0;

    cs_activate();
    spi_write_blocking(spi0, &addr, 1);
    sleep_us(50);
    spi_read_blocking(spi0, 0, &value, 1);
    cs_deactivate();

    return value;
}

//Latência absurda
void pmw3901_init_registers() {
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x61, 0xAD);
    pmw3901_write(0x7F, 0x03);
    pmw3901_write(0x40, 0x00);
    pmw3901_write(0x7F, 0x05);
    pmw3901_write(0x41, 0xB3);
    pmw3901_write(0x43, 0xF1);
    pmw3901_write(0x45, 0x14);
    pmw3901_write(0x5B, 0x32);
    pmw3901_write(0x5F, 0x34);
    pmw3901_write(0x7B, 0x08);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x44, 0x1B);
    pmw3901_write(0x40, 0xBF);
    pmw3901_write(0x4E, 0x3F);
    pmw3901_write(0x7F, 0x08);
    pmw3901_write(0x65, 0x20);
    pmw3901_write(0x6A, 0x18);
    pmw3901_write(0x7F, 0x09);
    pmw3901_write(0x4F, 0xAF);
    pmw3901_write(0x5F, 0x40);
    pmw3901_write(0x48, 0x80);
    pmw3901_write(0x49, 0x80);
    pmw3901_write(0x57, 0x77);
    pmw3901_write(0x60, 0x78);
    pmw3901_write(0x61, 0x78);
    pmw3901_write(0x62, 0x08);
    pmw3901_write(0x63, 0x50);
    pmw3901_write(0x7F, 0x0A);
    pmw3901_write(0x45, 0x60);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x4D, 0x11);
    pmw3901_write(0x55, 0x80);
    pmw3901_write(0x74, 0x1F);
    pmw3901_write(0x75, 0x1F);
    pmw3901_write(0x4A, 0x78);
    pmw3901_write(0x4B, 0x78);
    pmw3901_write(0x44, 0x08);
    pmw3901_write(0x45, 0x50);
    pmw3901_write(0x64, 0xFF);
    pmw3901_write(0x65, 0x1F);
    pmw3901_write(0x7F, 0x14);
    pmw3901_write(0x65, 0x60);
    pmw3901_write(0x66, 0x08);
    pmw3901_write(0x63, 0x78);
    pmw3901_write(0x7F, 0x15);
    pmw3901_write(0x48, 0x58);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x41, 0x0D);
    pmw3901_write(0x43, 0x14);
    pmw3901_write(0x4B, 0x0E);
    pmw3901_write(0x45, 0x0F);
    pmw3901_write(0x44, 0x42);
    pmw3901_write(0x4C, 0x80);
    pmw3901_write(0x7F, 0x10);
    pmw3901_write(0x5B, 0x02);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x41);
    pmw3901_write(0x70, 0x00);

    sleep_ms(100);
    pmw3901_write(0x32, 0x44);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x40);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x62, 0xf0);
    pmw3901_write(0x63, 0x00);
    pmw3901_write(0x7F, 0x0D);
    pmw3901_write(0x48, 0xC0);
    pmw3901_write(0x6F, 0xd5);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x5B, 0xa0);
    pmw3901_write(0x4E, 0xA8);
    pmw3901_write(0x5A, 0x50);
    pmw3901_write(0x40, 0x80);
}

void pmw3901_update_values(pmw3901_t* sensor) {
    if(sensor == NULL) return;

    uint8_t motion_reg = pmw3901_read(MOTION);
    sensor->motion = (motion_reg & 0x80) ? 1 : 0;
    sensor->squal = pmw3901_read(SQUAL);

    if(!sensor->motion) {
        sensor->dx = 0;
        sensor->dy = 0;
        sensor->dx_m = 0.0f;
        sensor->dy_m = 0.0f;
        return;
    }

    uint8_t h, l;

    h = pmw3901_read(DELTA_X_H);
    l = pmw3901_read(DELTA_X_L);
    sensor->dx = (int16_t)((h << 8) | l);

    h = pmw3901_read(DELTA_Y_H);
    l = pmw3901_read(DELTA_Y_L);
    sensor->dy = (int16_t)((h << 8) | l);

    sensor->dx_m = sensor->dx * PMW_SCALE;
    sensor->dy_m = sensor->dy * PMW_SCALE;

}