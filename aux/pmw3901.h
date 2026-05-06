#ifndef PMW3901_H
#define PMW3901_H

#include <stdio.h>

#define PMW_SCALE  0.001 //calibrar depois de acordo com datasheet
#define DT         0.05 //precisa bater com o valor do timer do microros

//Pinout for Pico 2
#define PIN_MISO      16 
#define PIN_MOSI      19
#define PIN_SCK       18
#define PIN_CS        17
#define INTERFACE_SPI spi0

//Register List
#define PRODUCT_ID          0x00
#define REVISION_ID         0x01
#define MOTION              0x02
#define DELTA_X_L           0x03
#define DELTA_X_H           0x04
#define DELTA_Y_L           0x05
#define DELTA_Y_H           0x06
#define SQUAL               0x07
#define RAWDATA_SUM         0x08
#define MAXIMUM_RAWDATA     0x09
#define MINIMUM_RAWDATA     0x0A
#define SHUTTER_LOWER       0x0B
#define SHUTTER_UPPER       0x0C
#define OBSERVATION         0x15
#define MOTION_BURST        0x16
#define POWER_UP_RESET      0x3A
#define SHUTDOWN            0x3B
#define RAWDATA_GRAB        0x58
#define RAWDATA_GRAB_STATUS 0x59
#define INVERSE_PRODUCT_ID  0x5F

//Reset Register List
#define PRODUCT_ID_RST          0x49
#define REVISION_ID_RST         0x00
#define MOTION_RST              0x00
#define DELTA_X_L_RST           0x00
#define DELTA_X_H_RST           0x00
#define DELTA_Y_L_RST           0x00
#define DELTA_Y_H_RST           0x00
#define SQUAL_RST               0x00
#define RAWDATA_SUM_RST         0x00
#define MAXIMUM_RAWDATA_RST     0x00
#define MINIMUM_RAWDATA_RST     0x00
#define SHUTTER_LOWER_RST       0x00
#define SHUTTER_UPPER_RST       0x00
#define OBSERVATION_RST         0x00
#define MOTION_BURST_RST        0x00
#define RAWDATA_GRAB_RST        0x00
#define RAWDATA_GRAB_STATUS_RST 0x00
#define INVERSE_PRODUCT_ID_RST  0xB6

typedef struct {
    int16_t dx;
    int16_t dy;

    uint8_t motion;
    uint8_t squal;

    uint16_t shutter;
    uint16_t frame_period;

    float dx_m;   
    float dy_m;

} pmw3901_t;


static inline void cs_activate();
static inline void cs_deactivate();
void init_spi();
void pmw3901_write(uint8_t reg, uint8_t data);
uint8_t pmw3901_read(uint8_t reg);
void pmw3901_init_registers();
void pmw3901_update_values(pmw3901_t* sensor);

#endif 
