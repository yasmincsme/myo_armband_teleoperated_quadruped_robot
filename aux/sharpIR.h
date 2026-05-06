#ifndef SHARPIR_H
#define SHARPIR_H

#include <stdio.h>
#include <stdint.h>

#define IR_SENSOR_ADC_INPUT 2  // ADC2 = GPIO28
#define IR_SENSOR_PIN 28

float sharpIR_read(uint16_t* raw_out, float* voltage_out);
void sharpIR_init(void);

#endif