#include "sharpIR.h"

#include "hardware/adc.h"

float sharpIR_read(uint16_t* raw_out, float* voltage_out) {
    adc_select_input(IR_SENSOR_ADC_INPUT);
    uint16_t raw = adc_read();

    float voltage = raw * 3.3f / 4095.0f; //Corrigir isso aqui
    float distance = (27.86f * (1.0f /voltage)) - 0.42f;

    if (distance < 10.0f) distance = 10.0f;
    if (distance > 80.0f) distance = 80.0f;

    if(raw_out) *raw_out = raw;
    if(voltage_out) *voltage_out = voltage;

    return distance;
}

void sharpIR_init(void) {
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
}