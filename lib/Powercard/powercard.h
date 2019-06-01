#ifndef powercard_h
#define powercard_h
#endif

#include "Arduino.h"
#include "MCP342X.h"
#include "MCP47X6.h"
#include "MCP23008.h"
#include "Wire.h"

class powercard
{
private:
    
    long DAC_bits = 4095;
    long ADC_bits = 32767;
    float _vout = 0.000, _vin =0.000, _current = 0.000, _T =0.000;
    int16_t _vout_adc, _vin_adc, _current_adc, _T_adc;
    float _max_volt = 20.48 , _min_volt = 0.00, _max_current = 2.048, _min_current = 0.000;   
    
public:
    uint8_t volt_add, curr_add, adc_add, IO_add;
    powercard (uint8_t volt_adr, uint8_t curr_adr, uint8_t adc_adr, uint8_t IO_adr);
    MCP47X6 dac_volt;
    MCP47X6 dac_current;
    MCP342X ADC_measure;
    void begin ();
    float read(int p);
    void write (float set, int j);
};