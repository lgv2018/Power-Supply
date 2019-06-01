#include "powercard.h"


powercard::powercard (uint8_t volt_adr, uint8_t curr_adr, uint8_t adc_adr, uint8_t IO_adr){
    volt_add = volt_adr;
    curr_add = curr_adr;
    adc_add = adc_adr;
    IO_add = IO_adr;    
}

void powercard::begin(){

    dac_volt = MCP47X6 (volt_add);
    dac_current = MCP47X6 (curr_add);
    ADC_measure = MCP342X (adc_add);

    dac_volt.begin();
    dac_volt.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
    dac_volt.setGain(MCP47X6_GAIN_1X);
    dac_volt.saveSettings();

    dac_current.begin();
    dac_current.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
    dac_current.setGain(MCP47X6_GAIN_1X);
    dac_current.saveSettings();

    ADC_measure.configure(MCP342X_MODE_CONTINUOUS | 
                  MCP342X_CHANNEL_1 |
                  MCP342X_CHANNEL_2 |
                  MCP342X_CHANNEL_3 |
                  MCP342X_CHANNEL_4 |
                  MCP342X_SIZE_16BIT |
                  MCP342X_GAIN_1X);   
}

float powercard::read(int p){
    switch (p)
    {
    case 0:
        ADC_measure.startConversion(MCP342X_CHANNEL_1);
        ADC_measure.getResult(&_vout_adc);
        _vout = _vout_adc*_max_volt/ADC_bits;
        return _vout;
        break;
    case 1:
        ADC_measure.startConversion(MCP342X_CHANNEL_2);
        ADC_measure.getResult(&_current_adc);
        _current = _current_adc*_max_current/ADC_bits;
        return _current;
        break;
    case 2:
        ADC_measure.startConversion(MCP342X_CHANNEL_3);
        ADC_measure.getResult(&_vin_adc);
        _vin = _vin_adc*_max_volt*4/ADC_bits;
        return _vin;
        break;
    case 3:
        ADC_measure.startConversion(MCP342X_CHANNEL_4);
        ADC_measure.getResult(&_T_adc);
        _T = ((_T_adc*_max_current/ADC_bits)-0.5)*100;
        return _T;
        break;
    default:
        break;
    }
return 0;
}

void powercard::write(float set, int j){

    switch (j)
    {
    case 0:
        dac_volt.setOutputLevel (uint16_t (set*DAC_bits/_max_volt));
        break;
    case 1:
        dac_current.setOutputLevel (uint16_t (set*DAC_bits/_max_current));
        break;
    default:
        break;
    }
}