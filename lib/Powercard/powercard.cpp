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
    port_expander.begin(IO_add);

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
    //GPIO PINMODE CONFIGURATION
    port_expander.pinMode(0, OUTPUT);   //Output off
    port_expander.pinMode(1, INPUT);    //CV_ON
    port_expander.pinMode(2, INPUT);    //CC_ON
    port_expander.pinMode(3, OUTPUT);   //ADC READBACK VA
    port_expander.pinMode(4, OUTPUT);   //ADC READBACK ON
    port_expander.pinMode(5, INPUT);    //CHG_Pump
    //GPIO pullUP CONFIGURATION FOR INPUTS
    port_expander.pullUp (1,HIGH);
    port_expander.pullUp (2,HIGH);
    port_expander.pullUp (5,HIGH);
    //GPIO OUTPUT_OFF By default
    port_expander.digitalWrite(0, HIGH); //output is off by default

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

void powercard::power_on (){
    port_expander.digitalWrite(0, LOW); // card (DAC output) is on when Pin 0 i.e. OUTPUT_OFF is LOW 
}
void powercard::power_off (){
    port_expander.digitalWrite(0, HIGH); // card (DAC output) is OFF when Pin 0 i.e. OUTPUT_OFF is HIGH
}

void powercard::measure_dac_volt_on (){
    port_expander.digitalWrite(4, HIGH); // ADC readback is on when pin 4 i.e.ADC_READBACK_ON is HIGH
    port_expander.digitalWrite(3, LOW); // voltage is measured when pin 3 i.e.ADC_READBACK_VA is LOW
}

void powercard::measure_dac_volt_off (){
    port_expander.digitalWrite(4, LOW); // ADC readback is off when pin 4 i.e.ADC_READBACK_ON is LOW
}

void powercard::measure_dac_curr_on (){
    port_expander.digitalWrite(4, HIGH); // ADC readback is on when pin 4 i.e.ADC_READBACK_ON is HIGH
    port_expander.digitalWrite(3, HIGH); // current is measured when pin 3 i.e.ADC_READBACK_VA is HIGH
}

void powercard::measure_dac_curr_off (){
    port_expander.digitalWrite(4, LOW); // ADC readback is off when pin 4 i.e.ADC_READBACK_ON is LOW
}

