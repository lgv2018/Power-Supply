#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <MCP47X6.h>
#include <MCP342X.h>

//Defination of I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);
//Defination of card 1
MCP47X6 Card1_dac_volt = MCP47X6(0X60);
MCP47X6 Card1_dac_current =MCP47X6(0X61);
MCP342X Card1_ADC =MCP342X(0x68);
//Defination of card 2
MCP47X6 Card2_dac_volt = MCP47X6(0x62);
MCP47X6 Card2_dac_current =MCP47X6(0x63);
MCP342X Card2_ADC =MCP342X(0x6F);

static int16_t vout1_adc, vin1_adc, current1_adc, T1_adc;
static int16_t vout2_adc, vin2_adc, current2_adc, T2_adc;
int v1_set_raw, v2_set_raw, a1_set_raw, a2_set_raw;
uint16_t v1_set, v2_set, a1_set, a2_set;
float vout1 = 0.000, vin1 =0.000, current1 = 0.000, T1 =0.000;
float vout2 = 0.000, vin2 =0.000, current2 = 0.000, T2 =0.000;
uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};
uint8_t ON[8] = {0x18,0x1C,0x1E,0x1F,0x1E,0x1C,0x18,0x00};
uint8_t OFF[8] = {0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00};

//Variables for the menu encoder
int counter = 0; 
int page=1;
int Ready=1;
int submenu=0;
int menu=0;
int last_counter = 0;   
int pushed = 0;
int DL = 500; // Delay for action
float inc_r = 0.5;
int i;
int j;
int lastctr = 0;
int max_cnt =10;
int min_cnt =0;

//Voltage divider Resistors variable
float R[2][4]; // = {{50.0,60.0,70.0,80.00},{10.0,20.0,30.0,40.0}};
float v_cal[2][4]; //= {{12.0,13.0,14.0,15.00},{5.0,6.0,7.0,8.0}}; //Calibration voltage 
float set_current[2] = {5,5};   //Current settings variable
float set_volt[2] = {1,1};

//Variables for LCD
String voltage_h = "Volt(H):";
String voltage_L = "Volt(L):";
String R1 = "R1=";
String R2 = "R2=";
String Back = "Back";

//Pin assignments
#define push 4
#define CH1_SW 8
#define CH2_SW 9
#define CH3_SW 10
#define CH4_SW 11
#define encoderPinA 2
#define encoderPinB 3
#define Menu 5

volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;            // interrupt service routine vars
boolean B_set = false;            // interrupt service routine vars
void doEncoderA();
void doEncoderB();
void Summaryscreen();

void setup() 
{
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // turn on pullup resistors
  digitalWrite(encoderPinB, HIGH); // turn on pullup resistors
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  Serial.begin(9600);

  //Define the pin as input & OUTPUT
  pinMode (push,INPUT);
  
  // Read EEPROM settings
  int EEPROM_add = 0;
  for(i=0;i<2;i++)
  { 
    for (j=0;j<4;j++)
    {
      R[i][j]=EEPROM.read(EEPROM_add);
      if (R[i][j] >20 || R[i][j] <0) R[i][j] = 10;
      v_cal[i][j]=EEPROM.read(EEPROM_add+10);
      if (v_cal[i][j] >20 || v_cal[i][j] <0) v_cal[i][j] = 10;
      EEPROM_add++;
    }
  }

  // Initialize LCD
  lcd.begin();
  lcd.backlight(); 
  lcd.createChar(1, arrow);
  lcd.createChar(2, ON);
  lcd.createChar(3, OFF);
  lcd.clear();
  lcd.setCursor(0,1);  
  //Initialize DAC1 for setting of the voltage
  Card1_dac_volt.begin();
  Card1_dac_volt.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
  Card1_dac_volt.setGain(MCP47X6_GAIN_1X);
  Card1_dac_volt.saveSettings();
  // Initialize DAC1 for setting of the current
  Card1_dac_current.begin();
  Card1_dac_current.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
  Card1_dac_current.setGain(MCP47X6_GAIN_1X);
  Card1_dac_current.saveSettings();
  // Initialize DAC2 for setting of the voltage
  Card2_dac_volt.begin();
  Card2_dac_volt.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
  Card2_dac_volt.setGain(MCP47X6_GAIN_1X);
  Card2_dac_volt.saveSettings();
  // Initialize DAC2 for setting of the current
  Card2_dac_current.begin();
  Card2_dac_current.setVReference(MCP47X6_VREF_VREFPIN_BUFFERED);
  Card2_dac_current.setGain(MCP47X6_GAIN_1X);
  Card2_dac_current.saveSettings();
  //Initialize ADC for reading of the current & voltage
  Wire.begin();
  Card1_ADC.configure(MCP342X_MODE_CONTINUOUS | 
                  MCP342X_CHANNEL_1 |
                  MCP342X_CHANNEL_2 |
                  MCP342X_CHANNEL_3 |
                  MCP342X_CHANNEL_4 |
                  MCP342X_SIZE_16BIT |
                  MCP342X_GAIN_1X);
  Card2_ADC.configure(MCP342X_MODE_CONTINUOUS | 
                  MCP342X_CHANNEL_1 |
                  MCP342X_CHANNEL_2 |
                  MCP342X_CHANNEL_3 |
                  MCP342X_CHANNEL_4 |
                  MCP342X_SIZE_16BIT |
                  MCP342X_GAIN_1X);

  set_current[0]=analogRead(A0)*2.000/1000;
  set_volt[0]=analogRead(A2)*20.000/1000;
  set_current[1]=analogRead(A1)*2.000/1000;
  set_volt[1]=analogRead(A3)*20.000/1000;

  Card1_dac_current.setOutputLevel(uint16_t (set_current[0]*4096/2.048));
  Card1_dac_volt.setOutputLevel(uint16_t (set_volt[0]*4096/20.48));
  Card2_dac_current.setOutputLevel(uint16_t (set_current[1]*4096/2.048));
  Card2_dac_volt.setOutputLevel(uint16_t (set_volt[1]*4096/20.48));
  
}
void loop() {
  rotating = true;  // reset the debouncer
  Start: 
  if (menu==0)
  {
    Summaryscreen();    
  }

  if((last_counter > counter) || (last_counter < counter)  || pushed) //Only print on the LCD when a step is detected or the button is pushed  
   {
    menu=1;                                                                                                                                                                                                                           menu =1;
    Ready=1;
    if(submenu == 0) //Main Menu
    {  
      max_cnt=2;
      if(0 <= counter && counter < 1)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);  
        lcd.print("Current settings");
        lcd.setCursor(0,1);  
        lcd.print(" Voltage settings");
        lcd.setCursor(1,2);
        lcd.print(Back);
        page=1;
      }
      if(1 <= counter && counter < 2)
      {
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" Current settings");
        lcd.setCursor(0,1);
        lcd.write(1);    
        lcd.print("Voltage settings");
        lcd.setCursor(1,2);
        lcd.print(Back);
        page=2;
      }
      if(2 <= counter && counter < 3)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" Current settings");
        lcd.setCursor(0,1);
        lcd.print(" Voltage settings");
        lcd.setCursor(0,2);
        lcd.write(1);
        lcd.print(Back);
        page=3;
      }
    }//submenu = 0;

    if(submenu == 1) //Current settings
    {  
        max_cnt=3;
        if(0 <= counter && counter < 1)
        {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.write(1);  
          lcd.print("Ch1-Set curr=");
          lcd.setCursor(0,1);  
          lcd.print(" Ch2-Set curr=");
          lcd.setCursor(1,2);  
          lcd.print(Back);
          page=1;
          pushed=0;    
        }
      
        if(1 <= counter && counter < 2)
        {
          lcd.clear();
          lcd.setCursor(0,0);  
          lcd.print(" Ch1-Set curr=");
          lcd.setCursor(0,1); 
          lcd.write(1); 
          lcd.print("Ch2-Set curr=");
          lcd.setCursor(1,2);  
          lcd.print(Back);
          page=2;
          pushed=0;      
        }
      
        if(2 <= counter && counter < 3)
        {
          lcd.clear();
          lcd.setCursor(0,0);  
          lcd.print(" Ch1-Set curr=");
          lcd.setCursor(0,1); 
          lcd.print(" Ch2-Set curr=");
          lcd.setCursor(0,2); 
          lcd.write(1);  
          lcd.print(Back);
          page=3;
          pushed=0;      
        }
        if(page <=3)
        {
          lcd.setCursor(15,0);
          lcd.print(set_current[0],3);
          lcd.setCursor(15,1);
          lcd.print(set_current[1],3);
          delay(100);
        }
        
    }//submenu = 1;

    if(submenu == 2) //Voltage settings
    {  
      max_cnt=3;
      if(0 <= counter && counter < 1)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);  
        lcd.print("Ch1-Set volt=");
        lcd.setCursor(0,1);  
        lcd.print(" Ch2-Set volt=");
        lcd.setCursor(1,2);  
        lcd.print(Back);  
        page=1;
        pushed=0;    
      }
    
      if(1 <= counter && counter < 2)
      {
        lcd.clear();
        lcd.setCursor(0,0);  
        lcd.print(" Ch1-Set volt=");
        lcd.setCursor(0,1);  
        lcd.write(1);
        lcd.print("Ch2-Set volt=");
        lcd.setCursor(1,2);  
        lcd.print(Back); 
        page=2;
        pushed=0;      
      }  

      if(2 <= counter && counter < 3)
      {
        lcd.clear();
        lcd.setCursor(0,0);  
        lcd.print(" Ch1-Set volt=");
        lcd.setCursor(0,1);  
        lcd.print(" Ch2-Set volt=");
        lcd.setCursor(0,2); 
        lcd.write(1);  
        lcd.print(Back);
        page=3;
        pushed=0;      
      }
      if(page <=3)
      {
        lcd.setCursor(15,0);
        lcd.print(set_volt[0],3);
        lcd.setCursor(15,1);
        lcd.print(set_volt[1],3);
        delay(100);
      } 

    }//submenu = 2;

    if(submenu == 3) //Calibration
    {  
      if(0 <= counter && counter < 2)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);  
        lcd.print("Current Sensor");
        lcd.setCursor(0,1);  
        lcd.print(" Voltage Devider"); 
        lcd.setCursor(1,2);  
        lcd.print(Back);
        page=1;
        pushed=0;    
      }
    
      if(2 <= counter && counter < 4)
      {
        lcd.clear();
        lcd.setCursor(0,0);  
        lcd.print(" Current Sensor");
        lcd.setCursor(0,1);  
        lcd.write(1);
        lcd.print("Voltage Devider");
        lcd.setCursor(1,2);  
        lcd.print(Back);
        page=2;
        pushed=0;      
      } 
      if(4 <= counter && counter < 6)
      {
        lcd.clear();
        lcd.setCursor(0,0);  
        lcd.print(" Current Sensor");
        lcd.setCursor(0,1);  
        lcd.print(" Voltage Devider");
        lcd.setCursor(0,2);
        lcd.write(1);  
        lcd.print(Back);
        page=3;
        pushed=0;      
      }  
    }//submenu = 3;
    last_counter = counter; //Save the value of the last state
   }
  if(!digitalRead(push)){
    delay(100);
    if(submenu == 0 && Ready==1) //Main Menu
    {    
       if(page==1)
       {
        submenu=1;
        counter=0;
        pushed=1;
        delay(100);
        goto Start;
       }
    
       if(page==2)
       {
        submenu=2;
        counter=0;
        pushed=1;
        delay(100);
        goto Start;
       }
    
       if(page==3)
       {
        submenu=0;
        counter=0;
        last_counter=0;
        pushed=0;
        menu =0;
        goto Start;
       }
    }//end of submenu 0
    if(submenu == 1) //Current Settings
    {
      do
      {
        if(page==3)
          {
            submenu =0;
            delay(DL);
            counter=0;
            pushed=0;
            Ready=0;
            goto Start;
          }
        switch (page)
      {
        case 1:
          lcd.setCursor(14,0);
          lcd.write(1);
          //a1_set_raw = analogRead(A0);
          set_current[0]=a1_set_raw*2.000/1000;
          lcd.print(a1_set_raw*2.000/1000,3);
          Card1_dac_current.setOutputLevel(uint16_t (set_current[0]*4096/2.048));
          break;
        case 2:
          lcd.setCursor(14,1);
          lcd.write(1);
          a2_set_raw = analogRead(A1);
          set_current[1]=a2_set_raw*2.000/1000;
          lcd.print(a2_set_raw*2.000/1000,3);
          Card2_dac_current.setOutputLevel(uint16_t (set_current[0]*4096/2.048));
          break;
        default:
       
          break;
      }
      delay(100);
       if(!digitalRead(push)) //button on encoder to be pressed to break the loop 
          {
            delay(5);
            counter=0;
            pushed=0;
            Ready=0;
            goto Start;
          }
      } while (1);
    }//end of submenu 1
    if(submenu == 2) //Voltage settings
    { 
      do
      {
        if(page==3)
          {
            submenu =0;
            page = 1;
            delay(DL);
            counter=0;
            pushed=0;
            Ready=0;
            goto Start;
          }
      switch (page)
      {
        case 1:
          lcd.setCursor(14,0);
          lcd.write(1);
          v1_set_raw = analogRead(A2);
          set_volt[0]=v1_set_raw*20.000/1000;
          lcd.print(v1_set_raw*20.000/1000,3);
          Card1_dac_volt.setOutputLevel(uint16_t (set_volt[0]*4096/20.48));
          break;
        case 2:
          lcd.setCursor(14,1);
          lcd.write(1);
          v2_set_raw = analogRead(A3);
          set_volt[1]=v2_set_raw*20.000/1000;
          lcd.print(v2_set_raw*20.000/1000,3);
          Card2_dac_volt.setOutputLevel(uint16_t (set_volt[0]*4096/20.48));
          break;
        default:
          break;
      }
      delay(100);
       if(!digitalRead(push))
          {
            delay(DL);
            counter=0;
            pushed=0;
            Ready=0;
            goto Start;
          }
      }while(1);
      if(page==3)
       {
        submenu=0;
        counter=0;
        pushed=0;
        Ready=0;
        delay(DL);
       }
    }//end of submenu 2
    if(submenu == 3) //Calibration
    {    
       if(page==1)
       {
        submenu=6;
        counter=0;
        pushed=0;
        Ready=0; 
        delay(DL);
        goto Start;
       }
    
       if(page==2)
       {
        submenu=7;
        counter=0;
        pushed=0;
        Ready=0;
        delay(DL);
        goto Start;
       }
       if(page==3)
       {
        submenu=0;
        counter=0;
        pushed=0;
        Ready=0;
        delay(DL);
       }
    }//end of submenu 3

  }
  
  //Add limit for the counter. Each line of the menu has 2 points. Since my menu has 5 lines the maximum counter will be from 0 to 10
  if(counter > max_cnt) counter=0;
  if(counter < 0) counter=max_cnt;  
}//end void

// Interrupt on A changing state
void doEncoderA() {
  // debounce 
  if ( rotating ) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set )
      counter += 1;
    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      counter -= 1;
    rotating = false; // no more debouncing until loop() hits again
  }
}

void Summaryscreen(){
        //Printing fix portion of screen
        lcd.clear();
        lcd.setCursor(1,0); lcd.print("V1="); lcd.setCursor(11,0); lcd.print("A1=");
        lcd.setCursor(1,1); lcd.print("Vi="); lcd.setCursor(11,1); lcd.print("T1=");
        lcd.setCursor(1,2); lcd.print("V2="); lcd.setCursor(11,2); lcd.print("A2=");
        lcd.setCursor(1,3); lcd.print("Vi="); lcd.setCursor(11,3); lcd.print("T2=");
        //Printing Variable portion of screen 
        do {
        //Reading from ADC of Card1
        Card1_ADC.startConversion(MCP342X_CHANNEL_1);
        Card1_ADC.getResult(&vout1_adc);
        vout1 = vout1_adc*20.480/32767;
        Card1_ADC.startConversion(MCP342X_CHANNEL_2);
        Card1_ADC.getResult(&current1_adc);
        current1 = current1_adc*2.048/32767;
        Card1_ADC.startConversion(MCP342X_CHANNEL_3);
        Card1_ADC.getResult(&vin1_adc);
        vin1 = vin1_adc*20.480*4/32767;
        Card1_ADC.startConversion(MCP342X_CHANNEL_4);
        Card1_ADC.getResult(&T1_adc);
        T1 = ((T1_adc*2.048/32767)-0.5)*100;

        //Reading from ADC of Card2
        Card2_ADC.startConversion(MCP342X_CHANNEL_1);
        Card2_ADC.getResult(&vout2_adc);
        vout2 = vout2_adc*20.480/32767;
        Card2_ADC.startConversion(MCP342X_CHANNEL_2);
        Card2_ADC.getResult(&current2_adc);
        current2 = current2_adc*2.048/32767;
        Card2_ADC.startConversion(MCP342X_CHANNEL_3);
        Card2_ADC.getResult(&vin2_adc);
        vin2 = vin2_adc*20.480*4/32767;
        Card2_ADC.startConversion(MCP342X_CHANNEL_4);
        Card2_ADC.getResult(&T2_adc);
        T2 = ((T2_adc*2.048/32767)-0.5)*100;
        
        //Printing all data 
        lcd.setCursor(4,0);lcd.print(vout1,3);lcd.setCursor(14,0);lcd.print(current1,3);lcd.print(" ");
        lcd.setCursor(4,1);lcd.print(vin1,2);lcd.setCursor(14,1);lcd.print(T1,1);lcd.print(" ");
        lcd.setCursor(4,2);lcd.print(vout2,3);lcd.setCursor(14,2);lcd.print(current2,3);lcd.print(" ");
        lcd.setCursor(4,3);lcd.print(vin2,2);lcd.setCursor(14,3);lcd.print(T2,1);lcd.print(" ");
      
        //set_volt[0]=analogRead(A2)*20.000/1000;
        //set_current[0]=analogRead(A0)*2.000/1000;
        //Card1_dac_volt.setOutputLevel(uint16_t (set_volt[0]*4096/20.48));
        //Card1_dac_current.setOutputLevel(uint16_t (set_current[0]*4096/2.048));
        //delay(100);
        if((last_counter > counter) || (last_counter < counter)  || pushed)
        {
          last_counter = counter;
          counter = 0;
          menu = 1;
          return;
        }
      } while(1);
}