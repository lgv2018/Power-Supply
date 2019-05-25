#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <MCP47X6.h>
#include <MCP342X.h>
#include <Keypad.h>


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

float set_current[2] = {1,1};   //Current settings variable
float set_volt[2] = {1,1};    //Current settings variable

//Variables for LCD
String voltage_h = "Volt(H):";
String voltage_L = "Volt(L):";
String R1 = "R1=";
String R2 = "R2=";
String Back = "Back";

//Pin assignments
#define push 4
#define encoderPinA 2
#define encoderPinB 3
#define Menu 5

long summary_delay =5000;
long reset_timer;

volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;            // interrupt service routine vars
boolean B_set = false;            // interrupt service routine vars
void doEncoderA();
void doEncoderB();
void Summaryscreen();

const byte ROWS = 5;
const byte COLUMNS =4;
//Define symbols on keypad
char hexakeys [ROWS][COLUMNS] = {
  {'A','B','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','*'},
  {'<','0','>','*'}
};

byte rowPins[ROWS] = {1,3,5,8,10}; //connect to row pinout of the keypad
byte columnPins[COLUMNS] = {2,4,6,9}; //connect to column pinout of keypad
//initialize an instance of class NewKeypad
Keypad Customkeypad = Keypad(makeKeymap (hexakeys),rowPins,columnPins,ROWS, COLUMNS);
int keypadEntry = 0;

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
      /* R[i][j]=EEPROM.read(EEPROM_add);
      if (R[i][j] >20 || R[i][j] <0) R[i][j] = 10;
      v_cal[i][j]=EEPROM.read(EEPROM_add+10);
      if (v_cal[i][j] >20 || v_cal[i][j] <0) v_cal[i][j] = 10;
      EEPROM_add++; */
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
      reset_timer= millis ();
      Serial.println(reset_timer);
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
        max_cnt=2;
        reset_timer= millis ();
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
          delay(10);
        }
        
    }//submenu = 1;

    if(submenu == 2) //Voltage settings
    {  
      max_cnt=2;
      reset_timer= millis ();
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
        delay(10);
      } 

    }//submenu = 2;

    last_counter = counter; //Save the value of the last state
   }
  if(!digitalRead(push) || keypadEntry){
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
      counter = 0;
      lastctr = 0;
       if(page==1 || page==2 )
       {
         int COL = 0;
         int ROW = 0;
         counter = 0;
         for(i=0; i <=3; i++)
         {
          if (page==(i+1))
          {
            COL =i;
            ROW =i;
          }
         }
         lcd.setCursor(0,ROW);
         lcd.print(" ");
         do
         {
          if(counter != lastctr)
          {
            if (counter > lastctr) set_current[COL] = set_current[COL]+0.01;
            if (counter < lastctr) set_current[COL] = set_current[COL]-0.01;
            if (set_current[COL]>2.048) set_current [COL]= 2.048;
            if (set_current[COL]<0) set_current [COL] = 0.0;
          }
          lcd.setCursor(14,ROW);
          lcd.write(1);
          switch (page)
          {
            case 1:
              Card1_dac_current.setOutputLevel(uint16_t (set_current[COL]*4096/2.048));
              break;
            case 2:
              Card2_dac_current.setOutputLevel(uint16_t (set_current[COL]*4096/2.048));
              break;
          }
          lcd.print(set_current[COL]);
          lastctr = counter;
          delay(100);
          if(!digitalRead(push))
          {
            page = ROW +1;
            delay(DL);
            counter=0;
            pushed=0;
            Ready=0;
            keypadEntry = 0;
            goto Start;
          }
         }while(1);
       }

       if(page==3)
       {
        submenu=0;
        counter=0;
        pushed=0;
        Ready=0;
        delay(DL);
       }
    }//end of submenu 1
    if(submenu == 2) //Voltage settings
    { 
      counter = 0;
      lastctr = 0;  
       if(page==1 || page==2 )
       {
         int COL = 0;
         int ROW = 0;
         counter = 0;
         for(i=0; i <=3; i++)
         {
          if (page==(i+1))
          {
            COL =i;
            ROW =i;
          }
         }
         lcd.setCursor(0,ROW);
         lcd.print(" ");
         do
         {
          if(counter != lastctr)
          {
            if (counter > lastctr) set_volt[COL] = set_volt[COL]+0.1;
            if (counter < lastctr) set_volt[COL] = set_volt[COL]-0.1;
            if (set_volt[COL]>20.48) set_volt[COL]= 20.48;
            if (set_volt[COL]<0) set_volt[COL] = 0.000;
          }
          lcd.setCursor(14,ROW);
          lcd.write(1);
          switch (page)
          {
            case 1:
              Card1_dac_volt.setOutputLevel(uint16_t (set_volt[COL]*4096/20.48));
              break;
            case 2:
              Card2_dac_volt.setOutputLevel(uint16_t (set_volt[COL]*4096/20.48));
              break;
          }
          lcd.print(set_volt[COL]);
          lastctr = counter;
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
       }

       if(page==3)
       {
        submenu=0;
        counter=0;
        pushed=0;
        Ready=0;
        delay(DL);
       }
    }//end of submenu 2
  }

    if (millis() - reset_timer > summary_delay)
  {
    menu = 0;
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
        char customkey =Customkeypad.getKey();
        switch (customkey)
        {
        case '1': // change current setting of the card 1
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =1; //jump to submenu 1 for currrent setting
          counter = 0; //Entry in page 1 of submenu 1
          page=1; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;
        
        case '2': // change current setting of the card 2
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =1; //jump to submenu 1 for current
          counter = 1; //Entry in page 2 of submenu 1
          page=2; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;

        case '4': // change volt setting of the card 1
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =2; //jump to submenu 2 for voltage setting
          counter = 0; //Entry in page 1 of submenu 1
          page=1; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;

        case '5': // change volt setting of the card 2
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =1; //jump to submenu 1
          counter = 1; //Entry in page 2 of submenu 1
          page=2; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;

        default:
          break;
        }
        
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
      
        if((last_counter > counter) || (last_counter < counter)  || pushed)
        {
          last_counter = counter;
          counter = 0;
          menu = 1;
          return;
        }
      } while(1);
}