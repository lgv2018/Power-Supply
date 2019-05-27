#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <MCP47X6.h>
#include <MCP342X.h>
#include <Keypad.h>
#include <config.h>

void setup() 
{
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // turn on pullup resistors
  digitalWrite(encoderPinB, HIGH); // turn on pullup resistors
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  Serial.begin(9600);
  pinMode (push,INPUT); //Define the pin as input & OUTPUT
  
  // Read EEPROM settings
  /* int EEPROM_add = 0;
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
 */
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

  Card1_dac_current.setOutputLevel(uint16_t (set_current[0]*DAC_bits/max_current));
  Card1_dac_volt.setOutputLevel(uint16_t (set_volt[0]*DAC_bits/max_volt));
  Card2_dac_current.setOutputLevel(uint16_t (set_current[1]*DAC_bits/max_current));
  Card2_dac_volt.setOutputLevel(uint16_t (set_volt[1]*DAC_bits/max_volt));
  
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
    menu=1;                                                                                                                                                                                                                          menu =1;
    Ready=1;
    if(submenu == 0) //Main Menu
    { 
      reset_timer= millis ();
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
        pushed =0;
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
        pushed = 0;
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
        pushed =0;
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
        if(set_volt[0]+2.0<10.0) lcd.print((set_volt[0]+2.0),3);
        if(set_volt[0]+2.0>=10.0) lcd.print((set_volt[0]+2.0),2);
        lcd.setCursor(15,1);
        if(set_volt[1]+2.0<10.0) lcd.print((set_volt[1]+2.0),3);
        if(set_volt[1]+2.0>=10.0) lcd.print((set_volt[1]+2.0),2);
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
      lcd.setCursor(8,3);
      lcd.print("Delta= " );
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
          if(delta>=10) delta =1;
          if(delta<0.001) delta =0.001;
          lcd.setCursor(15,3);
          lcd.print(delta,3); 
          if(counter != lastctr)
          {
            if (counter > lastctr) set_current[COL] = set_current[COL]+delta;
            if (counter < lastctr) set_current[COL] = set_current[COL]-delta;
            if (set_current[COL]>max_current) set_current [COL]= max_current;
            if (set_current[COL]<min_current) set_current [COL] =min_current;
          }
          lcd.setCursor(14,ROW);
          lcd.write(1);
          switch (page)
          {
            case 1:
              Card1_dac_current.setOutputLevel(uint16_t (set_current[COL]*DAC_bits/max_current));
              break;
            case 2:
              Card2_dac_current.setOutputLevel(uint16_t (set_current[COL]*DAC_bits/max_current));
              break;
          }
          lcd.print(set_current[COL],3);
          lastctr = counter;
          delay(100); //important for debouncing of encoder push switch
          char customkey1 =Customkeypad.getKey();
          if(customkey1 == '<') delta=delta*10;
          if(customkey1 == '>') delta=delta/10;
          if(customkey1 == 'E')
          {
            last_counter = 0;
            counter=0;
            pushed=0;
            Ready=0;
            keypadEntry = 0;
            submenu = 0;
            menu =0;
            goto Start;
          }
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
      lcd.setCursor(9,2);
      lcd.print("Vact= " );  
      lcd.setCursor(8,3);
      lcd.print("Delta= " );
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
          if(page==1)
          {
            Card1_ADC.startConversion(MCP342X_CHANNEL_1);
            Card1_ADC.getResult(&vout1_adc);
            vout1 = vout1_adc*max_volt/ADC_bits;
            lcd.setCursor(15,2);
            if(calibration(vout1,1)<10.0) lcd.print(calibration(vout1,1),3);
            if(calibration(vout1,1)>=10.0) {lcd.print(calibration(vout1,1),2);
            Serial.print(calibration(vout1,1),3);
            Serial.print("\n");}
          }
          if(page==2)
          {
            Card2_ADC.startConversion(MCP342X_CHANNEL_1);
            Card2_ADC.getResult(&vout2_adc);
            vout2 = vout2_adc*max_volt/ADC_bits;
            lcd.setCursor(15,2);
            if(calibration(vout2,2)<10.0) lcd.print(calibration(vout2,2),3);
            if(calibration(vout2,2)>=10.0) lcd.print(calibration(vout2,2),2);
          }
          if(delta>=10) delta =1;
          if(delta<0.001) delta =0.001;
          lcd.setCursor(15,3);
          lcd.print(delta,3); 
          if(counter != lastctr)
          {
            if (counter > lastctr) set_volt[COL] = set_volt[COL]+delta;
            if (counter < lastctr) set_volt[COL] = set_volt[COL]-delta;
            if (set_volt[COL]>max_volt) set_volt[COL]= max_volt;
            if (set_volt[COL]<min_volt) set_volt[COL]= min_volt;
          }
          lcd.setCursor(14,ROW);
          lcd.write(1);
          switch (page)
          {
            case 1:
              Card1_dac_volt.setOutputLevel(uint16_t (set_volt[COL]*DAC_bits/max_volt));
              break;
            case 2:
              Card2_dac_volt.setOutputLevel(uint16_t (set_volt[COL]*DAC_bits/max_volt));
              break;
          }
          if(set_volt[COL]+2.0<10.0) lcd.print((set_volt[COL]+2.0),3);
          if(set_volt[COL]+2.0>=10.0) lcd.print((set_volt[COL]+2.0),2);
          lastctr = counter;
          delay(100); //important for debouncing of encoder push switch
          char customkey1 =Customkeypad.getKey();
          if(customkey1 == '<') delta=delta*10;
          if(customkey1 == '>') delta=delta/10;
          if(customkey1 == 'E')
          {
            last_counter = 0;
            counter=0;
            pushed=0;
            Ready=0;
            submenu =0;
            keypadEntry = 0;
            menu =0;
            goto Start;
          }
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
        //Serial.print(customkey);
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

        case '4': // change volt setting of the card 2
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =2; //jump to submenu 2 for voltage setting
          counter = 0; //Entry in page 1 of submenu 2
          page=1; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;
  
        case '5': // change volt setting of the card 2
          menu = 1; // to prevent progam entering summary screen screen
          pushed = 1; //Entry in to menu
          submenu =2; //jump to submenu 2 for voltage setting
          counter = 1; //Entry in page 2 of submenu 2
          page=2; //required for the pushed button function
          keypadEntry = 1; //forces entry for pushe button function
          return;
          break;
        case '*':
          //Card1_dac_volt.setOutputLevel(uint16_t ((set_volt[0] + (set_volt[0]+2.0-(vout1+cal_volt1)))*DAC_bits/max_volt));
          break;

        default:
          break;
        } 
        
        Card1_ADC.startConversion(MCP342X_CHANNEL_1);
        Card1_ADC.getResult(&vout1_adc);
        vout1 = vout1_adc*max_volt/ADC_bits;
        Card1_ADC.startConversion(MCP342X_CHANNEL_2);
        Card1_ADC.getResult(&current1_adc);
        current1 = current1_adc*max_current/ADC_bits;
        Card1_ADC.startConversion(MCP342X_CHANNEL_3);
        Card1_ADC.getResult(&vin1_adc);
        vin1 = vin1_adc*max_volt*4/ADC_bits;
        Card1_ADC.startConversion(MCP342X_CHANNEL_4);
        Card1_ADC.getResult(&T1_adc);
        T1 = ((T1_adc*max_current/ADC_bits)-0.5)*100;

        //Reading from ADC of Card2
        Card2_ADC.startConversion(MCP342X_CHANNEL_1);
        Card2_ADC.getResult(&vout2_adc);
        vout2 = vout2_adc*max_volt/ADC_bits;
        Card2_ADC.startConversion(MCP342X_CHANNEL_2);
        Card2_ADC.getResult(&current2_adc);
        current2 = current2_adc*max_current/ADC_bits;
        Card2_ADC.startConversion(MCP342X_CHANNEL_3);
        Card2_ADC.getResult(&vin2_adc);
        vin2 = vin2_adc*max_volt*4/ADC_bits;
        Card2_ADC.startConversion(MCP342X_CHANNEL_4);
        Card2_ADC.getResult(&T2_adc);
        T2 = ((T2_adc*max_current/ADC_bits)-0.5)*100;
        
        //Printing all data 
        lcd.setCursor(4,0);lcd.print(calibration(vout1,1),3);lcd.setCursor(14,0);lcd.print(calibration(current1,3),3);lcd.print(" ");
        lcd.setCursor(4,1);lcd.print(vin1,2);lcd.setCursor(14,1);lcd.print(T1,1);lcd.print(" ");
        lcd.setCursor(4,2);lcd.print(calibration(vout2,2),3);lcd.setCursor(14,2);lcd.print(calibration(current2,4),3);lcd.print(" ");
        lcd.setCursor(4,3);lcd.print(vin2,2);lcd.setCursor(14,3);lcd.print(T2,1);lcd.print(" ");
        if((last_counter > counter) || (last_counter < counter))
        {
          last_counter = counter;
          counter = 0;
          pushed = 1;
          menu = 1;
          return;
        }
      } while(1);
}

float calibration (float rawvalue, int param){
  
  float corrected_value;
  switch (param)
  {
  case 1:
    //corrected_value = rawvalue;
    corrected_value = ((((rawvalue - v1_raw_low) * (v1_ref_high - v1_ref_low )) / (v1_raw_high - v1_raw_low) ) + v1_ref_low);
    return corrected_value;
    break;
  case 2:
    corrected_value = ((((rawvalue - v2_raw_low) * (v2_ref_high - v2_ref_low )) / (v2_raw_high - v2_raw_low) ) + v2_ref_low);
    return corrected_value;
    break;
  case 3:
    corrected_value = ((((rawvalue - a1_raw_low) * (a1_ref_high - a1_ref_low )) / (a1_raw_high - a1_raw_low) ) + a1_ref_low);
    return corrected_value;
    break;
  case 4:
    corrected_value = ((((rawvalue - a2_raw_low) * (a2_ref_high - a2_ref_low )) / (a2_raw_high - a2_raw_low) ) + a2_ref_low);
    return corrected_value;
    break;
  default:
    return rawvalue;
    break;
  }

}