#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <config.h>
#include <powercard.h>

powercard card1 = powercard(0x60, 0x61, 0x68, 0x00);
powercard card2 = powercard(0x62, 0x63, 0x6F, 0x00);

void setup() 
{
  //encoder configuration
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // turn on pullup resistors
  digitalWrite(encoderPinB, HIGH); // turn on pullup resistors
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  pinMode (push,INPUT); //Define the pin as input & OUTPUT
  //end of encoder configuration
  Serial.begin(9600);
  
  //Read calibration data from EEPROM
  int EEPROM_add = 0;
  for(i=0;i<4;i++)
  { 
    for (j=0;j<4;j++)
    {
      EEPROM.get(EEPROM_add,cal[i][j]);
      if (cal[i][j] >20 || cal[i][j] <0) cal[i][j] = 10;
      EEPROM_add += sizeof(cal[i][j]);
    }
  }

  //LCD configuration
  lcd.begin();
  lcd.backlight(); 
  lcd.createChar(1, arrow);
  //end of encoder configuration

  //configuration of power card
  //Initialize card1
  card1.begin();
  // Initialize card2
  card2.begin();
  card1.write(set_current[0],1);
  card1.write(set_volt[0],0);
  card2.write(set_current[1],1);
  card2.write(set_volt[1],0);
  //end of initialization
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
      max_cnt=3;
      if(0 <= counter && counter < 1)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);  
        lcd.print("Current settings");
        lcd.setCursor(0,1);  
        lcd.print(" Voltage settings");
        lcd.setCursor(0,2);
        lcd.print(" Calibration");
        lcd.setCursor(1,3);
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
        lcd.setCursor(0,2);
        lcd.print(" Calibration");
        lcd.setCursor(1,3);
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
        lcd.print("Calibration");
        lcd.setCursor(1,3);
        lcd.print(Back);
        pushed =0;
        page=3;
      }
      if(3 <= counter && counter < 4)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" Current settings");
        lcd.setCursor(0,1);
        lcd.print(" Voltage settings");
        lcd.setCursor(0,2);
        lcd.print(" Calibration");
        lcd.setCursor(0,3);
        lcd.write(1);
        lcd.print(Back);
        pushed =0;
        page=4;
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

    if(submenu == 3) //calibration Menu
    { 
      reset_timer= millis ();
      max_cnt=3;
      if(0 <= counter && counter < 1)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);  
        lcd.print("V1_ref_low =");
        lcd.setCursor(0,1);  
        lcd.print(" V1_ref_high=");
        lcd.setCursor(0,2);
        lcd.print(" V1_raw_low =");
        lcd.setCursor(0,3);
        lcd.print(" V1_raw_high=");
        pushed =0;
        page=1;
      }
      if(1 <= counter && counter < 2)
      {
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" V1_ref_low =");
        lcd.setCursor(0,1);
        lcd.write(1);    
        lcd.print("V1_ref_high=");
        lcd.setCursor(0,2);
        lcd.print(" V1_raw_low =");
        lcd.setCursor(0,3);
        lcd.print(" V1_raw_high=");
        pushed = 0;
        page=2;
      }
      if(2 <= counter && counter < 3)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" V1_ref_low =");
        lcd.setCursor(0,1);
        lcd.print(" V1_ref_high=");
        lcd.setCursor(0,2);
        lcd.write(1);
        lcd.print("V1_raw_low =");
        lcd.setCursor(0,3);
        lcd.print(" V1_raw_high=");
        pushed =0;
        page=3;
      }
      if(3 <= counter && counter < 4)
      {
        lcd.clear();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" V1_ref_low =");
        lcd.setCursor(0,1);
        lcd.print(" V1_ref_high=");
        lcd.setCursor(0,2);
        lcd.print(" V1_raw_low =");
        lcd.setCursor(0,3);
        lcd.write(1);
        lcd.print("V1_raw_high=");
        pushed =0;
        page=4;
      }
      if(page <=4)
      {
        lcd.setCursor(14,0);
        lcd.print(cal[0][0],3);
        lcd.setCursor(14,1);
        lcd.print(cal[0][1],3);
        lcd.setCursor(14,2);
        lcd.print(cal[0][2],3);
        lcd.setCursor(14,3);
        lcd.print(cal[0][3],3);
        delay(10);
      }
      delay(100);
    }//submenu = 3;

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
        submenu=3;
        counter=0;
        pushed=1;
        delay(100);
        goto Start;
       }

       if(page==4)
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
              card1.write(set_current[COL],1);
              break;
            case 2:
              card2.write(set_current[COL],1);
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
            vout1 = card1.read(0);
            lcd.setCursor(15,2);
            if(calibration(vout1,1)<10.0) lcd.print(calibration(vout1,1),3);
            if(calibration(vout1,1)>=10.0) lcd.print(calibration(vout1,1),2);
          }
          if(page==2)
          {
            vout2 = card2.read(0);
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
              card1.write(set_volt[COL],0);
              break;
            case 2:
              card2.write(set_volt[COL],0);
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
    if(submenu == 3) //Calibration menu
    { 
      counter = 0;
      lastctr = 0;
       if(page==1 || page==2 || page==3 || page==4 )
       {
         int COL = 0;
         int ROW = 0; //lcd Row
         int ROW1 = 0; //array row
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
            if (counter > lastctr) cal[ROW1][COL] = cal[ROW1][COL]+delta;
            if (counter < lastctr) cal[ROW1][COL] = cal[ROW1][COL]-delta;
            if (cal[ROW1][COL]>20) cal[ROW1][COL]= 20;
            if (cal[ROW1][COL]<0) cal[ROW1][COL]= 0;
          }
          lcd.setCursor(13,ROW);
          lcd.write(1);
          lcd.print(cal[ROW1][COL],3);
          lastctr = counter;
          //delay(100); //important for debouncing of encoder push switch
          char customkey1 = Customkeypad.getKey();
          if(customkey1 == '<') delta=delta*10;
          if(customkey1 == '>') delta=delta/10;
          if(customkey1 == '1') ROW1 = 0;
          if(customkey1 == '2') ROW1 = 1;
          if(customkey1 == '3') ROW1 = 2;
          if(customkey1 == '4') ROW1 = 3;
          if(customkey1 == 'D'){ 
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(13,i);
              lcd.print(' ');
            }
            ROW++;
            COL++;
            if (ROW > 3) ROW = 0;
            if (COL > 3) COL = 0;
          }
          if(customkey1 == 'U'){ 
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(13,i);
              lcd.print(' ');
            }
            ROW--;
            COL--;
            if (ROW < 0) ROW = 3;
            if (COL < 0) COL = 3;
          }
          if(customkey1 == '<'){
            ROW1--;
            if (ROW1 < 0) ROW1 = 3;
          }
          if(customkey1 == '>'){
            ROW1++;
            if (ROW1 > 3) ROW1 = 0;
          }
          if(customkey1 == 'E'){
            last_counter = 0;
            counter=0;
            pushed=0;
            Ready=0;
            submenu =0;
            keypadEntry = 0;
            menu =0;
            goto Start;
          }
          if(customkey1 == 'C'){
            int EEPROM_add = 0;
            for(i=0;i<4;i++)
            { 
              for (j=0;j<4;j++)
              {
                EEPROM.put(EEPROM_add, cal[i][j]);
                EEPROM_add += sizeof(cal[i][j]);
              }
            }
            goto Start;
          } 
          switch (ROW1)
          {
          case 0:
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(1,i);
              lcd.print("V1");
            }
            break;
          case 1:
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(1,i);
              lcd.print("V2");
            }
            break;
          case 2:
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(1,i);
              lcd.print("A1");
            }
            break;
          case 3:
            for (int i = 0; i < 4; i++)
            {
              lcd.setCursor(1,i);
              lcd.print("A2");
            }
            break;
          default:
            break;
          }
          lcd.setCursor(14,0);
          lcd.print(cal[ROW1][0],3);
          lcd.setCursor(14,1);
          lcd.print(cal[ROW1][1],3);
          lcd.setCursor(14,2);
          lcd.print(cal[ROW1][2],3);
          lcd.setCursor(14,3);
          lcd.print(cal[ROW1][3],3);
          delay(10);
         }while(1);
       }
    }//end of submenu 3
  }

  if (millis() - reset_timer > summary_delay)
  {
    menu = 0;
  }

  if(Customkeypad.getKey() == 'E')
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
        char customkey = Customkeypad.getKey();
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
          lcd.clear();
          lcd.setCursor(1,1);
          lcd.print ("Entering calibration..");
          do
          {
            switch (Customkeypad.getKey())
            {
            case '1':
              counter = 0;
              last_counter = 0;
              submenu = 3;
              pushed = 1;
              menu = 1;
              return;
              break;
            default:
              break;
            }
          } while (1);
          
          break;

        default:
          break;
        } 
        
        //Reading from ADC of Card2
        vout1 = card1.read(0);
        current1 = card1.read(1);
        vin1 = card1.read(2);
        T1 = card1.read(3);

        //Reading from ADC of Card2
        vout2 = card2.read(0);
        current2 = card2.read(1);
        vin2 = card2.read(2);
        T2 = card2.read(3);

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
    corrected_value = ((((rawvalue - cal[0][2]) * (cal[0][1] - cal[0][0])) / (cal[0][3] - cal[0][2])) + cal[0][0]);
    return corrected_value;
    break;
  case 2:
    corrected_value = ((((rawvalue - cal[1][2]) * (cal[1][1] - cal[1][0])) / (cal[1][3] - cal[1][2])) + cal[1][0]);
    return corrected_value;
    break;
  case 3:
    corrected_value = ((((rawvalue - cal[2][2]) * (cal[2][1] - cal[2][0])) / (cal[2][3] - cal[2][2])) + cal[2][0]);
    return corrected_value;
    break;
  case 4:
    corrected_value = ((((rawvalue - cal[3][2]) * (cal[3][1] - cal[3][0])) / (cal[3][3] - cal[3][2])) + cal[3][0]);
    return corrected_value;
    break;
  default:
    return rawvalue;
    break;
  }
}