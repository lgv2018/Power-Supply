//Defination of I2C devices
LiquidCrystal_I2C lcd(0x27,20,4);


//Variables for LCD
String voltage_h = "Volt(H):";
String voltage_L = "Volt(L):";
String R1 = "R1=";
String R2 = "R2=";
String Back = "Back";
String Current_settings = "Current settings";
String Voltage_settings = "Voltage settings";
String calibration_str = "Calibration";

float vout1 = 0.000, vin1 =0.000, current1 = 0.000, T1 =0.000;
float vout2 = 0.000, vin2 =0.000, current2 = 0.000, T2 =0.000;
uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};
float delta =0.10;
float max_volt = 20.48 , min_volt = 0.00, max_current = 2.048, min_current = 0.000;
float set_current[2] = {0.1,0.1}, set_volt[2] = {3.000,3.000} ;   //Current settings variable
float cal[4][4];  /* ={{6.103, 11.083, 6.082, 11.043},  //V1 {ref_low, ref_high, raw_low, raw_high}
                  {6.000, 12.000, 6.000, 12.000},  //V2 {ref_low, ref_high, raw_low, raw_high}
                  {6.000, 12.000, 6.000, 12.000},  //a1 {ref_low, ref_high, raw_low, raw_high}
                  {6.000, 12.000, 6.000, 12.000}}; */ //a2 {ref_low, ref_high, raw_low, raw_high}

//Variables for the menu encoder
int counter = 0; 
int page=1;
int Ready=1;
int submenu=0;
int menu=0;
int last_counter = 0;   
int pushed = 0;
int DL = 100; // Delay for action
float inc_r = 0.5;
int i;
int j;
int lastctr = 0;
int max_cnt =10;
int min_cnt =0;
volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;            // interrupt service routine vars
boolean B_set = false;            // interrupt service routine vars

//Pin assignments
#define push 4
#define encoderPinA 2
#define encoderPinB 3

ulong summary_delay =5000;
long reset_timer;

int keypadEntry = 0;

const byte ROWS = 5;
const byte COLUMNS =4;
byte rowPins[ROWS] = {13,12,11,10,9}; //connect to row pinout of the keypad
byte columnPins[COLUMNS] = {5,6,A0,A1}; //connect to column pinout of keypad
//Define symbols on keypad
char hexakeys [ROWS][COLUMNS] = {
  {'A','B','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','E'},
  {'<','0','>','C'}
};

//function
void doEncoderA();
void doEncoderB();
void Summaryscreen();
//Defination of the keypad
Keypad Customkeypad = Keypad(makeKeymap (hexakeys),rowPins,columnPins,ROWS, COLUMNS);
float calibration (float rawvalue, int param);