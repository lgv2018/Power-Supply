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
int DAC_bits = 4095;
int ADC_bits = 32767;

static int16_t vout1_adc, vin1_adc, current1_adc, T1_adc;
static int16_t vout2_adc, vin2_adc, current2_adc, T2_adc;
int v1_set_raw, v2_set_raw, a1_set_raw, a2_set_raw;
uint16_t v1_set, v2_set, a1_set, a2_set;
float vout1 = 0.000, vin1 =0.000, current1 = 0.000, T1 =0.000;
float vout2 = 0.000, vin2 =0.000, current2 = 0.000, T2 =0.000;
uint8_t arrow[8] = {0x0, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00, 0x00};
uint8_t ON[8] = {0x18,0x1C,0x1E,0x1F,0x1E,0x1C,0x18,0x00};
uint8_t OFF[8] = {0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00};
float delta =0.10;
float max_volt = 20.48;
float min_volt = 0.00;
float max_current = 2.048;
float min_current = 0.000;
float set_current[2] = {0.1,0.1};   //Current settings variable
float set_volt[2] = {6.000,5.000};    //Current settings variable
float cal_volt1 = 0.032;
float cal_volt2 = 0;
float cal_cur1 = 0;
float cal_cur2 = 0;

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

//Variables for LCD
String voltage_h = "Volt(H):";
String voltage_L = "Volt(L):";
String R1 = "R1=";
String R2 = "R2=";
String Back = "Back";
bool cursor_flag = LOW;

//Pin assignments
#define push 4
#define encoderPinA 2
#define encoderPinB 3

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
  {'7','8','9','E'},
  {'<','0','>','C'}
};

byte rowPins[ROWS] = {13,12,11,10,9}; //connect to row pinout of the keypad
byte columnPins[COLUMNS] = {5,6,A0,A1}; //connect to column pinout of keypad
//initialize an instance of class NewKeypad
Keypad Customkeypad = Keypad(makeKeymap (hexakeys),rowPins,columnPins,ROWS, COLUMNS);
int keypadEntry = 0;
