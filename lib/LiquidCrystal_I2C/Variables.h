#include <Arduino.h>

//Variables for the menu encoder
int page=1;
int Ready=1;
int submenu=0;
int last_counter = 0; 
bool clk_State;
bool Last_State; 
bool dt_State;  
int pushed = 0;
int DL = 500; // Delay for action
float inc_r = 0.10;
int i;

//Voltage divider Resistors
float R[2][4] = {{50.0,60.0,70.0,80.00},{10.0,20.0,30.0,40.0}};
float v_cal[2][4] = {{12.0,13.0,14.0,15.00},{5.0,6.0,7.0,8.0}};
unsigned int off_current[4] = {2500,2500,2500,2500};

String voltage_h = "Volt(H):";
String voltage_L = "Volt(L):";
String R1 = "R1=";
String R2 = "R2=";
String Back = "Back";
int lastctr = 0;

//The pin for the push button
#define push 10
#define CH1_SW 4
#define CH2_SW 5
#define CH3_SW 6
#define CH4_SW 7

//Declarations for Encoder
int counter = 0; 
enum PinAssignments { 
  encoderPinA = 2,   // right
  encoderPinB = 3,   // left
  clearButton = 8    // another two pins
};
volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;            // interrupt service routine vars
boolean B_set = false;            // interrupt service routine vars
void doEncoderA();
void doEncoderB();