/* This is the first attempt at bringing the Arduino Mega and Multimoto to life.
   The goal is to power the actuator through multimoto & relay
   The essense is to power multimoto M1 and fire Relay IN1
   Erling Roberts 3-14-20
   The foundation of this software is Progressive Automation for Mulitmoto

     Example code to control up to 4 actuators,using the Robot Power MultiMoto driver.
   Hardware:
    - Robot Power MultiMoto
    - Arduino Uno

    Wiring:
  - Connect actuators to the M1, M2, M3, M4 connections on the MultiMoto board.
  - Connect the negative (black) to the right connection, positive (red) to the left.
  - Connect a 12 volt source (minimum 1A per motor if unloaded, 8A per motor if fully loaded)to the BAT terminals. Ensure that positive and negative are placed in the correct spots.

   Code modified by Progressive Automations from the example code provided by Robot Power
     <a href="http://www.robotpower.com/downloads/" rel="nofollow"> http://www.robotpower.com/downloads/</a>

    Robot Power MultiMoto v1.0 demo
    This software is released into the Public Domain
*/

// include the SPI library:

#include <SPI.h>
// L9958 slave select pins for SPI
#define SS_ORANGE 14
#define SS_GREEN  13
#define SS_BLUE   12
#define SS_RED    11
// L9958 DIRection pins
#define DIR_RED    2
#define DIR_BLUE   3
#define DIR_GREEN  4
#define DIR_ORANGE 7
// L9958 PWM pins
#define PWM_RED    9
#define PWM_BLUE  10     // Timer1
#define PWM_GREEN  5
#define PWM_ORANGE 6     // Timer0

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwmR, pwmB, pwmG, pwmO;
boolean dirR, dirB, dirG, dirO;

// Relay Setup

const unsigned int RED_P    = 22; //Relay control for relay 1
const unsigned int RED_N    = 24;
const unsigned int BLUE_P   = 26;
const unsigned int BLUE_N   = 28;
const unsigned int GREEN_P  = 30;
const unsigned int GREEN_N  = 32;
const unsigned int ORANGE_P = 34;
const unsigned int ORANGE_N = 36;

#define N_S HIGH
#define E_W LOW

void setup() {
  unsigned int configWord;

  // put your setup code here, to run once:
  pinMode(SS_RED, OUTPUT); digitalWrite(SS_RED, LOW);  // HIGH = not selected
  pinMode(SS_BLUE, OUTPUT); digitalWrite(SS_BLUE, LOW);
  pinMode(SS_GREEN, OUTPUT); digitalWrite(SS_GREEN, LOW);
  pinMode(SS_ORANGE, OUTPUT); digitalWrite(SS_ORANGE, LOW);

  // L9958 DIRection pins
  pinMode(DIR_RED, OUTPUT);
  pinMode(DIR_BLUE, OUTPUT);
  pinMode(DIR_GREEN, OUTPUT);
  pinMode(DIR_ORANGE, OUTPUT);

  // L9958 PWM pins
  pinMode(PWM_RED, OUTPUT);  digitalWrite(PWM_RED, LOW);
  pinMode(PWM_BLUE, OUTPUT);  digitalWrite(PWM_BLUE, LOW);    // Timer1
  pinMode(PWM_GREEN, OUTPUT);  digitalWrite(PWM_GREEN, LOW);
  pinMode(PWM_ORANGE, OUTPUT);  digitalWrite(PWM_ORANGE, LOW);    // Timer0

  // Set up Relays
  pinMode(RED_P, OUTPUT);
  pinMode(RED_N, OUTPUT);
  pinMode(BLUE_P, OUTPUT);
  pinMode(BLUE_N, OUTPUT);
  pinMode(GREEN_P, OUTPUT);
  pinMode(GREEN_N, OUTPUT);
  pinMode(ORANGE_P, OUTPUT);
  pinMode(ORANGE_N, OUTPUT);
  
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT); 
 digitalWrite(ENABLE_MOTORS, HIGH);  // HIGH = disabled

 /******* Set up L9958 chips *********
  ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate (1 enables slew limit, 0 disables)
  '9 - ISR - current slew rate (1 enables slew limit, 0 disables)
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;

  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

  // Motor 1
  digitalWrite(SS_RED, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_RED, HIGH);
  
 /* // Motor 2
  digitalWrite(SS_BLUE, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_BLUE, HIGH);
  // Motor 3
  digitalWrite(SS_GREEN, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_GREEN, HIGH);
  // Motor 4
  digitalWrite(SS_ORANGE, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_ORANGE, HIGH);
*/
  //Set initial actuator settings to pull at 0 speed for safety
  dirR = 0; dirB = 0; dirG = 0; dirO = 0; // Set direction
  pwmR = 0; pwmB = 0; pwmG = 0; pwmO = 0; // Set speed (0-255)

digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled
} // End setup

void loop() {
    dirR = 1;
    pwmR = 255; //set direction and speed 
    digitalWrite(RED_P, N_S);
    digitalWrite(RED_N, N_S);
    digitalWrite(DIR_RED, dirR);
    analogWrite(PWM_RED, pwmR); // write to pins

    dirB = 0;
    pwmB = 128;
    digitalWrite(DIR_BLUE, dirB);
    analogWrite(PWM_BLUE, pwmB);

    dirG = 1;
    pwmG = 255;
    digitalWrite(DIR_GREEN, dirG);
    analogWrite(PWM_GREEN, pwmG);

    dirO = 0;
    pwmO = 128;
    digitalWrite(DIR_ORANGE, dirO);
    analogWrite(PWM_ORANGE, pwmO);

    delay(5000); // wait once all four motors are set
    digitalWrite(RED_P, E_W);
    digitalWrite(RED_N, E_W);
    
    dirR = 0;
    pwmR = 128;
    digitalWrite(RED_P, N_S);
    digitalWrite(RED_N, N_S);
    digitalWrite(DIR_RED, dirR);
    analogWrite(PWM_RED, pwmR);
/* for later, correct variable names
    dirB = 1;
    pwmB = 255;
    digitalWrite(DIR_M2, dir2);
    analogWrite(PWM_M2, pwm2);

    dirG = 0;
    pwmG = 128;
    digitalWrite(DIR_M3, dir3);
    analogWrite(PWM_M3, pwm3);

    dirO = 1;
    pwmO = 255;
    digitalWrite(DIR_M4, dir4);
    analogWrite(PWM_M4, pwm4);   */

   delay(5000); 
   digitalWrite(RED_P, E_W);
   digitalWrite(RED_N, E_W);
   delay(5000);
   
}//end void loop}
