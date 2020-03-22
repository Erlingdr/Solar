/*  This is the first draft of the Solar Tracking Program
 *  The purpose of this software is to control the solar panels through a variety of conditions
 *  and protect the solar panels while also collecting a maximum amount of light
 *  The system needs protection from high winds and snow
 *  The system needs to track the sun based on time of year and time of day
 *  The system needs to accept some manual input to allow for maintenance of various types
 *  Erling Roberts 3/14/20 (with help from various sources)
 */

// Define a variety of constants and variables
#include <SPI.h>  //multimoto control
#include <Wire.h> //I2C, accelerometers, screen
#include <DFRobot_LIS2DH12.h> //accelerometers


//----------------Set Pins to define schematic-------------------
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

// Relay Setup
const short RED_P    = 22; //Relay control for relay 1
const short RED_N    = 24;
const short BLUE_P   = 26;
const short BLUE_N   = 28;
const short GREEN_P  = 30;
const short GREEN_N  = 32;
const short ORANGE_P = 34;
const short ORANGE_N = 36;

//Setup the relay feedback pin
const short REDN_Feed = 23;
const short REDE_Feed = 25;
const short BLUEN_Feed = 27;
const short BLUEE_Feed = 29;
const short GREENN_Feed = 31;
const short GREENE_Feed = 33;
const short ORANGEN_Feed = 35;
const short ORANGEE_Feed = 37;

//Setup the Gravity sensors

//-------------------Class variables---------------------
    //Arrays for poles, actuators(Pole,NSEW), limits(Pole,NSEW)
    //Panel Position (lat and long)
class Actuator {
  public:
    short SlaveSelect_Pin;
    short Direction_Pin;
    short Relay_P;
    short Relay_N;
    short PWM_Pin;
    short Feed_Pin;
    bool RelayCMD;
    int MinLimit;
    int LimitMax;
    int act_posit;
    // Construcor
    Actuator() {
      MinLimit = -10;
      LimitMax = 10;
    }
    void populateActuator(short ss, short dir, short rp, short rn,short pp, short feed, bool cmd) {
      SlaveSelect_Pin = ss;
      Direction_Pin = dir;
      Relay_P = rp;
      Relay_N = rn;
      PWM_Pin = pp;
      RelayCMD = cmd;
    } 
    void extend(int moveDirection, int counts) {
      //loop so that you never exceed the limits    
        if ((act_posit > LimitMax) or (act_posit < MinLimit)) {
          counts = 0;
                           Serial.println("Wrong side of limits");
                           Serial.print(act_posit);
                           Serial.print(";");
                           Serial.print(MinLimit);
                           Serial.print(";");
                           Serial.println(LimitMax);

        } else {
        //drive the actuator in the direction for counts       
         int pwmR = 255; //set direction and speed 
                  Serial.println("ready to run");
         digitalWrite(Relay_P, RelayCMD);
         digitalWrite(Relay_N, RelayCMD);
         digitalWrite(Direction_Pin, moveDirection);
         analogWrite(PWM_Pin, pwmR); // write to pins
         while (counts > 0) {
          //change counts until counts is 0
          delay(50);
          counts = counts - 50;
         }
         analogWrite(PWM_Pin, 0); //stop motor
         digitalWrite(Relay_P, LOW);
         digitalWrite(Relay_N, LOW);
    } }
    void setMinLimit(int limit) {
      MinLimit = limit;
    }
    void setLimitMax(int limit) {
      LimitMax = limit;
    }
    void setActPosit(int pos) {
      act_posit = pos;
    }
};
class Pole {
  public:
    Actuator North_South;
    Actuator East_West;
    int RotationError;
    double NSangle;
    double EWangle;
    // Constructor
    Pole(short SSt_Pin, short Dir_Pin, short R_P, short R_N, short PWM_Pin, short feed_NPin, short feed_EPin) {
      North_South.populateActuator(SSt_Pin, Dir_Pin, R_P, R_N,PWM_Pin, feed_NPin, HIGH);
      East_West.populateActuator(SSt_Pin, Dir_Pin, R_P, R_N,PWM_Pin, feed_EPin, LOW);
      pinMode(SSt_Pin, OUTPUT); digitalWrite(SSt_Pin, LOW);  // HIGH = not selected
      pinMode(Dir_Pin, OUTPUT);
      pinMode(PWM_Pin, OUTPUT);  digitalWrite(PWM_Pin, LOW);
      pinMode(R_P, OUTPUT);
      pinMode(R_N, OUTPUT);
      pinMode(feed_NPin, INPUT);
      pinMode(feed_EPin, INPUT);     

    }
    void getVector(int x, int y, int z) {
 //     NSangle = arctan(y/(sqrt(sq(x) + sq(z))));
 //     EWangle = arctan(x/(sqrt(sq(y) + sq(z))));
    }
    void setRotationError(int error) {
      RotationError = error;
    }
    void setupMotoshield(unsigned int configWord) {
      digitalWrite(North_South.SlaveSelect_Pin, LOW);
      SPI.transfer(lowByte(configWord));
      SPI.transfer(highByte(configWord));
      digitalWrite(North_South.SlaveSelect_Pin, HIGH);
    }

};
 
    //sunrise and sunset times (equation?)
    //sun position
    //Logger functions? ambient light, temp, position
//----------------Other initialization variables
  //Program logic variables
    //Daylight status (wakeup, daytime, gotosleep, nighttime)
    
//-----------------Initialize system-------------------------- 
void setup() {
    Wire.begin();
    Serial.begin(115200);
    while(!Serial);
    delay(100);
             Serial.println("Pole class defined");

// Initialize the various components in the system
  //Set operating limits by input or 'by example'
  //set the time (zulu?)
  Pole Red(SS_RED, DIR_RED, RED_P, RED_N, PWM_RED, REDN_Feed, REDE_Feed);
  Pole Blue(SS_BLUE, DIR_BLUE, BLUE_P, BLUE_N, PWM_BLUE, BLUEN_Feed, BLUEE_Feed);
  Pole Green(SS_GREEN, DIR_GREEN, GREEN_P, GREEN_N, PWM_GREEN, GREENN_Feed, GREENE_Feed);
  Pole Orange(SS_ORANGE, DIR_ORANGE, ORANGE_P, ORANGE_N, PWM_ORANGE, ORANGEN_Feed, ORANGEE_Feed);
    unsigned int configWord;

  // Multimoto - L9958 Enable for all 4 motors
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

  Red.setupMotoshield(configWord);
  Blue.setupMotoshield(configWord);
  Green.setupMotoshield(configWord);
  Orange.setupMotoshield(configWord);
  digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled

  //test software
    Serial.println("Start of test");
  Red.North_South.setActPosit(2);
  Red.North_South.extend(1, 1000);
  delay(2000);

}

//-----------------------Tracker Program-----------------------
void loop() {
// Loop the main software to operate and protect the system
  // Check for some manual override state
    //Override can go to safe position
    //Override may allow manual operation of one or more poles
    //Set operating limits
    //Error position for BLUE pole to say "Check me"
  // Check for High Winds, go to safe
    //have some timer so return from safe after 15 min in day or till morning
  // Check for daytime (vs no action nighttime)
    //time to wake up? (ambient light input?)
      //if cold, dump snow, else, 
        //wake up position
        //check NS position for date and adjust
        //set daylightstatus daytime
    //daytime operation
      //Calculate time based position
        //adjust every 2 degrees?
        //is it sunset? (ambient, time)
    //go to sleep operation
      //go to best position (neutral for EW)
      //uncritical error - orange pole flat
      //Record day's activity?
    //nighttime
      //monitor for bad stuff

// Some key operations 
   // Go to safe position for all poles, calibrate
   // Snow Dump Operation
   // Show off dance


          
 // Basic Operations
   //Move panel(Pole, actuator, direction, time, Limit)
   //Achieve target position(Pole, actuator, goal, Limit)
   //Move all panels(actuator, direction, time, Limit())
   //Achieve target position(actuator, goal, Limit())
   //Calibrate position(pole actuator, gravity, position)
   //set operating limits(pole, actuator, position, max_or_min, Limit)
}
