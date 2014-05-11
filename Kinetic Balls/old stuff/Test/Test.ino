/*
 * Kinetic Balls -- David Henshaw, December 2012-
 * 
 * v8 - Multiple changes around unit motors and the programs - 5/6/13
 * v7 - Logic to handle switch code for 10 units - 4/23/13
 * v6 - Changed code to work for 10 units - 3/2/13
 * v5 - Use SN74HC165N to multiplex inputs from recalibration switch - 2/26/13
 * v4 - Use shift registers to control H-Bridge - 2/16/13
 * v3 - Use geared DC motor instead of stepper, and use H-Bridge - 2/7/13
 * v2 - "Points" basic program control - 12/26/12
 * v1 - LEDs, menu system, rotary encoder - 12/23/12
 * v0 - basic motor, lcd, rtc and led control 12/15/12 
 *
 * Includes code from: http://bildr.org/2012/08/rotary-encoder-arduino/ to manage rotary encoder and menuing system
 */

#include "Wire.h"                //I2C protocol
#include "BlinkM_funcs.h"        //BlinkM library
#include "RealTimeClockDS1307.h" //https://github.com/davidhbrown/RealTimeClockDS1307
#include "LiquidCrystal_I2C.h"   //http://arduino-info.wikispaces.com/LCD-Blue-I2C
#include "avr/pgmspace.h"        //for flash memory storage
#include "EEPROM.h"              //for eeprom storage

// Flash Storage: 20 chrs max       12345678901234567890 - these go into flash (program) memory
prog_char menuItem_0[] PROGMEM   = "Auto Run";     //menu items start here
prog_char menuItem_1[] PROGMEM   = "Run Ladder";
prog_char menuItem_2[] PROGMEM   = "Run Points";
prog_char menuItem_3[] PROGMEM   = "Run Angles";
prog_char menuItem_4[] PROGMEM   = "Run Wave";
prog_char menuItem_5[] PROGMEM   = "Run Lines";
prog_char menuItem_6[] PROGMEM   = "Light Show";
prog_char menuItem_7[] PROGMEM   = "Romantic";
prog_char menuItem_8[] PROGMEM   = "Party";
prog_char menuItem_9[] PROGMEM   = "Business";
prog_char menuItem_10[] PROGMEM  = "MOTOR PWM";
prog_char menuItem_11[] PROGMEM  = "Sun + Moon";
prog_char menuItem_12[] PROGMEM  = "(Not Used 3)";

prog_char menuItem_13[] PROGMEM  = "Sleep";
prog_char menuItem_14[] PROGMEM  = "Recalibrate";
prog_char menuItem_15[] PROGMEM  = "Set sleep/wake time";
prog_char menuItem_16[] PROGMEM  = "Set Time";
prog_char menuItem_17[] PROGMEM  = "Stop";
prog_char menuItem_18[] PROGMEM  = "Set ceiling";
prog_char menuItem_19[] PROGMEM  = "Set floor";

prog_char menuItem_20[] PROGMEM  = "Rotate & push to set";  //status messages from this point on
prog_char menuItem_21[] PROGMEM  = "Recalibrating...";//not being shown xxx
prog_char menuItem_22[] PROGMEM  = "Move to ceiling...";
prog_char menuItem_23[] PROGMEM  = "Move to floor...";
prog_char menuItem_24[] PROGMEM  = "Sleeping";
prog_char menuItem_25[] PROGMEM  = "STOP. Push to start";
prog_char menuItem_26[] PROGMEM  = "Moving...";
prog_char menuItem_27[] PROGMEM  = "Moving down...";   //xxx not currently being used
prog_char menuItem_28[] PROGMEM  = "Menu: Turn dial+wait"; //xxx not currently being used
prog_char menuItem_29[] PROGMEM  = "Paused";
// 20 characters maximum:           12345678901234567890

// Then set up a table to refer to the strings:
PROGMEM const char *menuTable[] = {   
  menuItem_0,  menuItem_1,  menuItem_2,  menuItem_3,  menuItem_4,  menuItem_5,  menuItem_6,  menuItem_7,
  menuItem_8,  menuItem_9,  menuItem_10,  menuItem_11,  menuItem_12,  menuItem_13,  menuItem_14,  menuItem_15,
  menuItem_16, menuItem_17, menuItem_18, menuItem_19, menuItem_20, menuItem_21, menuItem_22, menuItem_23,
  menuItem_24, menuItem_25, menuItem_26, menuItem_27, menuItem_28, menuItem_29  };

#define allBlinkMs 0x00  // sends to the I2C "broadcast" address of 0, so all BlinkMs on the I2C bus will respond
#define number_of_74hc595s 4             //How many of the shift registers
#define numOfRegisterPins number_of_74hc595s * 8  //do not touch
//Next lines for 74hc165:
#define NUMBER_OF_SHIFT_CHIPS   2  // How many shift register chips are daisy-chained.
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8  // Width of data (how many ext lines).
#define PULSE_WIDTH_USEC   5  // Width of pulse to trigger the shift register to read and latch. was: 15

//Constants:
  const int minPauseTime = 45;              // in seconds = 2 minutes = 120 xxx
  const int maxPauseTime = 60;              // in seconds = 5 minutes = 300 xxx
  const byte maxCycles = 3;            // How many loops to run through before an automatic recalibration: normally 15 or more?  xxx
  const int maxIndex = 250;
  const byte maxMenuItems = 19;          //how many menu items there are - last item is "set floor"
  const int encoderGap = 2;             //size of gap from encoder before we pay attention to it
  const byte msgUseDial = 20;            //message pointers for menu follow...
  const byte msgRecalibrate = 21;  
  const byte msgCeiling = 22;
  const byte msgFloor = 23;
  const byte msgSleeping = 24;
  const byte msgStopped = 25;
  const byte msgMoving = 26;
  const byte msgDown = 27;
  const byte msgSeeMenu = 28;
  const byte msgPause = 29;
  const byte motorCurrentIndex = 0;      // motor array index position for currentIndex
  const byte motorTargetIndex = 1;      // motor array index position for targetIndex
  const byte motorDeltaMoves = 2;          // motor array index position for deltaMoves that motor has to make
  const byte sleepMenuItem = 13;    // which menu selection corresponds to sleep mode?
  const byte sunAndMoonMenuItem = 11; // which menu selection corresponds to Sun & Moon mode?
  const byte maxAutoRunPrograms = 13; // when on auto run, when to loop back to program #1 (use value of sleep mode)
  const byte maxLightShows = 11;	// maximum number of pre-programmed light shows (used in random light show calls)
  
// Constants for EEPROM memory (refers to index location in EEPROM where we will store the value) using EEPROM.write(address, value) and EEPROM.read(address, value)
  const byte sleepHour_EEPROM = 0;
  const byte wakeHour_EEPROM  = 1;
  const byte ceilingIndex_EEPROM  = 2;              
  const byte floorIndex_EEPROM  = 3;
  
//Variables:
  int SER_Pin = 8;                      //pin 14 on the 75HC595 Shift Register
  int RCLK_Pin = 9;                     //pin 12 on the 75HC595 Shift Register
  int SRCLK_Pin = 10;                   //pin 11 on the 75HC595 Shift Register
  boolean registers[numOfRegisterPins]; // number of 595 registers
  int ploadPin165        = 6;  // Connects to Parallel load (= latch) pin (1) the 165 Shift Register
  int clockEnablePin165  = 13;  // Connects to Clock Enable pin the 165 Shift Register
  int dataPin165         = 5; // Connects to the Q7 (7) pin the 165 Shift Register
  int clockPin165        = 4; // Connects to the Clock pin (2)the 165 Shift Register
  int targetIndex;                      // Index position we need to be at
  char buffer[20];                      // max size of message, and max size of lcd screen
  int encoderPin1 = 2;                  // these pins can not be changed 2/3 are special pins
  int encoderPin2 = 3;                  // used for interrupts on rotary encoder
  int encoderSwitchPin = 7;             // push button switch on rotary encoder
  volatile int lastEncoded = 0;         // for interrupt routine - do not change
  volatile int encoderValue = 1;        // for interrupt routine - do not change
  int activeMenuSelection;              // which menu item is currently selected?
  byte selectedMenuItem = 99;           // value of menu selection at the time the button was pushed
  byte prevSelectedMenuItem = 99;       // previous menu item selection, used for waketime
  int lastEncoderValue = 1;
  int ceilingIndex;                // index for ceiling level
  int floorIndex;                 // index for floor level xxx 60
  int prevSeconds = 99;                // prior value of seconds
  int prevMinutes = 99;                // prior value of minutes
  int lastMSB = 0;                     // for interrupts
  int lastLSB = 0;                     // for interrupts
  int i = 0;  //counter  
  int n0 = 0; //counter
  byte sleepHour;                // time to go to sleep
  byte wakeHour;                 // time to wake up
  byte loopCounter = 0; 
  unsigned int pinValues165;//for 165 Shift Register
  boolean delayFlag = false;    // used as a flag to note if the motor loop needs to have a delay in it
  boolean reverseFlag = false;  // used as a flag to denote whether to change direction of the Lines Program
  boolean autoRunFlag = false; // used to flag if autorun is selected
  boolean doNotRecalibrate = false;
  //boolean motorStateChange = false;
  float sineFrequency = 0;
  int programPauseTime;
  
  byte pointA = 0;  // this section used for selecting random values in programs
  byte pointB = 0;
  byte pointC = 0;
  byte pivotPoint = 0;
  byte gapA = 0;
  byte gapB = 0;

  int motor[10][3];  // 10 motors, 3 attributes: currentIndex | targetIndex | deltaMoves
  //boolean motorRecalibration[10];  //10 motors, 1 bit-sized attribute for recalibration point
  boolean switchValue[10];  // status of switches for each motor

byte color_list[][3] = {
  { 0xff, 0xff, 0xff }, // white = 0
  { 0xff, 0x00, 0xff }, // purple = 1
  { 0xff, 0xff, 0x00 }, // orange = 2
  { 0x00, 0xff, 0xff }, // cyan = 3
  { 0xff, 0x00, 0x00 }, // red = 4
  { 0x00, 0x00, 0xff }, // blue = 5
  { 0xFF, 0xFF, 0x00 }, // yellow = 6
  { 0x00, 0xFF, 0x00 }, // green = 7
  { 0x00, 0x00, 0x00 }, // black = 8
};

byte r;  // red
byte g;  // green
byte b;  // blue

const byte white = 0;  // constants to make it easy to refer to colors in the code
const byte purple = 1;
const byte orange = 2;
const byte cyan = 3;
const byte red = 4;
const byte blue = 5;
const byte yellow = 6;
const byte green = 7;
const byte black = 8;

LiquidCrystal_I2C lcd(0x27,20,4);      // set the LCD address to 0x27 for a 16 chars and 4 line display

void setup()
{
  pinMode(SER_Pin, OUTPUT);          // Shift Register pin
  pinMode(RCLK_Pin, OUTPUT);         // Shift Register pin
  pinMode(SRCLK_Pin, OUTPUT);        // Shift Register pin
  clearRegisters();                  // reset all register pins
  writeRegisters();
  pinMode(ploadPin165, OUTPUT);      // Initialize 165 Shift Register digital pins...
  pinMode(clockEnablePin165, OUTPUT);
  pinMode(clockPin165, OUTPUT);
  pinMode(dataPin165, INPUT);
  digitalWrite(clockPin165, LOW);
  digitalWrite(ploadPin165, HIGH);
  
  Serial.begin(9600);                 // start serial comms for terminal
  lcd.init();                         // initialize the lcd 
  lcd.backlight();                    // turn on the backlight
  
  BlinkM_beginWithPower();            // turn on LED
  BlinkM_setFadeSpeed(allBlinkMs, 30);// Higher numbers means faster fading, 255 == instantaneous fading
  BlinkM_stopScript( allBlinkMs );    // turn off startup script
  ballFadeTo(allBlinkMs, black );     // turn off any LEDs
  swipeRight(50, white, green);      // turn on all LEDs on at a time
  BlinkM_scanI2CBus(1,100, scanfunc); // show debug printout of all i2c devices
  swipeLeft(20, green, black);       // visual fun
  ballGoTo(1, green);                 // when plugged in, show unit 1 as green so you know it's working
  
//Configure pins:
  pinMode(encoderPin1, INPUT);         // rotary encoder
  pinMode(encoderPin2, INPUT);         // interrupt pins
  pinMode(encoderSwitchPin, INPUT);    // push button
  digitalWrite(encoderPin1, HIGH);     // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);     // turn pullup resistor on
  digitalWrite(encoderSwitchPin, HIGH);// turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);   //call updateEncoder() when any high/low changed seen
  attachInterrupt(1, updateEncoder, CHANGE);   //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 

  randomSeed(analogRead(A1));          // random seed based on empty analog pin 1

	//  EEPROM.write( floorIndex_EEPROM, 40 );  only needs to be set once - do not uncomment
	//  EEPROM.write( ceilingIndex_EEPROM, 3 ); only needs to be set once - do not uncomment

		sleepHour =    EEPROM.read( sleepHour_EEPROM );	// refresh these variables as stored in EEPROM
		wakeHour =     EEPROM.read( wakeHour_EEPROM );
		ceilingIndex = EEPROM.read( ceilingIndex_EEPROM );              
		floorIndex =   EEPROM.read( floorIndex_EEPROM );
  
  /*RTC.setHours(11);   //At startup: set time - TEST CODE ONLY
  RTC.setMinutes(56);
  RTC.setClock();*/ 

//activeMenuSelection = 10;
} 

void loop()
{
showTime();

//menu selection  
if (abs(lastEncoderValue - encoderValue) > encoderGap) {  //menu selection has changed
  if (encoderValue > lastEncoderValue) {                  //figure out what the new menu selection is
    activeMenuSelection ++;                               // add 1 to menu
    if (activeMenuSelection > maxMenuItems) activeMenuSelection = 0;
  }else{ 
    activeMenuSelection --;                               // subtract 1 from menu
    if (activeMenuSelection < 0) activeMenuSelection = maxMenuItems;
  }
      clearLine(0);
      getMessage(activeMenuSelection);
      lcd.setCursor(0, 0);
      lcd.print(">");                                     //highlight active menu item
      lcd.print(buffer);
     
  lastEncoderValue = encoderValue;
 }
 
if(digitalRead(encoderSwitchPin)){  // check if button has been pushed 
    switch (selectedMenuItem) {  // button is not being pushed, which means we can execute code based on it being pushed last time round this loop

      case 0:    // auto-run... cycle through multiple programs
        autoRunFlag = true;  // signal that we are in auto run mode  
        break;  

      case 1:    // ladder program (up and down while running scripts)
          recalibrate();
          do { // head down until units reach the floor
            targetIndex = programTargetIndex(motor[0][motorCurrentIndex]+1, floorIndex); // find a random place to go between currentIndex of motor 0 [0][0] and floor
            for (int i=0 ; i < 10; i++) {   // assign value to all motor targetIndex
				motor[i][motorTargetIndex] = targetIndex;  // position [1] is targetIndex
				ballFadeTo((i+1), blue);
				moveTo();
				if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
				BlinkM_playScript(allBlinkMs, 11, 0, 0);                    // play script #11: mood light 
            }
           
            if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
            programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
            pauseTime(programPauseTime);                                // convert to milliseconds
          } while (motor[0][motorCurrentIndex] != floorIndex);   //[0][0] = motor 0, currentIndex value

         if (abs(lastEncoderValue - encoderValue) > encoderGap) break;
         do { // head up until units reach the ceiling
            targetIndex = programTargetIndex(ceilingIndex, (motor[0][motorCurrentIndex]-1)); // find a random place to go between currentIndex of motor 0 [0][0] and top
            for (int i=0 ; i < 10; i++) {   // assign value to all motor targetIndex
				motor[i][1] = targetIndex;  // position [1] is targetIndex
				ballFadeTo((i+1), purple);
				moveTo();
				if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
				BlinkM_playScript(allBlinkMs, 14, 0, 0);                    // play script #14: old neon - reds
            }
            
            if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
            programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
            pauseTime(programPauseTime);                                 // convert to milliseconds           
          } while (motor[0][0]  != ceilingIndex); //[0][0] = motor 0, currentIndex value     
        break;
        
      case 2:   // random points program
        recalibrate(); 
        loopCounter = 0;     
        do {
           for (int i=0 ; i < 10; i++) { 
				motor[i][motorTargetIndex] = programTargetIndex(ceilingIndex, floorIndex);             // find a random place to go for each ball
				ballFadeTo(Ii+1), cyan);
				moveTo();
				if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
				BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
           }          
                    
          loopCounter ++;
          if (loopCounter > maxCycles) break;  // exit this case statement; we'll come around and do it again
          programPauseTime = random(minPauseTime, maxPauseTime);           // in seconds - wait before moving
          //pauseTime(programPauseTime); 
		  pauseTime(10); // temp override for debugging XXX
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;
                
      case 3:  // angles
        recalibrate(); 
        loopCounter = 0; 

         do {      
          pointA = programTargetIndex( ceilingIndex, ((floorIndex - ceilingIndex) * 0.4) ); // PointA: find a random point between the ceiling and top 40% from the ceiling
          pointB = programTargetIndex( pointA, floorIndex ); // PointB: find a random point between PointA and the floor
          pointC = programTargetIndex( ceilingIndex, pointB ); // PointC: find a random point between PointB and the ceiling
          pivotPoint =  programTargetIndex( 3 , 7 ); // PivotPoint: find a random spot between units 3 and 7
              
          gapA = (pointB - pointA) / (pivotPoint -1) ; // gapA gaps between pointA and pointB is (pB-pA)/(pivotPoint-1)
          gapB = (pointB - pointC) / (10 - pivotPoint) ; // gapB gaps between pointB and pointC is (pB-pC)/(10 - pivotPoint)
          
		  curtainsOut(100, white, black);
          ballFadeTo(allBlinkMs, blue);

          motor[0][motorTargetIndex] = pointA;
          motor[(pivotPoint-1)][motorTargetIndex] = pointB; // pivotPoint varies and we need to subtract 1 because arrays are counted from zero
          motor[9][motorTargetIndex] = pointC;
          
          for (int i = 1; i < pivotPoint; i++) { // program units 1 thru pivotPoint (exclusive)
            motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA);
		            moveTo();
									if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
          BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color }
          for (int i = pivotPoint; i < 9; i++) { // program units pivotPoint (exclusive) thru 10
            motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] - gapB);
		            moveTo();
									if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
          BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color }
         
          loopCounter ++;
          if (loopCounter > maxCycles) {
            break; }

          programPauseTime = random(minPauseTime, maxPauseTime);
          pauseTime(programPauseTime);                               
        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;

      case 4:  // waves
       recalibrate();
       sineFrequency = random(1,11);              // pick a random sine wave multiplier from 1 to 10
       sineFrequency = sineFrequency/10;          // convert to a number between 0.1 and 1.0
       loopCounter = 1;    // same as "current step"
       
       pointA = calculateSinPosition(loopCounter, sineFrequency);  // pointA is the value for unit 10
       
       for (int i=0; i < 10; i++) {  // first time round, populate all units with same value
         motor[i][motorTargetIndex] = pointA; }
       
       do{
         ballFadeTo(allBlinkMs, blue);
         moveTo();
		 if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.             
         curtainsOut(100, white, black);
         BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
         programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
         pauseTime(programPauseTime); 
         
         for (int i=0 ; i < 9; i++) {// shift all values to the left by one unit
           motor[i][motorTargetIndex] = motor[i+1][motorTargetIndex]; }
         
         loopCounter ++;
              if (loopCounter > (maxCycles * 3)) { // xxx check this size loop
              break; }
              
         pointA = calculateSinPosition(loopCounter, sineFrequency);
         motor[9][motorTargetIndex] = pointA;  // seed unit 10 with the new value
        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;

      case 5:  // Lines
        recalibrate(); 
        reverseFlag = ~reverseFlag;  // flip from false-true
        loopCounter = 0; 

         do {      
          pointA = programTargetIndex( ceilingIndex, ((floorIndex - ceilingIndex) * 0.4) ); // PointA: find a random point between the ceiling and top 40% from the ceiling
          pointB = programTargetIndex( (pointA * 1.5), floorIndex ); // PointB: find a random point between PointA and the floor
           
          gapA = (pointB - pointA) / 8 ; // gapA gaps between pointA and pointB is (pB-pA)/82
         
          if (reverseFlag == false) {      // line goes from left to right      
            motor[0][motorTargetIndex] = pointA;
            motor[9][motorTargetIndex] = pointB;          
            for (int i = 1; i < 9; i++) { // program units 1 thru 9 (exclusive)
              motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA); }
          } else {                     // line goes from right to left
            motor[9][motorTargetIndex] = pointA;
            motor[0][motorTargetIndex] = pointB;          
            for (int i = 8; i > 0; i--) { // program units 1 thru 9 (exclusive)
              motor[i][motorTargetIndex] = (motor[(i+1)][motorTargetIndex] + gapA); }                  
          }
                      
          curtainsOut(100, white, black);
          ballFadeTo(allBlinkMs, blue);          
          moveTo();
		  if (abs(lastEncoderValue - encoderValue) > encoderGap) break;// exit if dial has turned.
          BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
                   
          loopCounter ++;
          if (loopCounter > maxCycles) break; 

          programPauseTime = random(minPauseTime, maxPauseTime);           // in seconds - wait before moving
          pauseTime(programPauseTime);                                // convert to milliseconds

        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
               
        break;
        
      case 6:  // Light show 
       //setUpChandelier(); // adopt chandelier shape     xxx
       programPauseTime = random(minPauseTime, maxPauseTime);      // in seconds - wait before moving
       pauseTime(programPauseTime);                                
       randomLightShow(random(0,maxLightShows+1));	// do something flashy
       randomLightShow(7);      // then play a random script                           
       break;
       
      case 7:  // "Romantic"
       recalibrate();
       sineFrequency = random(1,11);              // pick a random sine wave multiplier from 1 to 10
       sineFrequency = sineFrequency/10;          // convert to a number between 0.1 and 1.0
       loopCounter = 1;                           // same as "current step"
       
       pointA = calculateSinPosition(loopCounter, sineFrequency);  // pointA is the value for unit 10
       
       for (int i=0; i < 10; i++) {  // first time round, populate all units with same value
         motor[i][motorTargetIndex] = pointA; 
         moveTo();
         if (abs(lastEncoderValue - encoderValue) > encoderGap) break;   }  // move units one at a time (slowly)
       
       do{             
         curtainsOut(100, white, black);
         randomLightShow(7);      // play a random script 
         programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
         pauseTime(programPauseTime); 
         
         for (int i=0 ; i < 9; i++) {  // shift all values to the left by one unit
           motor[i][motorTargetIndex] = motor[i+1][motorTargetIndex];
           ballFadeTo(i+1, white);
           moveTo();
           if (abs(lastEncoderValue - encoderValue) > encoderGap) break;  // line to allow break from loops if dial is turned
           randomLightShow(7);   // play one of the scripts
         }
         
         loopCounter ++;
         if (loopCounter > (maxCycles * 3)) break; // xxx check this size loop
              
         pointA = calculateSinPosition(loopCounter, sineFrequency);
         motor[9][motorTargetIndex] = pointA;  // seed unit 10 with the new value        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;      
       
      case 8:  // "Party"
        recalibrate(); 
        loopCounter = 0;     
        do {
           for (int i=0 ; i < 10; i++) { 
            motor[i][motorTargetIndex] = programTargetIndex(ceilingIndex, floorIndex);             // find a random place to go for each ball  
            randomLightShow(random(0,maxLightShows+1));              
            ballFadeTo( i+1, white);
            moveTo();
            if (abs(lastEncoderValue - encoderValue) > encoderGap) break;  // line to allow break from loops if dial is turned            
            //BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
        }          

          loopCounter ++;
          if (loopCounter > maxCycles) break;  // exit this case statement; we'll come around and do it again
                             
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;
       
      case 9:  // "Business" xxx merge with code from Lines function
        recalibrate(); 
        reverseFlag = ~reverseFlag;  // flip from false-true
        loopCounter = 0; 

         do {      
          pointA = programTargetIndex( ceilingIndex, ((floorIndex - ceilingIndex) * 0.4) ); // PointA: find a random point between the ceiling and top 40% from the ceiling
          pointB = programTargetIndex( (pointA * 1.5), floorIndex ); // PointB: find a random point between PointA and the floor
           
          gapA = (pointB - pointA) / 8 ; // gapA gaps between pointA and pointB is (pB-pA)/82
         
          if (reverseFlag == false) {      // line goes from left to right      
            motor[0][motorTargetIndex] = pointA;
            motor[9][motorTargetIndex] = pointB;          
            for (int i = 1; i < 9; i++) { // program units 1 thru 9 (exclusive)
              motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA);
              BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
              programPauseTime = random(minPauseTime, maxPauseTime);           // in seconds - wait before moving
              pauseTime(programPauseTime);                                 
              ballFadeTo( i+1, green);            
              moveTo();
              if (abs(lastEncoderValue - encoderValue) > encoderGap) break; } // line to allow break from loops if dial is turned   
          } else {                     // line goes from right to left
            motor[9][motorTargetIndex] = pointA;
            motor[0][motorTargetIndex] = pointB;          
            for (int i = 8; i > 0; i--) { // program units 1 thru 9 (exclusive)
              motor[i][motorTargetIndex] = (motor[(i+1)][motorTargetIndex] + gapA);
              BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
              programPauseTime = random(minPauseTime, maxPauseTime);           // in seconds - wait before moving
              pauseTime(programPauseTime);                                          
              ballFadeTo( i+1, green);
              moveTo(); 
              if (abs(lastEncoderValue - encoderValue) > encoderGap) break; } // line to allow break from loops if dial is turned               
          }
     
          loopCounter ++;
          if (loopCounter > maxCycles) break; 

        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.      
      break;
      
      case 10:  // motor PWM     
      
		  motor[1][motorTargetIndex] = motor[1][motorCurrentIndex] - 10;
		  moveTo();

		selectedMenuItem = 99;
		break;
       
      case 11:  // "Sun & Moon" (Not affected by auto sleep)
        //setUpChandelier(); // adopt chandelier shape xxx
        i = determineTimeOfDay(RTC.getHours());
        if (i > 1) BlinkM_playScript(allBlinkMs, i, 0, 0); //   Args: blinkM addr, script#, 0=repeat forever, 0=play from beginning
        
        if (i == 0) { // if night...
          BlinkM_fadeToHSB(allBlinkMs, 43, 0, 0x2d);  // white (43) dim      
          BlinkM_fadeToRandomHSB(allBlinkMs, 0, 0, 0x3c); // no change in hue, 0 sat and random brightness +/- 60
        }
        
        if (i == 1) { // if afternoon...
        // Serial.println("i==1");
          BlinkM_fadeToHSB(allBlinkMs, 43, 0x00, 0xff);  // white (43) bright      
          BlinkM_fadeToRandomHSB(allBlinkMs, 0, 10, 0xc8); // no change in hue, 0 sat and random brightness +/- 200
        }
		pauseTime(60);  // no rush, let this linger for a minute
      break;    // and then, allow to repeat
      
       
      case 12:  // "(Not Used 5)"
      
      break;
              
      case 13:   //sleep mode
        recalibrate();
        printStatusMessage(msgSleeping);
        
        do {
          for (int i = 1; i < 11; i++) {
              ballFadeTo(i, white);
              pauseTime(1);
              ballFadeTo(i, black);
              pauseTime(5);
              if (abs(lastEncoderValue - encoderValue) > encoderGap) break;  // line to allow break from loops if dial is turned
          }
         //showTime();
         
             if (wakeHour == RTC.getHours()) break;
               
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.         
        
        selectedMenuItem = 99;  // prevent this case from being selected next time round
        if (wakeHour == RTC.getHours()) selectedMenuItem = prevSelectedMenuItem; // remember what the previously-running program was
        break;
      
      case 14:  //recalibrate  
        recalibrate();
        selectedMenuItem =99;  // prevent this case from being selected next time round        
        break;     
       
      case 15:  // set sleep and wake time 
        printStatusMessage(msgUseDial);
        lcd.setCursor(0, 2);
        lcd.print("Sleep");
        do{ // let's set sleep hour...
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {
                sleepHour = addHour(sleepHour);
            }else{ 
                sleepHour = subtractHour(sleepHour);
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set time        
        EEPROM.write( sleepHour_EEPROM, sleepHour );      // update EEPROM
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print("Wake");
        delay(500);  // slight pause to allow time for button to be released
        do{ // let's set wake hour...
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {
                wakeHour = addHour(wakeHour);
                if (wakeHour == sleepHour) wakeHour = addHour(wakeHour);	// if wake and sleep are the same, increment wake
            }else{ 
                wakeHour = subtractHour(wakeHour);
                if (wakeHour == sleepHour) wakeHour = subtractHour(wakeHour);	// if wake and sleep are the same, decrement wake
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set time   
        EEPROM.write( wakeHour_EEPROM, wakeHour );      // update EEPROM
        clearLine(2);
        selectedMenuItem =99;  // prevent this case from being selected next time round        
        break;
       
      case 16: //set time
        printStatusMessage(msgUseDial);
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
            //figure out what the new menu selection is
              if (encoderValue > lastEncoderValue) {
                addMinute();
            }else{ 
                subtractMinute();
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set time
        clearLine(2);
        clearLine(3);
        selectedMenuItem =99;  // prevent this case from being selected next time round        
       break; 
              
      case 17:  //stop/resume - do not recalibrate - just stop whatever is happening right now
        printStatusMessage(msgStopped);
        ballFadeTo( allBlinkMs, green);
        do {
          displayFullTime();
          delay(500);  // just wait until a resume is requested
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed 
        selectedMenuItem =99;  // prevent this case from being selected next time round        
        break;
  
      case 18:  //Set Ceiling 
        recalibrate();  //first, recalibrate
        printStatusMessage(msgUseDial);
        motor[0][motorTargetIndex] = ceilingIndex; 
        moveTo();
        
        do{
          encoderHeightChange();
          lcd.setCursor(0, 2);    //print hours on lcd
          lcd.print("Ceiling:");
          printPositionDigits(motor[0][motorTargetIndex]);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();
        
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        ceilingIndex = motor[0][motorCurrentIndex];              // new value for ceiling
        EEPROM.write( ceilingIndex_EEPROM, ceilingIndex ); 
        clearLine(2);
        clearLine(3);
        selectedMenuItem =99;  // prevent this case from being selected next time round        
        break;
        
      case 19:  //Set floor
        recalibrate();  //first, recalibrate
        printStatusMessage(msgUseDial);
        motor[0][motorTargetIndex] = floorIndex; 
        moveTo();        
        
       do{
          encoderHeightChange();
          lcd.setCursor(0, 2);    //print hours on lcd
          lcd.print("Floor:");
          printPositionDigits(motor[0][motorTargetIndex]);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();           
          
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        floorIndex = motor[0][motorCurrentIndex];              // new value for floor
        EEPROM.write( floorIndex_EEPROM, floorIndex ); 
        clearLine(2);
        clearLine(3);
        selectedMenuItem =99;  // prevent this case from being selected next time round
        break;
        
    }
    
	delay(200); //gives time for button to be released  
    if (abs(lastEncoderValue - encoderValue) > encoderGap){ // the dial has just been turned
     //Serial.print("@99 ");
     autoRunFlag = false;  // signal that we are not in auto run mode     
     selectedMenuItem = 99;}  // prevent any case from being selected next time round if dial was turned
    
    if ((sleepHour == RTC.getHours()) && (selectedMenuItem != sunAndMoonMenuItem)) { // ensure we're not in Sun & Moon mode
      prevSelectedMenuItem = selectedMenuItem; // remember what the currently-running program is for wake time
      selectedMenuItem = sleepMenuItem; } // go to sleep if it's sleeping hour
      
    if (autoRunFlag == true){  // we're in auto-run mode and need to cycle through each program in turn
      selectedMenuItem ++;     // if it was 0 (auto run selected) now it will be 1
      if (selectedMenuItem >= maxAutoRunPrograms) selectedMenuItem = 1; } // next time round, the correct program will run
    
  }else{
    //button is being pushed
    lcd.setCursor(17, 0);
    lcd.print("<<<"); //indicate to user it has been pushed
    selectedMenuItem = activeMenuSelection;
    BlinkM_stopScript( allBlinkMs );   // turn off any script currently playing
  }
}  // end of loop

//------------------------------------
void pwmMotor(byte motor, byte offDuty, byte onDuty, byte loops, boolean overRide = false){

		  for (byte i = 0; i < loops; i++) {			  
			  setRegisterPin((3*motor), LOW );           // OFF
			  writeRegisters();
			  delay(offDuty);
			  setRegisterPin((3*motor), HIGH );           // ON
			  writeRegisters();

			  pinValues165 = read_shift_regs165();		// read 165 shift registers
			  if((pinValues165 >> (motor + 6)) & true){	// need to adjust by 6 because of circuit design
				  switchValue[motor] = true;				// this motor has reached recalibration point
				  if (overRide == false) break;				// exit immediately unless we are forcing a slow down
				} else {
				  switchValue[motor] = false;}

			  delay(onDuty);
		  }
}

//------------------------------------
byte determineTimeOfDay(byte i){ // i = current hour
 byte script = 0;         // default = moon program
 if (i >  3) script = 12; //  4am+ random yellows
 if (i >  7) script = 11; //  8am+ random hues
 if (i > 10) script =  1; // 11am+ white program
 if (i > 16) script = 13; //  5pm+ random blues
 if (i > 18) script = 14; //  7pm+ random orangeish reds
 if (i > 21) script =  0; // 10pm+ moon program
 BlinkM_stopScript( allBlinkMs );   // turn off any script currently playing
 return script; // script = 0 or 1 for special program; or number of script to play on BlinkM
}
//------------------------------------
void setUpChandelier() {
	if (doNotRecalibrate == false) { // first time here, so we do need to recalibrate this one time
		recalibrate();
          
        // go to the chandelier position
        pointA = ceilingIndex;  // PointA: both edges of chandelier at ceiling height
        pointB = ((floorIndex - ceilingIndex) * 0.4); // PointB: 40% of the way down                   
        gapA = pointB - pointA ;
       
        motor[0][motorTargetIndex] = pointA;                  // top
        motor[1][motorTargetIndex] = pointA + (gapA * 0.3);
        motor[2][motorTargetIndex] = motor[1][motorTargetIndex] + (gapA * 0.2);
        motor[3][motorTargetIndex] = motor[2][motorTargetIndex] + (gapA * 0.1);        
        motor[4][motorTargetIndex] = pointB;                  // lowest point
        motor[5][motorTargetIndex] = pointB;                  // lowest point
        motor[6][motorTargetIndex] = motor[3][motorTargetIndex];
        motor[7][motorTargetIndex] = motor[2][motorTargetIndex];
        motor[8][motorTargetIndex] = motor[1][motorTargetIndex];                
        motor[9][motorTargetIndex] = pointA;                  // top
       
        BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color
        moveTo();
        doNotRecalibrate = true; // set flag so we don't recalibrate next time around
        } // end of doNotRecalibrate  
}
//------------------------------------
void randomLightShow(byte i) {  
 int lightSpeed = 0;
 byte lightColor1;
 byte lightColor2;
 lightSpeed = random(50,200);
 lightColor1 = random(0,9); // between colors 0 and 8
 lightColor2 = random(0,8); // between colors 0 and 7 (excludes 8=black)
 
 BlinkM_stopScript( allBlinkMs );    // stop all scripts from playing
 
 switch (i) {  // 

        case 0:    //
          christmasLights(250, lightColor1, lightColor2, 10);
        break; 
       
        case 1:    //
          curtainsOut(lightSpeed, lightColor1, lightColor2);        
        break; 
       
        case 2:    //
          curtainsIn(lightSpeed, lightColor1, lightColor2);      
        break; 
       
        case 3:    //
          swipeRight(lightSpeed, lightColor1, lightColor2);
        break; 
       
        case 4:    //
          swipeLeft(lightSpeed, lightColor1, lightColor2);
        break;        

        case 5:    //
          ballFadeTo(allBlinkMs, lightColor2); // prevents fade to black
        break; 

        case 6:    //
          flashAll(lightSpeed, lightColor1, lightColor2);
        break; 

        case 7:    // play a random script between script 10 and 16 inclusive
          BlinkM_playScript(allBlinkMs,(random(10,17)), 0, 0); //   Args: blinkM addr, script#, 0=repeat forever, 0=play from beginning      
        break; 

        case 8:    //
          BlinkM_fadeToRandomRGB(allBlinkMs, 0xff, 0xff, 0xff);      // set a random color        
        break; 

        case 9:    //
          fadeRight(lightSpeed, lightColor1, lightColor2);        
        break; 

        case 10:    // custom script
          fadeRight(30, lightColor2, black);  
          ballFadeTo(allBlinkMs, lightColor2); 
          pauseTime(5);
          flashAll(80, white, lightColor2);
          delay(100);   
          flashAll(80, white, lightColor2);   
          pauseTime(1);     
          for (byte i= random(3,9) ; i > 0; i--) {
            swipeRight(15, white, lightColor1);
            swipeRight(18, white, lightColor2);
          }
        break;
        
        case 11: // fill in        
          for (byte i = 0; i<50; i++){ // 50 loops should fill in each unit
           ballGoTo(random(1,11), lightColor1); // pick a random unit to color between 1 and 10 & turn it to the color   
           delay(100);         // slight pause for effect
          }          
        break;
       } // end of switch loop                      
}
//------------------------------------
void curtainsOut(byte velocity, byte color1, byte color2) {
  byte steps = 5;
  int unitA = 6;
  int unitB = 5;
  do {
    ballGoTo(unitA, color1);
    ballGoTo(unitB, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    ballGoTo(unitB, color2);
    unitA ++;
    unitB --;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void curtainsIn(byte velocity, byte color1, byte color2) {
  byte steps = 5;
  int unitA = 1;
  int unitB = 10;
  do {
    ballGoTo(unitA, color1);
    ballGoTo(unitB, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    ballGoTo(unitB, color2);
    unitA ++;
    unitB --;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void swipeRight(byte velocity, byte color1, byte color2) {
  byte steps = 10;
  int unitA = 1;
  do {
    ballGoTo(unitA, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    unitA ++;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void swipeLeft(byte velocity, byte color1, byte color2) {
  byte steps = 10;
  int unitA = 10;
  do {
    ballGoTo(unitA, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    unitA --;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void fadeRight(int velocity, byte color1, byte color2) {
  byte steps = 10;
  int unitA = 1;
  do {
    ballFadeTo(unitA, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    unitA ++;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void flashAll(byte velocity, byte color1, byte color2) {
    ballGoTo(allBlinkMs, color1);
    delay(velocity);
    ballGoTo(allBlinkMs, color2);
}
//------------------------------------
void christmasLights(byte velocity, byte color1, byte color2, byte flips) {
  do {  // swap the two colors
    flipColors(color1, color2);
    delay(velocity);
    flipColors(color2, color1);
    delay(velocity);
    flips --;
  } while (flips >0);
}
//------------------------------------
void flipColors(byte color1, byte color2) {
  byte steps = 5;
  int unitA = 1;
  int unitB = 2;
  do {
    ballGoTo(unitA, color1);
    ballGoTo(unitB, color2);
    unitA ++;
    unitA ++;
    unitB ++;
    unitB ++;
    steps --;
  } while (steps > 0);
}  
//------------------------------------
void ballFadeTo(byte unit,byte color) {  // fade to this color
  r = color_list[color][0];
  g = color_list[color][1];
  b = color_list[color][2];
  BlinkM_stopScript( unit );   // turn off any script currently playing
  BlinkM_fadeToRGB( unit, r,g,b );    
}
//------------------------------------
void ballGoTo(byte unit,byte color) {  // go directly to this color
  r = color_list[color][0];
  g = color_list[color][1];
  b = color_list[color][2];
  BlinkM_stopScript( unit );   // turn off any script currently playing
  BlinkM_setRGB( unit, r,g,b );    
}
//------------------------------------
int programTargetIndex(int i, int j) { // find a random point between two items
  int k;
  k = random (i,j+1);
  return k;
}
//------------------------------------
int calculateSinPosition(byte loopCounter, float sineFrequency) {
  int index = 0;              // prepare the index position to return to PointA
  float transformedSinWave = 0;
  float sinWave = 0;          // prepare a floating point number
  sinWave = sin(loopCounter * sineFrequency);  // this is the value, in degrees, for this point of the sine wave
  //Serial.print(sinWave);
  //Serial.print(",");
  // need to convert it (-1 to +1) into a value that represents a distance between ceiling and floor
  sinWave = sinWave +1;        // add 1: sinWave is now between 0 and 2
  transformedSinWave = ceilingIndex + ((sinWave / 2 ) * (floorIndex - ceilingIndex));// what percent of a max of value "2" is it?
   // this is the gap that we have
  // add ceilingIndex to get the value we should go to
  index = transformedSinWave;  // ram floating point number into integer... it will be close enough for union work
  //Serial.println(index);
  return index;
}
//------------------------------------
void pauseTime(int k) { // pause for k seconds, including a countdown
  printStatusMessage(msgPause);
  prevMinutes = RTC.getMinutes();
  for (int i=k ; i > 0; i--) {
    showTime();
    lcd.setCursor(17,3);
    printPositionDigits(i);
    delay(997);// pause for almost one second
    if ((RTC.getMinutes() != prevMinutes) && 
        (selectedMenuItem != sleepMenuItem) && 
        (selectedMenuItem != sunAndMoonMenuItem)) {  // if one minute has passed and we're not in sleep mode nor sun & moon mode
       randomLightShow(random(0,maxLightShows+1));  
       prevMinutes = RTC.getMinutes();
    }
    if (abs(lastEncoderValue - encoderValue) > encoderGap) break; // exit when dial has turned.
  }
}
//------------------------------------
void encoderHeightChange(){ //adjust ceiling/floor value by reading encoder
  if (abs(lastEncoderValue - encoderValue) > encoderGap){
    if (encoderValue > lastEncoderValue) {
       motor[0][motorTargetIndex] ++;
      if (targetIndex > maxIndex) targetIndex = maxIndex;             
   }else{ 
      motor[0][motorTargetIndex] --;
      if (targetIndex < 0) targetIndex = 0;
   }
 }
}
//------------------------------------
void updatePosition(){  // every 5 seconds, adjust the ball           
  RTC.readClock();  // find out what the time is 
  if (abs(RTC.getSeconds()-prevSeconds) > 4) {   // if 5 seconds has passed...
    displayFullTime();                           // display the new time
    prevSeconds = RTC.getSeconds();              // save for next time round
    moveTo();
    printStatusMessage(msgUseDial);}     
}
//------------------------------------   
//Go to any point - based on currentIndex [motor#][0] go to targetIndex [motor#][1] 
void moveTo() {
  Serial.println("@MoveTo");
              for (int i=0 ; i < 10; i++) {   // display all motor currentIndex, targetIndex
              Serial.print(i);
              Serial.print("=");
              Serial.print(motor[i][motorCurrentIndex]);
              Serial.print(" > ");
              Serial.println(motor[i][motorTargetIndex]);
            }
  
// for each motor, set 595 register pins for direction
// calculate how many steps (moves) each motor needs to take
for (int i=0 ; i < 10; i++) {
  
  if (motor[i][motorCurrentIndex] > motor[i][motorTargetIndex]) {  // if currentIndex > targetIndex, need to move up CW
    //Serial.print("Up ");
    motorDirectionUp(i);    
    motor[i][motorDeltaMoves] = motor[i][motorCurrentIndex] - motor[i][motorTargetIndex];  //move this many steps: current - Target
    
} else if (motor[i][motorCurrentIndex]  < motor[i][motorTargetIndex]) {  // if currentIndex < targetIndex need to move down CCW
    //Serial.print("Down ");
    motorDirectionDown(i);
    motor[i][motorDeltaMoves] = motor[i][motorTargetIndex] - motor[i][motorCurrentIndex] ; //move this many steps: target - Current
  }
}
moveBall();  // move all balls
  
for (int i=0 ; i < 10; i++) {  
   motor[i][motorCurrentIndex] = motor[i][motorTargetIndex];
}
 //clearLine(2);  // clear moving to.. message        
 doNotRecalibrate = false; // set flag so programs with "do not recalibrate" will work properly
}
//------------------------------------
void motorDirectionUp(int unit) {  // set motor direction 0 thru 9 to CW (up)
    setRegisterPin((3*unit)+1,HIGH); // set appropriate 595 register pin
    setRegisterPin((3*unit)+2,LOW); 
}
//------------------------------------
void motorDirectionDown(int unit) {  // set motor direction 0 thru 9 to CCW (down)
    setRegisterPin((3*unit)+1,LOW); // set appropriate 595 register pin
    setRegisterPin((3*unit)+2,HIGH); 
}
//------------------------------------
void moveBall() { //spin motor so that each ball that needs to moves the right number of steps

//byte maximumDelta = 0;                   // used to determine maximum loops
//for (int i=0 ; i < 10; i++) {            // figure out which motors need to have their pin enabled
  //motorRecalibration[i] = false;         // set recalibration flag for each unit to false
  //if (motor[i][motorDeltaMoves] != 0) {  // this motor is set to move
    //Serial.print("E");
    //Serial.println(i);
    //if (maximumDelta < motor[i][motorDeltaMoves]) maximumDelta = motor[i][motorDeltaMoves]; // set maximum
    //setRegisterPin((3*i),HIGH);          // enable motor bit in shift register
  //} }
	
	for (int i=0 ; i < 10; i++) {	// assess motors 0 thru 9 in turn
		if (motor[i][motorDeltaMoves] != 0) { // if deltamoves != 0, then we are going to move this motor
			//ballGoTo((i+1), white);
			printStatusMessage(msgMoving); 
			// ramp up the motor
			pwmMotor(i, 25,  5, 30);												// slow @ 25%
			if (switchValue[i] == false) pwmMotor(i, 13, 13, 20);					// medium @ 50%
			if (switchValue[i] == false) pwmMotor(i,  5, 25, 10);					// fast @ 75%

			// go at full speed				
			while ((motor[i][motorDeltaMoves] != 0) && (switchValue[i] == false)) {// continue while moves != 0 & switch is false {
				lcd.setCursor(17,3);
				printPositionDigits(motor[i][motorDeltaMoves]);				        // update LCD with current step number
				motor[i][motorDeltaMoves] --;										// decrement delta moves
				pwmMotor(i,  0, 25, 39);											// full speed @ 100% xxx approx 1 sec according to debug trace
			} 
			
			// slow down the motor
			if (switchValue[i] == false) pwmMotor(i,  5, 25, 10);					// fast
			pwmMotor(i, 13, 13, 10, true);											// medium
			pwmMotor(i, 25,  5, 20, true);											// slow (gets called if recalibration point is reached - mandatory)

			setRegisterPin((3*i), LOW );		        // ensure motor is OFF
			writeRegisters();
			
			if (switchValue[i] == true)  {	// we are at recalibration point and need to unwind				 
				  ballGoTo((i+1), red);
				  pauseTime(1);								// small delay before we reverse
				  motorDirectionDown(i);                  // motor will begin to go down
				  do {
					  pwmMotor(i, 25,  5, 30, true);				// go down slowly
				  } while (switchValue[i] == true);
			
				setRegisterPin((3*i), LOW );		        // ensure motor is OFF
				writeRegisters();
				motor[i][motorDeltaMoves] = 0;				// no more moves to make
				motor[i][motorTargetIndex] = 0;				// we are at position 0 (adjusted for CurrentIndex later)
			}
		
		}	// end of if loop deltamoves != 0
		//ballGoTo((i+1), blue);
		showTime();    // keep time up to date
		clearLine(3);  // clear status message
	}	// end of assess motor loop... go to the next motor

  clearRegisters();
  writeRegisters();                      // disable all motors

/* do {     

    /*Serial.print(maximumDelta);
    Serial.print(" - ");
    for (int i=0 ; i < 10; i++) {                // for each motor 
      Serial.print(motor[i][motorDeltaMoves]);
      Serial.print(" ");}
      Serial.println(); 
    maximumDelta --;
    
    for (byte l=3; l>0; l--) {            // check ball status 3 times per second
        
      
      /*for(byte i = 0; i < 10; i++){  //loop thru each bit from bit positions 6 thru 15 (== units 0 thru 9)
        if((switchValue[i] == true) && (motorRecalibration[i] == false)) {       //if the bit equal to 1 (true) and recalibration flag is false, then
          motorRecalibration[i] = true;  //set this unit's recalibration flag to true                                            
          Serial.print(i);
          Serial.println(" -1");    //we are at position -1 and need to unwind until the switch turns off
          setRegisterPin((3*i),LOW);           // disable motor bit in shift register          
          motor[i][motorDeltaMoves] = 0;  // ensure we keep this loop active by keeping xxx check 99
          motorStateChange = true;
          ballGoTo(i+1, red); }          // go directly to red. motors are addressed 1 thru 10, not 0-9                            
      } // end of for loop
      


if (motorStateChange == true) writeRegisters();                // latch shift register. Motors spin
motorStateChange = false;

delayFlag = false;  // start with the premise we will not be delaying this loop
for (int i=0 ; i < 10; i++) {                // for each motor 
  if (motor[i][motorDeltaMoves] != 0) {      // is it currently moving?
    delayFlag = true;} }                     // and we will have a 1/10th second delay after this loop

if (delayFlag == true) delay(333); //wait about 1/2th second  \

         } // end of for loop //  end of shift-in check loop
 
//decrement delta count on each motor that is enabled; if any delta is now at zero, disable that motor
for (int i=0 ; i < 10; i++) {                // for each motor 
  if ((motor[i][motorDeltaMoves] != 0) && (motorRecalibration[i] == false)) {      // is it currently moving and not in recalibration mode?
    motor[i][motorDeltaMoves] --;            // if so, decrement count
    if (motor[i][motorDeltaMoves] == 0) {    // if we are done moving this motor...
      Serial.print("D");
      Serial.print(i);
      Serial.print(" @ ");
      Serial.println(motor[i][motorDeltaMoves]);
      setRegisterPin((3*i),LOW);} }          // disable motor bit in shift register
}
  writeRegisters();                          // update all motors

     } while (delayFlag == true);

pauseTime(1);  // wait 

// take any motors at the recalibration point and unwind them
     for(byte i = 0; i < 10; i++){  //loop thru each bit from bit positions 6 thru 15 (== units 0 thru 9)
        if(motorRecalibration[i] == true) {       //if the bit equal to 1 (true) and recalibration flag is false, then
          //motorRecalibration[i] = true;  //set this unit's recalibration flag to true                                            
          //Serial.print(i);
          //Serial.println(" -1");    //we are at position -1 and need to unwind until the switch turns off
          //setRegisterPin((3*i),LOW);           // disable motor bit in shift register
          //writeRegisters();                       // latch shift register. 
          motorDirectionDown(i);                  // motor will begin to go down
          setRegisterPin((3*i),HIGH);           // enable motor bit in shift register     
          motor[i][motorDeltaMoves] = 5;  // ensure we keep this loop active by keeping xxx check 99
          motorStateChange = true;
          ballGoTo(i+1, green); }          // go directly to red. motors are addressed 1 thru 10, not 0-9                            
      }
        writeRegisters();  // set motors that are at recalibration point heading down
      
// when motor has stopped tripping the switch, stop it

do {
    for (byte l=3; l>0; l--) {            // check ball status 3 times per second
      pinValues165 = read_shift_regs165();// read 165 shift registers
      
      //put the bits for pinValues165 into a simple array            
      //Serial.print(">");
      for(byte i = 6; i < DATA_WIDTH; i++){  //loop thru each bit from bit positions 6 thru 15 (== units 0 thru 9)
        if((pinValues165 >> i) & true){
          switchValue[i-6] = true;
        }else{
          switchValue[i-6] = false;
      } }
      
     
      for(byte i = 0; i < 10; i++){  //loop thru each bit from bit positions 6 thru 15 (== units 0 thru 9)

        if((switchValue[i] == false) && (motorRecalibration[i] == true)) {       // if the bit equal to 0 (false) and recalibration flag is true, then
                                            // we have just finished an unwind to position index 0
          motorRecalibration[i] = false;  // set this unit's recalibration flag to false                                            
          Serial.print(i);
          Serial.println(" 0:D");    // we are at position 0 and need to go to the ceiling
          motor[i][motorTargetIndex] = 0;
          motor[i][motorDeltaMoves] = 0; //motor[i][motorTargetIndex] - motor[i][motorCurrentIndex] - 1; //move this many steps: target - Current - 1
          setRegisterPin((3*i),LOW);           // disable motor bit in shift register                                 
          //writeRegisters();                       // latch shift register. 
          motorStateChange = true;
          ballFadeTo(i+1, blue);}             // motors are addressed 1 thru 10, not 0-9          
    }

    }
    
    
    if (motorStateChange == true) writeRegisters();                // latch shift register. Motors spin
motorStateChange = false;

delayFlag = false;  // start with the premise we will not be delaying this loop
for (int i=0 ; i < 10; i++) {                // for each motor 
  if (motor[i][motorDeltaMoves] != 0) {      // is it currently moving?
    delayFlag = true;} }                     // and we will have a 1/10th second delay after this loop

if (delayFlag == true) delay(333); //wait about 1/2th second  \

} while (delayFlag == true); */
}
//------------------------------------
void displayFullTime(){
   lcd.setCursor(0, 1);
   lcd.print("Time");
   printLCDDigits(RTC.getHours()); 
   printLCDDigits(RTC.getMinutes());
   printLCDDigits(RTC.getSeconds());
   lcd.print(" ");  // this solves a problem where a digit sometimes gets printed to the right of the time - clear it out instead
}
//------------------------------------
void printPositionDigits(int digits){   // utility function for lcd display: prints leading 0's
  if(digits < 100) lcd.print('0');
  if(digits < 10) lcd.print('0');
  lcd.print(digits);
}
//------------------------------------
void printLCDDigits(int digits){   // utility function for lcd display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10) lcd.print('0');
  lcd.print(digits);
}
//------------------------------------
void recalibrate() {       // Recalibrate
      clearLine(0);
      getMessage(activeMenuSelection);
      lcd.setCursor(0, 0);
      lcd.print(">");                                     //highlight active menu item
      lcd.print(buffer);
  printStatusMessage(msgRecalibrate);
  ballFadeTo(allBlinkMs, yellow);

  //first, go to what we think of is the ceiling
	for (int i=0 ; i < 10; i++) {   // assign value to all motor targetIndex
		motor[i][motorTargetIndex] = ceilingIndex;
             };
  moveTo(); // now go to the ceiling

  // next, for each ball in turn, do a recalibration
	for (int i=0 ; i < 10; i++) {   // assign value to all motor targetIndex
				motor[i][motorCurrentIndex] = (floorIndex + 5);	// limit rewind amount
				motor[i][motorTargetIndex] = 0;
				ballFadeTo(i+1, yellow);
				moveTo();               // go as high as we can till we hit the switch
				ballFadeTo(i+1, blue);
				motor[i][motorTargetIndex] = ceilingIndex;
				moveTo();
             };
 
  //xxx add error code if we leave this and didn't see the ball

  }
//------------------------------------
//clear a specific line
void clearLine(int i){
 for (int j = 0; j <20; j++){
   lcd.setCursor(j,i);
   lcd.print(" ");
 }
}
//------------------------------------
//print message at bottom of lcd
void printStatusMessage(byte i){        //print status message i
      getMessage(i);
      clearLine(3);
      lcd.setCursor(0, 3);
      lcd.print(buffer);
      }      
//------------------------------------
void addMinute(){
  if (RTC.getMinutes() == 59){
    RTC.setMinutes(0);
    if (RTC.getHours() == 23){
      RTC.setHours(0);
   }else{
      RTC.setHours(RTC.getHours()+1);}
   }else{  
      RTC.setMinutes(RTC.getMinutes()+1);}
  RTC.setClock();
  displayFullTime();
}
//------------------------------------
void subtractMinute(){
  if (RTC.getMinutes() == 0){
    RTC.setMinutes(59);
    if (RTC.getHours() == 0){
      RTC.setHours(23);
  }else{
      RTC.setHours(RTC.getHours()-1);}
  }else{  
      RTC.setMinutes(RTC.getMinutes()-1);}
  RTC.setClock();
  displayFullTime();
}    
//------------------------------------
byte addHour(byte k){  // to sleep or wake time
  k++;
  if (k > 23) k = 0;
  lcd.setCursor(5,2);
  printLCDDigits(k);
  return k;
}
//------------------------------------
byte subtractHour(byte k){  // to sleep or wake time
  k--;
  if (k > 250) k = 23; // k might roll from 0 to 256
  lcd.setCursor(5,2);
  printLCDDigits(k);
  return k;
}
//------------------------------------      
void showTime(){
RTC.readClock();  // find out what the time is 
if (RTC.getSeconds() != prevSeconds) {  // if one second has passed...
  displayFullTime(); }                  // display the new time
prevSeconds = RTC.getSeconds();         // save for next time round
}
//------------------------------------  
void clearRegisters(){  //set all register pins to LOW
  for(int i = numOfRegisterPins - 1; i >=  0; i--){
     registers[i] = LOW;
  }
} 
//------------------------------------  
void writeRegisters(){  //Set and display registers - 595's
//Only call AFTER all values are set how you would like (slow otherwise)
//Serial.print("@595 >");

  digitalWrite(RCLK_Pin, LOW);
  for(int i = numOfRegisterPins - 1; i >=  0; i--){
    digitalWrite(SRCLK_Pin, LOW);
    int val = registers[i];
    digitalWrite(SER_Pin, val);
    digitalWrite(SRCLK_Pin, HIGH);
  //  Serial.print(val, BIN);
  }
  digitalWrite(RCLK_Pin, HIGH);
  //Serial.println("<");
}
//------------------------------------  
void setRegisterPin(int index, int value){  //set an individual pin HIGH or LOW
  registers[index] = value;
}
//------------------------------------      
/* This function is a "shift-in" routine reading the
 * serial Data from the 165 shift register chips and representing
 * the state of those pins in an unsigned integer (or long).
 * Uses code from http://playground.arduino.cc/Code/ShiftRegSN74HC165N 
*/
unsigned int read_shift_regs165() {
    byte bitVal165;
    unsigned int bytesVal165 = 0;
    digitalWrite(clockEnablePin165, HIGH);  // Trigger a parallel Load to latch the state of the data lines,
    digitalWrite(ploadPin165, LOW);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(ploadPin165, HIGH);
    digitalWrite(clockEnablePin165, LOW);
    for(int i = 0; i < DATA_WIDTH; i++) // Loop to read each bit value from the serial out line of the SN74HC165N.
    {
        bitVal165 = digitalRead(dataPin165);
        bytesVal165 |= (bitVal165 << ((DATA_WIDTH-1) - i)); // Set the corresponding bit in bytesVal165
        digitalWrite(clockPin165, HIGH);  // Pulse the Clock (rising edge shifts the next bit).
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(clockPin165, LOW);
    }
    return(bytesVal165);
}
//------------------------------------      
//retrieve eeprom message and put it into buffer
void getMessage(byte index){ 
  strcpy_P(buffer, (char*)pgm_read_word(&(menuTable[index]))); // Necessary casts and dereferencing, just copy.
}
//------------------------------------    
void scanfunc( byte addr, byte result ) {  // called when address is found in BlinkM_scanI2CBus()
  Serial.print("addr: ");
  Serial.print(addr,DEC);
  Serial.print( (result==0) ? " found!":"       ");
  Serial.print( (addr%4) ? "\t":"\n");
}
//------------------------------------
//interrupt routine called whenever rotary dial is moved
void updateEncoder(){ 
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
  
  if (encoderValue < 1) encoderValue = 99; //prevent negative numbers on menu
  if (encoderValue > 99) encoderValue = 1; //prevent negative numbers on menu
}
