/*
 * Kinetic Balls -- David Henshaw, December 2012-January 2014
 * v9-999 - Hundreds of other changes thru Janury 2014
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
prog_char menuItem_1[] PROGMEM   = "Falling Stars";
prog_char menuItem_2[] PROGMEM   = "Points";
prog_char menuItem_3[] PROGMEM   = "Angles";
prog_char menuItem_4[] PROGMEM   = "Wave";
prog_char menuItem_5[] PROGMEM   = "Lines";
prog_char menuItem_6[] PROGMEM   = "Color Chandelier";
prog_char menuItem_7[] PROGMEM   = "Romantic";
prog_char menuItem_8[] PROGMEM   = "Party";
prog_char menuItem_9[] PROGMEM   = "Business";
prog_char menuItem_10[] PROGMEM  = "Colors";
prog_char menuItem_11[] PROGMEM  = "Quiet";
prog_char menuItem_12[] PROGMEM  = "Slo-Mo";
prog_char menuItem_13[] PROGMEM  = "White Chandelier";
prog_char menuItem_14[] PROGMEM  = "TEST"; // was: "Turn for settings >";xxx
prog_char menuItem_15[] PROGMEM  = " for settings >>>>>";
prog_char menuItem_16[] PROGMEM  = " settings >>>>>>>>>";
prog_char menuItem_17[] PROGMEM  = "ings >>>>>>>>>>>>>>";

prog_char menuItem_18[] PROGMEM  = "Sleep";
prog_char menuItem_19[] PROGMEM  = "Recalibrate";
prog_char menuItem_20[] PROGMEM  = "Set sleep/wake time";
prog_char menuItem_21[] PROGMEM  = "Set Time";
prog_char menuItem_22[] PROGMEM  = "STOP";
prog_char menuItem_23[] PROGMEM  = "Set ceiling";
prog_char menuItem_24[] PROGMEM  = "Set floor";

prog_char menuItem_25[] PROGMEM  = "Rotate & push to set";  //status messages from this point on
prog_char menuItem_26[] PROGMEM  = "Recalibrating";
prog_char menuItem_27[] PROGMEM  = "Move to ceiling...";
prog_char menuItem_28[] PROGMEM  = "Move to floor...";
prog_char menuItem_29[] PROGMEM  = "Asleep; Turn to wake";
prog_char menuItem_30[] PROGMEM  = "Flip switch to start";
prog_char menuItem_31[] PROGMEM  = "Moving #";
prog_char menuItem_32[] PROGMEM  = "Paused";
// 20 characters maximum:           12345678901234567890

// Then set up a table to refer to the strings:
PROGMEM const char *menuTable[] = {   
  menuItem_0,  menuItem_1,  menuItem_2,  menuItem_3,  menuItem_4,  menuItem_5,  menuItem_6,  menuItem_7,
  menuItem_8,  menuItem_9,  menuItem_10,  menuItem_11,  menuItem_12,  menuItem_13,  menuItem_14,  menuItem_15,
  menuItem_16, menuItem_17, menuItem_18, menuItem_19, menuItem_20, menuItem_21, menuItem_22, menuItem_23,
  menuItem_24, menuItem_25, menuItem_26, menuItem_27, menuItem_28, menuItem_29, menuItem_30, menuItem_31, menuItem_32  };

#define allBlinkMs 255								// sends to the I2C "broadcast" address of 0, so all BlinkMs on the I2C bus will respond (+1 is added)
#define scriptAllBlinkMs 0x00						// I2C "all" broadcast when using native BlinkM calls
#define number_of_74hc595s 4						// How many shift registers
#define numOfRegisterPins number_of_74hc595s * 8	// do not touch
//Next lines for 74hc165:
#define NUMBER_OF_SHIFT_CHIPS   2					// How many shift register chips are daisy-chained.
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8		// Width of data (how many ext lines).
#define PULSE_WIDTH_USEC   5						// Width of pulse to trigger the shift register to read and latch. was: 15

//Constants:
  const int minPauseTime = 120;             // in seconds = 2 minutes = 120 
  const int maxPauseTime = 300;				// in seconds = 5 minutes = 300 
  const byte maxCycles = 10;				// How many loops to run through before an automatic recalibration: normally 10
  const int maxIndex = 250;
  const byte maxMenuItems = 24;				// how many menu items there are - last item is "set floor"
  const int encoderGap = 2;					// size of gap from encoder before we pay attention to it
  const byte msgUseDial = 25;				// message pointers for menu follow...
  const byte msgRecalibrate = 26;  
  const byte msgCeiling = 27;
  const byte msgFloor = 28;
  const byte msgSleeping = 29;
  const byte msgStopped = 30;
  const byte msgMoving = 31;
  const byte msgPause = 32;
  const byte motorCurrentIndex = 0;			// motor array index position for currentIndex
  const byte motorTargetIndex = 1;			// motor array index position for targetIndex
  const byte motorDeltaMoves = 2;			// motor array index position for deltaMoves that motor has to make
  const byte sleepMenuItem = 18;			// which menu selection corresponds to sleep mode?
  const byte quietMood = 11;				// which menu selection corresponds to Sun & Moon mode?
  const byte stopMood = 22;					// which menu selection corresponds to Stop Mode?
  const byte maxAutoRunPrograms = 14;		// when on auto run, when to loop back to program #1 (use value of max mood item + 1)
  const byte maxLightShows = 13;			// maximum number of pre-programmed light shows (used in random light show calls)
  const byte fadeNormal = 15;				// default fade speed
  const byte fadeFast = 50;					// faster fade speed
  const byte fadeSlow =  1;					// slower fade speed
  const byte maxRewind = 60;				// maximum value of rewinding for use in recalibration
  
// Constants for EEPROM memory (refers to index location in EEPROM where we will store the value) using EEPROM.write(address, value) and EEPROM.read(address, value)
  const byte sleepHour_EEPROM = 0;
  const byte wakeHour_EEPROM  = 1;
  const byte ceilingIndex_EEPROM  = 2;              
  const byte floorIndex_EEPROM  = 3;
  
//Variables:
  int SER_Pin = 8;							// pin 14 on the 75HC595 Shift Register
  int RCLK_Pin = 9;							// pin 12 on the 75HC595 Shift Register
  int SRCLK_Pin = 10;						// pin 11 on the 75HC595 Shift Register
  boolean registers[numOfRegisterPins];		// number of 595 registers
  int ploadPin165        = 6;				// Connects to Parallel load (= latch) pin (1) the 165 Shift Register
  int clockEnablePin165  = 13;				// Connects to Clock Enable pin the 165 Shift Register
  int dataPin165         = 5;				// Connects to the Q7 (7) pin the 165 Shift Register
  int clockPin165        = 4;				// Connects to the Clock pin (2)the 165 Shift Register
  int targetIndex;							// Index position we need to be at
  char buffer[20];							// max size of message, and max size of lcd screen
  int encoderPin1 = 2;						// these pins can not be changed 2/3 are special pins
  int encoderPin2 = 3;						// used for interrupts on rotary encoder
  int encoderSwitchPin = 7;					// push button switch on rotary encoder
  const byte masterSwitch = 12;				// master on-off switch at control panel Future enhancement: adjust all pins to const int/byte
  volatile int lastEncoded = 0;				// for interrupt routine - do not change
  volatile int encoderValue = 1;			// for interrupt routine - do not change
  byte activeMenuSelection;					// which menu item is currently selected?
  byte selectedMenuItem = 99;				// value of menu selection at the time the button was pushed
  byte prevSelectedMenuItem = 99;			// previous menu item selection, used for waketime
  int lastEncoderValue = 1;
  byte ceilingIndex;					    // index for ceiling level
  byte floorIndex;							// index for floor level xxx 60
  byte prevSeconds = 99;					// prior value of seconds
  byte prevMinutes = 99;					// prior value of minutes
  int lastMSB = 0;							// for interrupts
  int lastLSB = 0;							// for interrupts
  byte i = 0;								// counter  
  byte sleepHour;							// time to go to sleep
  byte wakeHour;							// time to wake up
  boolean overrideSleepMode = false;		// flag used to over ride sleep mode
  byte loopCounter = 0; 
  unsigned int pinValues165;				// for 165 Shift Register
  boolean delayFlag = false;				// used as a flag to note if the motor loop needs to have a delay in it
  boolean reverseFlag = false;				// used as a flag to denote whether to change direction of the Lines Program
  boolean autoRunFlag = false;				// used to flag if autorun is selected
  boolean doNotRecalibrate = false;
  boolean noMoreChanges = false;			// used in SloMo function
  float sineFrequency = 0;
  int programPauseTime;
  byte sequenceList[]		= {0,1,2,3,4,5,6,7,8,9};	// array used to randomize sequence of motor moves
  byte sequenceColorList[]	= {0,1,2,3,4,5,6,7,8,9};	// array used to randomize sequence of ball colors
  
  byte pointA = 0;							// this section used for selecting random values in programs
  byte pointB = 0;
  byte pointC = 0;
  byte pivotPoint = 0;
  byte gapA = 0;
  byte gapB = 0;

  byte motor[10][3];						// 10 motors, 3 attributes: currentIndex | targetIndex | deltaMoves
  byte motorDestination[10];				// holding place for final destination of motors (used in SloMo function)
  boolean switchValue[10];					// status of switches for each motor

byte color_list[][3] = {
  { 0xff, 0xff, 0xff },						// white = 0
  { 0xff, 0x00, 0xff },						// purple = 1
  { 0xff, 0xff, 0x00 },						// orange = 2
  { 0x00, 0xff, 0xff },						// cyan = 3
  { 0xff, 0x00, 0x00 },						// red = 4
  { 0x00, 0x00, 0xff },						// blue = 5
  { 0xFF, 0xFF, 0x00 },						// yellow = 6
  { 0x00, 0xFF, 0x00 },						// green = 7
  { 0x00, 0x00, 0x00 },						// black = 8
};

byte r;										// red
byte g;										// green
byte b;										// blue

const byte white = 0;						// constants to make it easy to refer to colors in the code
const byte purple = 1;
const byte orange = 2;
const byte cyan = 3;
const byte red = 4;
const byte blue = 5;
const byte yellow = 6;
const byte green = 7;
const byte black = 8;

LiquidCrystal_I2C lcd(0x27,20,4);			// set the LCD address to 0x27 for a 16 chars and 4 line display

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
  
  Serial.begin(19200);						// start serial comms for terminal
  lcd.init();								// initialize the lcd 
  lcd.backlight();							// turn on the backlight
  
  BlinkM_beginWithPower();					// turn on LED
  setFadeSpeed( fadeNormal );
  BlinkM_stopScript( scriptAllBlinkMs );    // turn off startup script
  ballFadeTo(allBlinkMs+1, black );			// turn off any LEDs
  swipeRight(50, white, green);				// turn on all LEDs on at a time
  BlinkM_scanI2CBus(1,100, scanfunc);		// show debug printout of all i2c devices
  swipeLeft(20, green, black);				// visual fun
  ballGoTo(0, green);						// when plugged in, show unit 1 as green so you know it's working
  
//Configure pins:
  pinMode(encoderPin1, INPUT);				// rotary encoder
  pinMode(encoderPin2, INPUT);				// interrupt pins
  pinMode(encoderSwitchPin, INPUT);			// push button
  pinMode(masterSwitch, INPUT);				// master on-off switch
  digitalWrite(encoderPin1, HIGH);			// turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);			// turn pullup resistor on
  digitalWrite(encoderSwitchPin, HIGH);		// turn pullup resistor on
  digitalWrite(masterSwitch, HIGH);			// turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);//call updateEncoder() when any high/low changed seen
  attachInterrupt(1, updateEncoder, CHANGE);//on interrupt 0 (pin 2), or interrupt 1 (pin 3) 

  randomSeed(analogRead(A1));				// random seed based on empty analog pin 1

	//  EEPROM.write( floorIndex_EEPROM, 40 );  only needs to be set once - do not uncomment
	//  EEPROM.write( ceilingIndex_EEPROM, 3 ); only needs to be set once - do not uncomment

	sleepHour =    EEPROM.read( sleepHour_EEPROM );	// refresh these variables as stored in EEPROM
	wakeHour =     EEPROM.read( wakeHour_EEPROM );
	ceilingIndex = EEPROM.read( ceilingIndex_EEPROM );              
	floorIndex =   EEPROM.read( floorIndex_EEPROM );
  
  /*RTC.setHours(11);		// At startup: set time - TEST CODE ONLY
  RTC.setMinutes(56);		// Note: Resetting time can help fix RTC clock errors
  RTC.setClock();*/ 		
} 
//------------------------------------
void randomizeList(byte maxItems = 9) {		// generate a random list of numbers from 0 - maxItems for use in changing motors. Default is maxItems = 9
	byte b;
	byte temp;

	for (int i=0; i < (maxItems+1); i++) {	// randomize the list
	  b = random(i,9);
	  temp = sequenceList[i];
	  sequenceList[i] = sequenceList[b];
	  sequenceList[b] = temp;
	};
}
//------------------------------------
void pauseTime(int k, boolean preventRandomLightShow = false) {		// pause for k seconds, including a countdown
  printStatusMessage(msgPause);
  prevMinutes = RTC.getMinutes();
  for (int i=k ; i > 0; i--) {
    showTime();
    lcd.setCursor(17,3);
    printPositionDigits(i);
    delay(997);														// pause for almost one second

    if ( (selectedMenuItem != sleepMenuItem) &&						// if we're not sleeping and
		(preventRandomLightShow == false) ) {						// if flag isn't set to prevent a random color change

		if ( (RTC.getMinutes() != prevMinutes) &&					// if one minute has passed
			(selectedMenuItem != quietMood) &&
			(selectedMenuItem != 13) ) {							// and we're not in Sun/Moon program or white chandelier
			randomLightShow(random(0,maxLightShows+1));  
			prevMinutes = RTC.getMinutes(); }

		// if on average 6 seconds has passed, cause a random pendant to fade to a random color
		if ((random(0,6) == 5)) {
			byte unit = random(0,10);
			BlinkM_stopScript( unit + 1 );							// turn off any script currently playing
			ballGoTo ( unit, black );
			BlinkM_fadeToRandomRGB( unit + 1 , 0xff, 0xff, 0xff);	// set a random color
		}
    }

    if (dialHasTurned() == true) break;								// exit when dial has turned.
  } 
}
void loop() {
showTime();

//menu selection  
if (abs(lastEncoderValue - encoderValue) > encoderGap) {  //menu selection has changed
  if (encoderValue > lastEncoderValue) {                  //figure out what the new menu selection is
    activeMenuSelection ++;                               // add 1 to menu
    if (activeMenuSelection > maxMenuItems) activeMenuSelection = 0;
  }else{ 
    activeMenuSelection --;                               // subtract 1 from menu
    if (activeMenuSelection > 200) activeMenuSelection = maxMenuItems;
  }
  printLCDActiveMood(activeMenuSelection);     
  lastEncoderValue = encoderValue;
 }


if(digitalRead(encoderSwitchPin)){  // check if button has been pushed 
    switch (selectedMenuItem) {  // button is not being pushed, which means we can execute code based on it being pushed last time round this loop

      case 0:												// auto-run... cycle through multiple programs
        autoRunFlag = true;									// signal that we are in auto run mode  
        break;

	  case 1:												// falling stars program (3 down at any point)		   
        if (prevSelectedMenuItem != selectedMenuItem) recalibrate(); // need to recalibrate only the first time ever here
		loopCounter = 0;  
		BlinkM_playScript(scriptAllBlinkMs, 13, 0, 0);		// play script #13: random blues
		if (dialHasTurned() == true) break;
		dropFallingStar(sequenceList[2]);					// drop pendant @ array #2
		if (dialHasTurned() == true) break;
		dropFallingStar(sequenceList[1]);					// drop pendant @ array #1
		if (dialHasTurned() == true) break;
		dropFallingStar(sequenceList[0]);					// drop pendant @ array #0

        do {
			pauseTime(360,true);							// drop a new pendant every 6 mins (360 seconds)
			if (dialHasTurned() == true) break;
			if (sleepHour == RTC.getHours()) break;			// because this case code re-enters very infrequently, exit if we're at sleep time

			raiseFallenStar(sequenceList[2]);				// raise pendant 2 (include force recalibrate) xxx use routine from setCeiling
			sequenceList[2] = sequenceList[1];				// rotate array... 1 is now 2, 0 is now 1, TBD = 0
			sequenceList[1] = sequenceList[0];

			do {
				sequenceList[0] = random(0,10);				// pick a new pendant to drop
			} while (sequenceList[0] == sequenceList[2] || sequenceList[0] == sequenceList[1]);	// ...as long as no duplicates
			dropFallingStar(sequenceList[0]);				// drop pendant @ array #0

			if (dialHasTurned() == true) break;
			loopCounter ++;
			if (loopCounter > (maxCycles * 5)) break;		// exit this case statement; we'll come around and do it again after a recalibration

            } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.           
		  break;
        
      case 2:   // random points program
        recalibrate(); 
        loopCounter = 0;  
		randomizeList();													// call this and refer to sequenceList[i] to get random order of motor moves
        do {
           for (int i=0 ; i < 10; i++) { 
				motor[sequenceList[i]][motorTargetIndex] = programTargetIndex(ceilingIndex, floorIndex);  // find a random place to go for each ball
				fadeRight(150, yellow, cyan);
				BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);	// set a random color
				ballFadeTo((sequenceList[i]), cyan);
				moveTo();
				BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);	// set a random color
				programPauseTime = random(minPauseTime, maxPauseTime);      // in seconds - wait before moving
				pauseTime(programPauseTime); 
				if (dialHasTurned() == true) break;
           }          
                    
          loopCounter ++;
          if (loopCounter > maxCycles) break;								// exit this case statement; we'll come around and do it again after a recalibration
		  if (dialHasTurned() == true) break;

        } while (abs(lastEncoderValue - encoderValue) < encoderGap);		// exit when dial has turned. Note < operation.
        break;
                
      case 3:  // angles
		recalibrate(); 
        loopCounter = 0; 

        do {      
			setupAngles();
          
			for (int i = 1; i < pivotPoint; i++) {						// program units 1 thru pivotPoint (exclusive)
				motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA);
				ballFadeTo(i, white);
				moveTo();
				if (dialHasTurned() == true) break;
				curtainsOut(100, white, black);
				BlinkM_fadeToRandomRGB(scriptAllBlinkMs,0xff,0xff,0xff);// set a random color 
			}
			for (int i = pivotPoint; i < 9; i++) {						// program units pivotPoint (inclusive) thru 10
				motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] - gapB);
				ballFadeTo(i, white);
				moveTo();
				if (dialHasTurned() == true) break;
				curtainsOut(100, white, black);
				BlinkM_fadeToRandomRGB(scriptAllBlinkMs,0xff,0xff,0xff);// set a random color 
			}
         
          loopCounter ++;
          if (loopCounter > maxCycles) break;
		  if (dialHasTurned() == true) break;

		  for (byte i = random(3,6) ; i > 0; i--) {						// some random number of times
			BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);	// set a random color
			programPauseTime = random(5, minPauseTime);					// in seconds
			pauseTime(programPauseTime);                                // pause
			if (dialHasTurned() == true) break;  
			}                            
		
		  if (dialHasTurned() == true) break;

        } while (abs(lastEncoderValue - encoderValue) < encoderGap);	// exit when dial has turned. Note < operation.
        break;

      case 4:  // waves
       recalibrate();
       sineFrequency = random(1,11);									// pick a random sine wave multiplier from 1 to 10
       sineFrequency = sineFrequency/10;								// convert to a number between 0.1 and 1.0
       loopCounter = 1;
       
       pointA = calculateSinPosition(loopCounter, sineFrequency);		// pointA is the value for unit 10
       
       for (int i=0; i < 10; i++) {										// first time round, prep each target
			motor[i][motorTargetIndex] = calculateSinPosition(loopCounter, sineFrequency);
			loopCounter++;
	   }
       
       do{
         randomLightShow(7);											// play a random script 
         moveTo();
		 if (dialHasTurned() == true) break;            
         curtainsOut(100, white, black);
		 for (byte i = random(5,9) ; i > 0; i--) {						// some random number of times
			randomLightShow(7);											// play a random script 
			programPauseTime = random(5, minPauseTime);					// in seconds
			pauseTime(programPauseTime);                                // pause
			if (dialHasTurned() == true) break;  
			}    
         
         for (int i=0 ; i < 9; i++) {									// shift all values to the left by one unit
           motor[i][motorTargetIndex] = motor[i+1][motorTargetIndex]; }
         
         loopCounter ++;
         if (loopCounter > (maxCycles * 3)) break;						// xxx check this size loop
         if (dialHasTurned() == true) break;  

         pointA = calculateSinPosition(loopCounter, sineFrequency);
         motor[9][motorTargetIndex] = pointA;							// seed unit 10 with the new value
        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);	// exit when dial has turned. Note < operation.
        break;

      case 5:  // Lines
        recalibrate(); 
        reverseFlag = ~reverseFlag;  // flip from false-true
        loopCounter = 0; 

         do {      
          pointA = programTargetIndex( ceilingIndex, ((floorIndex - ceilingIndex) * 0.2) ); // PointA: find a random point between the ceiling and top 20% from the ceiling
          pointB = programTargetIndex( ((floorIndex - ceilingIndex) * 0.5), floorIndex );	// PointB: find a random point between midway and the floor
           
          gapA = (pointB - pointA) / 8 ;													// gapA gaps between pointA and pointB is (pB-pA)/82
         
          if (reverseFlag == false) {														// line goes from left to right      
            motor[0][motorTargetIndex] = pointA;
            motor[9][motorTargetIndex] = pointB;          
            for (int i = 1; i < 9; i++) {													// program units 1 thru 9 (exclusive)
				motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA);
				randomLightShow(7);															// random light script
				ballFadeTo(i, white);
				moveTo();
				if (dialHasTurned() == true) break;
				curtainsOut(100, white, black);
			}
          } else {																			// line goes from right to left
            motor[9][motorTargetIndex] = pointA;
            motor[0][motorTargetIndex] = pointB;          
            for (int i = 8; i > 0; i--) {													// program units 1 thru 9 (exclusive)
				motor[i][motorTargetIndex] = (motor[(i+1)][motorTargetIndex] + gapA);
				randomLightShow(7);															// random light script
				ballFadeTo(i, white);
				moveTo();
				if (dialHasTurned() == true) break;
				curtainsOut(100, white, black);
			} 
          }
                      
		  if (dialHasTurned() == true) break;         
                   
          loopCounter ++;
          if (loopCounter > maxCycles) break; 

		 for (byte i = random(3,6) ; i > 0; i--) {						// some random number of times
			BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);	// set a random color
			programPauseTime = random(5, minPauseTime);					// in seconds
			pauseTime(programPauseTime);                                // pause
			if (dialHasTurned() == true) break;  
		}
          if (dialHasTurned() == true) break;
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);	// exit when dial has turned. Note < operation.
               
        break;
        
      case 6:  // Light show 
    //   setUpChandelier();												// adopt chandelier shape xxx add back in!
       randomLightShow(random(0,maxLightShows+1));						// do something flashy
       randomLightShow(7);												// then play a random script 
       programPauseTime = random(minPauseTime, maxPauseTime);			// in seconds - wait
       pauseTime(programPauseTime); 
       break;
       
      case 7:  // "Romantic"
       recalibrate();
       sineFrequency = random(1,11);								// pick a random sine wave multiplier from 1 to 10
       sineFrequency = sineFrequency/10;							// convert to a number between 0.1 and 1.0 xxx merge with previous wave code
       loopCounter = 1;												// same as "current step"
       pointA = calculateSinPosition(loopCounter, sineFrequency);	// pointA is the value for unit 10 xxx merge
       
	   ballFadeTo(allBlinkMs,blue);
       for (int i=0; i < 10; i++) {									// first time round, populate all units
			motor[i][motorTargetIndex] = calculateSinPosition(loopCounter, sineFrequency);
			loopCounter++;
			ballFadeTo(i,white);
			moveTo();												// move units one at a time
			BlinkM_playScript(i+1,12, 0, 0);						// play candles script - uses ball number 1 thru 10
			if (dialHasTurned() == true) break;
	   }  
       
       do{             
         if (dialHasTurned() == true) break;

         for (int i=0 ; i < 9; i++) {								// shift all values to the left by one unit
			motor[i][motorTargetIndex] = motor[i+1][motorTargetIndex];
			programPauseTime = random(15, minPauseTime);			// in seconds - wait before moving
			pauseTime(programPauseTime);							// true = do not allow random light show when minute changes
			curtainsOut(50, yellow, cyan);
			BlinkM_playScript(scriptAllBlinkMs,12, 0, 0);			// play candles script
			ballFadeTo(i, white);
			moveTo();
			if (dialHasTurned() == true) break;						// line to allow break from loops if dial is turned
			ballFadeTo(i, black);									// turn this ball off
			pauseTime(2, true);										// wait 2 seconds
			BlinkM_playScript( i+1 ,12, 0, 0);						// play candles script
         }
         
         loopCounter ++;
         if (loopCounter > maxCycles) break;						// xxx check this size loop 10 or 20?
		 if (dialHasTurned() == true) break;
              
         pointA = calculateSinPosition(loopCounter, sineFrequency);
         motor[9][motorTargetIndex] = pointA;						// seed unit 10 with the new value        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);// exit when dial has turned. Note < operation.
        break;      
       
      case 8:  // "Party"
	   recalibrate(); 
       sineFrequency = random(4,15);									// pick a random sine wave multiplier from 1 to 14
       sineFrequency = sineFrequency/10;								// convert to a number between 0.4 to 1.4
       loopCounter = 1;
       
       pointA = calculateSinPosition(loopCounter, sineFrequency);		// pointA is the value for unit 10
       
       for (int i=0; i < 10; i++) {										// first time round, prep each target for the first sin wave
			motor[i][motorTargetIndex] = calculateSinPosition(loopCounter, sineFrequency);
			loopCounter++;
	   }
       
       do{
         transition(random(0,10));										// play random transition xxx check max value
		 playScript(random(1, 5));										// play a random script  case options 1, 2, 3, 4 (scripts 11 12 13 14)
         moveTo();
		 if (dialHasTurned() == true) break;            
         
		 for (byte i = random(12,21) ; i > 0; i--) {					// some random number of times (approx 6 to 10 minute loops)
			transition(random(0,10));									// play random transition xxx check max value
			pauseTime(1);
			byte scriptNumber = random(0,15);

			clearLine(2);
			lcd.setCursor(0,2);											// line 2 column 0
			lcd.print("Sin=");
			lcd.print(sineFrequency);									// print value of sineFrequency xxx need to clear this out later
			lcd.print(" Scr=");
			lcd.print(scriptNumber);

			playScript(scriptNumber);									// play a random script  xxx check max value
			if (scriptNumber < 9) {
				pauseTime(30);											// pause for a long time
			} else { 
				pauseTime(4);											// pause for a short time
			}

			if (dialHasTurned() == true) break;  
			}
         
         for (int i=0 ; i < 9; i++) {									// shift all values to the left by one unit
           motor[i][motorTargetIndex] = motor[i+1][motorTargetIndex]; }
         
         loopCounter ++;
         if (loopCounter > (maxCycles * 3)) break;						// xxx check this size loop
         if (dialHasTurned() == true) break;  

         pointA = calculateSinPosition(loopCounter, sineFrequency);
         motor[9][motorTargetIndex] = pointA;							// seed unit 10 with the new value
        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);	// exit when dial has turned. Note < operation.
        break;

      case 9:  // "Business"
		recalibrate(); 
        loopCounter = 0; 

        do {      
			setupAngles();
			for (int i = 1; i < pivotPoint; i++) motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA); // program units 1 thru pivotPoint (exclusive)
			for (int i = pivotPoint; i < 9; i++) motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] - gapB); // program units pivotPoint (exclusive) thru 10
			moveTo();													// go to the specified angular shape
			if (dialHasTurned() == true) break; 

			loopCounter ++;
			if (loopCounter > maxCycles) break;

			for (int i = 0; i < random(15,31); i++) {					// between 15 and 30 times...
				fadeRightSlow(random(4,10));							// fade with random colors with random delay (4-9 seconds)
				if (dialHasTurned() == true) break; 
			}                             
			
			if (dialHasTurned() == true) break; 
			swipeLeft(50,black,blue);
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);	// exit when dial has turned. Note < operation.
        break;   
      
      case 10:  // colors
		  
		pauseTime(10);	// random color changes
		break;
       
      case 11:  // "Auto Quiet - Sun & Moon" (Not affected by auto sleep)
        setUpChandelier();											// adopt chandelier shape if required
		fillIn(60, black);											// random fill to black
        i = determineTimeOfDay(RTC.getHours());						// what part of the day is it?

        if (i > 1) {
			BlinkM_playScript(scriptAllBlinkMs, i, 0, 0);			// Args: blinkM addr, script#, 0=repeat forever, 0=play from beginning
			pauseTime(60);											// no rush, let this linger for a minute
		}
        
        if (i == 0) {												// if night...
			setFadeSpeed( fadeSlow );								// default
			for (int i = 0; i < 10; i++) {
              BlinkM_fadeToHSB((i+1), 43, 0, 0x2d);					// white (43) dim   
              pauseTime(1, true);
              ballFadeTo(i, black);
              pauseTime(5, true);
              if (dialHasTurned() == true) break;
          }
        }
        
        if (i == 1) {												// if afternoon...
			BlinkM_fadeToHSB(scriptAllBlinkMs, 43, 0x00, 0xff);		// white (43) bright      
			BlinkM_fadeToRandomHSB(scriptAllBlinkMs, 0, 10, 0xc8);	// no change in hue, 0 sat and random brightness +/- 200
			pauseTime(60);											// no rush, let this linger for a minute
        }
		setFadeSpeed( fadeNormal );									// default

      break;														// and then, allow to repeat
      
       
      case 12:  // SloMo
		recalibrate(); 
        loopCounter = 0; 

        do {      
			setupAngles();
			for (int i = 1; i < pivotPoint; i++) motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] + gapA); // program units 1 thru pivotPoint (exclusive)
			for (int i = pivotPoint; i < 9; i++) motor[i][motorTargetIndex] = (motor[(i-1)][motorTargetIndex] - gapB); // program units pivotPoint (exclusive) thru 10
			for (int i = 0; i < 10; i++) {
				motorDestination[i] = motor[i][motorTargetIndex];			// move targets to another array that will hold the "final" destination
				motor[i][motorTargetIndex] = motor[i][motorCurrentIndex];	// set target to be same as current
			}
			
			do {
				curtainsOut(100, white, black);
				
				noMoreChanges = true;										// start with the premise that no more changes will be required
				for (int i = 0; i < 10; i++) {								// for each motor, does it need to adjust?
					if (motor[i][motorCurrentIndex] > motorDestination[i]){ // this motor needs to move up one
						motor[i][motorTargetIndex] = (motor[i][motorCurrentIndex] - 1);
						noMoreChanges = false;
						BlinkM_playScript(scriptAllBlinkMs, 15, 0, 0);				// 15: The Seasons script
						ballFadeTo( i, white);
						moveTo();											// go to the specified angular shape
						if (dialHasTurned() == true) break; 
						programPauseTime = random(15, minPauseTime);			// in seconds - wait before moving
						pauseTime(programPauseTime);							// true = do not allow random light show when minute changes
					}
					if (motor[i][motorCurrentIndex] < motorDestination[i]){ // this motor needs to move down one
						motor[i][motorTargetIndex] = (motor[i][motorCurrentIndex] + 1);
						noMoreChanges = false;
						BlinkM_playScript(scriptAllBlinkMs, 15, 0, 0);				// 15: The Seasons script
						ballFadeTo( i, yellow);
						moveTo();											// go to the specified angular shape
						if (dialHasTurned() == true) break;
						programPauseTime = random(15, minPauseTime);			// in seconds - wait before moving
						pauseTime(programPauseTime);							// true = do not allow random light show when minute changes
					}
					if (dialHasTurned() == true) break; 
				}
				if (dialHasTurned() == true) break; 
			} while (noMoreChanges == false);

			loopCounter ++;
			if (loopCounter > maxCycles) break;   
			if (dialHasTurned() == true) break; 
        
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);		// exit when dial has turned. Note < operation.
        break;  
      
      break;

	  case 13:		// white chandelier
		setUpChandelier();											// adopt chandelier shape
		ballFadeTo( allBlinkMs , white);
		pauseTime(5);												// pause with very few random color changes

	  break;

	  case 14:		// temporary test program
		playScript(14);
		pauseTime(5);
		break;
              
      case 18:   // sleep mode
        //recalibrate();											// not doing a recalibrate on sleep mode any longer.
		printLCDActiveMood(selectedMenuItem);						// will display "Sleep" even if automatically selected
        clearLine(2);												// Set message on LCD
		lcd.setCursor(0,2);
		lcd.print("Wake at");
		printLCDDigits(wakeHour);
		lcd.print(" hrs");
		printStatusMessage(msgSleeping);
 
        do {
          for (int i = 0; i < 10; i++) {
              ballFadeTo(i, white);
              pauseTime(1, true);
			  if((digitalRead(encoderSwitchPin)) == false) break;	// switch has been pressed
              ballFadeTo(i, black);
              pauseTime(5, true);
              if (dialHasTurned() == true) break;
			  if((digitalRead(encoderSwitchPin)) == false) break;	// switch has been pressed
          }
         
             if (wakeHour == RTC.getHours()) break;
			 if (dialHasTurned() == true) break;
			 if((digitalRead(encoderSwitchPin)) == false) break;	// switch has been pressed
               
        } while (abs(lastEncoderValue - encoderValue) < encoderGap);// exit when dial has turned. Note < operation.         
        

        selectedMenuItem = 99;										// prevent this case from being selected next time round
		overrideSleepMode = true;									// ignore sleep mode until the next wakeHour
        if ((wakeHour == RTC.getHours()) || (dialHasTurned() == true)){ // if user flips switch from on to off
			selectedMenuItem = prevSelectedMenuItem;				// remember what the previously-running program was
			printLCDActiveMood(selectedMenuItem);					// will display the previously-running mood
		}
		clearLine(2);
        break;
      
      case 19:  //recalibrate  
        recalibrate();
        selectedMenuItem = 99;  // prevent this case from being selected next time round        
        break;     
       
      case 20:  // set sleep and wake time 
        printStatusMessage(msgUseDial);
        lcd.setCursor(0, 2);
        lcd.print("Sleep");
		sleepHour = addHour(sleepHour);								// force first-time display of hour by adding then subtracting
		sleepHour = subtractHour(sleepHour);
        do{															// let's set sleep hour...
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {
                sleepHour = addHour(sleepHour);
            }else{ 
                sleepHour = subtractHour(sleepHour);
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin));					// exit when the button is pressed to set time        
        EEPROM.write( sleepHour_EEPROM, sleepHour );				// update EEPROM
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print("Wake");
		wakeHour = addHour(wakeHour);								// force first-time display of hour by adding then subtracting
		wakeHour = subtractHour(wakeHour);
        delay(500);													// slight pause to allow time for button to be released
        do{															// let's set wake hour...
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {
                wakeHour = addHour(wakeHour);
                if (wakeHour == sleepHour) wakeHour = addHour(wakeHour);		// if wake and sleep are the same, increment wake
            }else{ 
                wakeHour = subtractHour(wakeHour);
                if (wakeHour == sleepHour) wakeHour = subtractHour(wakeHour);	// if wake and sleep are the same, decrement wake
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin));					// exit when the button is pressed to set time   
        EEPROM.write( wakeHour_EEPROM, wakeHour );					// update EEPROM
        clearLine(2);
        selectedMenuItem =99;										// prevent this case from being selected next time round        
        break;
       
      case 21: //set time
        printStatusMessage(msgUseDial);
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {				// figure out what the new menu selection is
                addMinute();
            }else{ 
                subtractMinute();
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin));					// exit when the button is pressed to set time
        selectedMenuItem =99;										// prevent this case from being selected next time round        
        clearLine(2);
        clearLine(3);
       break; 
              
      case 22:  //stop/resume - do not recalibrate - just stop whatever is happening right now
        ballFadeTo(allBlinkMs, red);
		lcd.init();
		lcd.noDisplay();											// turn off LCD in case it's garbled
		pauseTime(1);
		lcd.display();												// turn on LCD
		printStatusMessage(msgStopped);
		swipeLeft(150, red, black);
        do {
          showTime();
          delay(900);												// just wait until a resume is requested
        } while (digitalRead(masterSwitch) == HIGH);				// continue while switch is off = high
		clearLine(3);
		swipeRight(45, black, blue);
		selectedMenuItem = prevSelectedMenuItem;					// when we exit, return to the previous program
		printLCDActiveMood(selectedMenuItem);						// restore name of mood
		lcd.setCursor(0, 0);
		lcd.print(">");
        break;
  
      case 23:  // Set Ceiling 
        recalibratePendant(0);										// first, recalibrate pendant
        printStatusMessage(msgUseDial);
        motor[0][motorTargetIndex] = ceilingIndex; 
        moveTo();
        
        do{
          encoderHeightChange();
          lcd.setCursor(0, 2);										// print ceiling on lcd
          lcd.print("Ceiling:");
          printPositionDigits(motor[0][motorTargetIndex]);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();
        
        } while (digitalRead(encoderSwitchPin));					// exit when the button is pressed to set hours
        ceilingIndex = motor[0][motorCurrentIndex];					// new value for ceiling
        EEPROM.write( ceilingIndex_EEPROM, ceilingIndex ); 
        clearLine(2);
        clearLine(3);
        selectedMenuItem = 99;										// prevent this case from being selected next time round        
        break;
        
      case 24:  // Set floor
        recalibratePendant(0);										// first, recalibrate pendant
        printStatusMessage(msgUseDial);
        motor[0][motorTargetIndex] = floorIndex; 
        moveTo();        
        
       do{
          encoderHeightChange();
          lcd.setCursor(0, 2);										// print on lcd
          lcd.print("Floor:");
          printPositionDigits(motor[0][motorTargetIndex]);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();           
          
        } while (digitalRead(encoderSwitchPin));					// exit when the button is pressed to set hours
        floorIndex = motor[0][motorCurrentIndex];					// new value for floor
        EEPROM.write( floorIndex_EEPROM, floorIndex ); 
        clearLine(2);
        clearLine(3);
        selectedMenuItem = 99;										// prevent this case from being selected next time round
        break;
        
    }															// end of case select
    
	delay(100);													// gives time for button to be released  
	prevSelectedMenuItem = selectedMenuItem;					// remember what the currently-running program is for wake time

	if (digitalRead(masterSwitch) == HIGH) selectedMenuItem = stopMood;// high = off position; do STOP mode (17)

    if (abs(lastEncoderValue - encoderValue) > encoderGap){		// the dial has just been turned
     autoRunFlag = false;										// signal that we are not in auto run mode     
     selectedMenuItem = 99;}									// prevent any case from being selected next time round if dial was turned
    
	if (wakeHour == RTC.getHours()) overrideSleepMode = false;	// if it's time to wake up, stop the sleep time override, if it's in effect

    if ((sleepHour == RTC.getHours()) &&						// if it's sleep hour
		(selectedMenuItem != quietMood) &&						// and we're not in quiet Mood
		(overrideSleepMode == false)) 							// and override sleep mode isn't set
      selectedMenuItem = sleepMenuItem; 						// go to sleep
      
    if (autoRunFlag == true){									// we're in auto-run mode and need to cycle through each program in turn
      selectedMenuItem ++;										// if it was 0 (auto run selected) now it will be 1
      if (selectedMenuItem >= maxAutoRunPrograms) selectedMenuItem = 1; } // next time round, the correct program will run
    
  }else{														// button is being pushed
    lcd.setCursor(0, 0);
    lcd.print(">");												// indicate to user it has been pushed
    selectedMenuItem = activeMenuSelection;
    BlinkM_stopScript( scriptAllBlinkMs );						// turn off any script currently playing
  }
}																// end of loop
//------------------------------------
void printLCDActiveMood(byte menu) {	// display the currently-running mood / mode
      clearLine(0);
	  clearLine(3);
      getMessage(menu);
	  lcd.setCursor(0, 0);
	  lcd.print(">");	
      lcd.print(buffer);
}
//------------------------------------
void randomizeColorList() {   // generate a random list of numbers from 0 - 9 for use in changing ball colors
	byte b;
	byte temp;

	for (int i=0; i<10; i++) { // randomize the list
	  b = random(i,10);
	  temp = sequenceColorList[i];
	  sequenceColorList[i] = sequenceColorList[b];
	  sequenceColorList[b] = temp;
	};
}
//------------------------------------
boolean dialHasTurned(){	// set flag if dial was turned or switch set to on
	boolean dialChange = false;
	/*if (abs(lastEncoderValue - encoderValue) > encoderGap) dialChange = true;	// dial has been turned
	if (digitalRead(masterSwitch) == HIGH) dialChange = true;	// Master Switch flipped to high = off; use to force stop mode if master switch is turned to "off"
	    if ((sleepHour == RTC.getHours()) &&						// if it's sleep hour
		(selectedMenuItem != quietMood) &&						// and we're not in quiet Mood
		(overrideSleepMode == false)) 							// and override sleep mode isn't set
      dialChange = true; 						// set to true so we can break and then go to sleep
	  */

	if ((abs(lastEncoderValue - encoderValue) > encoderGap) ||	// dial has been turned OR
		(digitalRead(masterSwitch) == HIGH) ||					// Master Switch flipped to high = off OR
	    ((sleepHour == RTC.getHours()) &&						// if it's sleep hour
		(selectedMenuItem != sleepMenuItem) &&						// and we're not already in sleep mode
		(selectedMenuItem != quietMood) &&						// and we're not in quiet Mood
		(overrideSleepMode == false)) )							// and override sleep mode isn't set
      dialChange = true; 										// then set to true so we can break and change dial/go to off/go to sleep
	return dialChange;
}
//------------------------------------
void pwmMotor(byte motor, byte offDuty, byte onDuty, byte loops, boolean overRide = false){
	for (byte i = 0; i < loops; i++) {			  
		setRegisterPin((3*motor), LOW );			// OFF
		writeRegisters();
		delay(offDuty);
		setRegisterPin((3*motor), HIGH );			// ON
		writeRegisters();

		pinValues165 = read_shift_regs165();		// read 165 shift registers to detect microswitch
		if((pinValues165 >> (motor + 6)) & true){	// need to adjust by 6 because of circuit design
			switchValue[motor] = true;				// this motor has reached recalibration point
			if (overRide == false) break;			// exit immediately unless we are forcing a slow down
			} else {
				switchValue[motor] = false;}
				delay(onDuty);
		  }
}
//------------------------------------
byte determineTimeOfDay(byte i){		// i = current hour
 byte script = 0;						// default = moon program
 if (i >  3) script = 12;				//  4am+ random yellows
 if (i >  7) script = 11;				//  8am+ random hues
 if (i > 10) script =  1;				// 11am+ white program
 if (i > 16) script = 13;				//  5pm+ random blues
 if (i > 18) script = 14;				//  7pm+ random orangeish reds
 if (i > 21) script =  0;				// 10pm+ moon program
 BlinkM_stopScript( scriptAllBlinkMs );	// turn off any script currently playing
 return script;							// script = 0 or 1 for special program; or number of script to play on BlinkM
}
//------------------------------------
void setUpChandelier() {
	if (doNotRecalibrate == false) { // first time here, so we do need to recalibrate this one time
		recalibrate();
          
        // go to the chandelier position...
        pointA = ceilingIndex;									// PointA: both edges of chandelier at ceiling height
        pointB = ((floorIndex - ceilingIndex) * 0.4);			// PointB: 40% of the way down                   
        gapA = pointB - pointA ;
       
        motor[0][motorTargetIndex] = pointA;					// top
        motor[1][motorTargetIndex] = pointA + (gapA * 0.3);
        motor[2][motorTargetIndex] = motor[1][motorTargetIndex] + (gapA * 0.2);
        motor[3][motorTargetIndex] = motor[2][motorTargetIndex] + (gapA * 0.1);        
        motor[4][motorTargetIndex] = pointB;					// lowest point
        motor[5][motorTargetIndex] = pointB;					// lowest point
        motor[6][motorTargetIndex] = motor[3][motorTargetIndex];
        motor[7][motorTargetIndex] = motor[2][motorTargetIndex];
        motor[8][motorTargetIndex] = motor[1][motorTargetIndex];                
        motor[9][motorTargetIndex] = pointA;					// top
       
        BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);// set a random color
        moveTo();
        doNotRecalibrate = true;								// set flag so we don't recalibrate next time around
        } 
}
//------------------------------------
void randomLightShow(byte i) {  
 int lightSpeed = 0;
 byte lightColor1;
 byte lightColor2;
 lightSpeed = random(50,200);
 lightColor1 = random(0,9); // between colors 0 and 8
 lightColor2 = random(0,8); // between colors 0 and 7 (excludes 8=black)
 
 BlinkM_stopScript( scriptAllBlinkMs );    // stop all scripts from playing
 
 switch (i) {  // 

        case 0:    //
          christmasLights((lightSpeed+200), lightColor1, lightColor2, random(5,16));
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
          BlinkM_playScript(scriptAllBlinkMs,(random(10,17)), 0, 0); //   Args: blinkM addr, script#, 0=repeat forever, 0=play from beginning      
        break; 

        case 8:    //
          BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);      // set a random color        
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
			fillIn(100, lightColor2); // prevents fade to black
        break;

		case 12: // ten individual white flashes
			ballFadeTo(allBlinkMs, lightColor2); // prevents fade to black
			pauseTime(5);
			randomizeColorList();
			for (int i=0 ; i < 10; i++) { 
				ballGoTo(sequenceColorList[i], white);   
				delay(50);					// slight pause for effect
				ballGoTo(sequenceColorList[i], lightColor2);
				delay(lightSpeed);          // slight pause for effect
			} 
		break;

		case 13: // ten multiple white flashes
			ballFadeTo(allBlinkMs, lightColor2); // prevents fade to black
			pauseTime(5);
			randomizeColorList();
			for (int i=0 ; i < 10; i++) {
				for (int j=0 ; j < random(2,6); j++){
					ballGoTo(sequenceColorList[i], white);   
					delay(35);					// slight pause for effect
					ballGoTo(sequenceColorList[i], lightColor2);
					delay(lightSpeed);          // slight pause for effect
				} 
			delay(lightSpeed);
			}
		break;

       } // end of switch loop                      
}
//------------------------------------
void transition(byte i) {  
 int lightSpeed = 0;
 byte lightColor1;
 byte lightColor2;
 lightSpeed = random(50,200);
 lightColor1 = random(0,9); // between colors 0 and 8
 lightColor2 = random(0,8); // between colors 0 and 7 (excludes 8=black)
 
 BlinkM_stopScript( scriptAllBlinkMs );    // stop all scripts from playing
 
 switch (i) {  // 

        case 0:    //
          christmasLights((lightSpeed+200), lightColor1, lightColor2, random(5,16));
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

        case 7:    
			BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);      // set a random color  
        break; 

        case 8:    //
            fadeRight(lightSpeed, lightColor1, lightColor2);    
        break; 

        case 9:    //
            fillIn(100, lightColor2); // prevents fade to black  
        break; 

        case 10:    // 

        break;
        
        case 11: // 
			
        break;

		case 12: // 

		break;

		case 13: //

		break;

       } // end of switch loop                      
}
//------------------------------------
void playScript(byte i) {  
 int lightSpeed = 0;
 byte lightColor1;
 byte lightColor2;
 lightSpeed = random(50,200);
 lightColor1 = random(0,9); // between colors 0 and 8
 lightColor2 = random(0,8); // between colors 0 and 7 (excludes 8=black)
 
 BlinkM_stopScript( scriptAllBlinkMs );    // stop all scripts from playing


 
 switch (i) {  // 

        case 0:    //
			BlinkM_playScript(scriptAllBlinkMs,10, 0, 0); //   hue cycle         
        break; 
       
        case 1:    //
            BlinkM_playScript(scriptAllBlinkMs,11, 0, 0); //   mood light              
        break; 
       
        case 2:    //
            BlinkM_playScript(scriptAllBlinkMs,12, 0, 0); //   virtual candle
        break; 
       
        case 3:    //
			BlinkM_playScript(scriptAllBlinkMs,13, 0, 0); //   water reflections         
        break; 
       
        case 4:    //
			BlinkM_playScript(scriptAllBlinkMs,14, 0, 0); //   old neon         
        break;        

        case 5:    //
			BlinkM_playScript(scriptAllBlinkMs,15, 0, 0); //   the seasons         
        break; 

        case 6:    //
			BlinkM_playScript(scriptAllBlinkMs,16, 0, 0); //   thunderstorm         
        break; 

        case 7:    // 
			BlinkM_playScript(scriptAllBlinkMs,17, 0, 0); //   stop light               
        break; 

        case 8:    //
          BlinkM_fadeToRandomRGB(scriptAllBlinkMs, 0xff, 0xff, 0xff);      // set a random color        
        break; 

        case 9:    //
                 
        break; 

        case 10:    // multiple fill-ins with with random color
			for (int j=0 ; j < random(5,11); j++) {		// between 5 and 10 times
			randomizeColorList();
			for (int k=0 ; k < 10; k++) { 
				ballGoTo(sequenceColorList[k], random(0,9));   // 9 = allow black
				delay(lightSpeed);         // slight pause for effect
			}
			delay(750);
			}
        break;
        
        case 11: // multiple fill-ins with same color
			for (int j=0 ; j < random(5,11); j++) {		// between 5 and 10 times
				fillIn(lightSpeed,random(0,8));			// random fill in; 8 = avoid black
				delay(750);								// very brief delay
			}
			
        break;

		case 12: // ten individual white flashes
			ballFadeTo(allBlinkMs, lightColor2); // prevents fade to black
			pauseTime(5);
			randomizeColorList();
			for (int j=0 ; j < 10; j++) { 
				ballGoTo(sequenceColorList[j], white);   
				delay(50);					// slight pause for effect
				ballGoTo(sequenceColorList[j], lightColor2);
				delay(lightSpeed);          // slight pause for effect
			} 
		break;

		case 13: // slow white blob moves right
			fillIn(lightSpeed, lightColor2);
			pauseTime(5);
			for (int j=0 ; j < 10; j++) {
					ballFadeTo(j, white);
					pauseTime(1, true);
					ballFadeTo(j, lightColor2);
					delay(350);
			}
		break;
		
		case 14: // decibel meter
			fillIn(lightSpeed, lightColor2);
			pauseTime(1);

			byte pendant = 0;
			byte prevPendant = 0;

			for (int j=0 ; j < 101; j++) {
				prevPendant = pendant;
				pendant = random(1,11);	// pick a random pendant 1-10
				if (pendant >= prevPendant) {
					swipeRightMeter(lightSpeed/2, white, lightColor2, pendant);
				} else {
					swipeLeftMeter(lightSpeed/2, white, lightColor2, (10-pendant));
				}
			
			delay(random(30,101));	// slight pause
			} // repeat
			swipeLeftMeter(lightSpeed/2, white, lightColor2, 9);
		break;

       } // end of switch loop
}
//------------------------------------
void fillIn(byte velocity, byte color1) {
	randomizeColorList();
	for (int i=0 ; i < 10; i++) { 
		ballGoTo(sequenceColorList[i], color1);   
		delay(velocity);         // slight pause for effect
	} 
}
//------------------------------------
void curtainsOut(byte velocity, byte color1, byte color2) {
  byte steps = 5;
  int unitA = 5;
  int unitB = 4;
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
  int unitA = 0;
  int unitB = 9;
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
  int unitA = 0;
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
  int unitA = 9;
  do {
    ballGoTo(unitA, color1);
    delay(velocity);
    ballGoTo(unitA, color2);
    unitA --;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void swipeRightMeter(byte velocity, byte color1, byte color2, byte steps) {
  //byte steps = 10;
  int unitA = 0;
  do {
    ballGoTo(unitA, color1);
    delay(velocity);
    //ballGoTo(unitA, color2);
    unitA ++;
    steps --;
  } while (steps > 0);
}
//------------------------------------
void swipeLeftMeter(byte velocity, byte color1, byte color2, byte steps) {
  //byte steps = 10;
  int unitA = 9;
  do {
    ballGoTo(unitA, color2);
    delay(velocity);
    //ballGoTo(unitA, color2);
    unitA --;
    steps --;
  } while (steps > 0);
}

//------------------------------------
void fadeRight(int velocity, byte color1, byte color2) {
  byte steps = 10;
  int unitA = 0;
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
    ballGoTo(allBlinkMs, color1);	// use fast fade instead? xxx
    delay(velocity);
    ballGoTo(allBlinkMs, color2);
}
//------------------------------------
void christmasLights(int velocity, byte color1, byte color2, byte flips) {
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
  int unitA = 0;
  int unitB = 1;
  setFadeSpeed( fadeFast );
  do {
    ballFadeTo(unitA, color1);
    ballFadeTo(unitB, color2);
    unitA ++;
    unitA ++;
    unitB ++;
    unitB ++;
    steps --;
  } while (steps > 0);
  setFadeSpeed( fadeNormal );
}  
//------------------------------------
void ballFadeTo(byte unit,byte color) {	// fade to this color
  r = color_list[color][0];
  g = color_list[color][1];
  b = color_list[color][2];
  BlinkM_stopScript( unit + 1 );		// turn off any script currently playing
  BlinkM_fadeToRGB( unit + 1, r,g,b );    
}
//------------------------------------
void ballGoTo(byte unit,byte color) {	// go directly to this color
  r = color_list[color][0];
  g = color_list[color][1];
  b = color_list[color][2];
  BlinkM_stopScript( unit + 1 );		// turn off any script currently playing
  BlinkM_setRGB( unit + 1, r,g,b );    
}
//------------------------------------
void fadeRightSlow(byte speed){
  byte lightColor2 = random(0,8);					// between colors 0 and 7 (excludes 8=black)
  r = color_list[lightColor2][0];
  g = color_list[lightColor2][1];
  b = color_list[lightColor2][2];
  BlinkM_stopScript( scriptAllBlinkMs );			// turn off any script currently playing
	for (int i = 0; i < 10; i++) {	
		BlinkM_fadeToRGB(i+1, r, g, b);				// fade the current ball to the random color
        BlinkM_fadeToRGB(i+2, 0x00, 0x00, 0xff);	// set the next ball to blue
		pauseTime(speed, true);						// wait with no random color changes
		if (dialHasTurned() == true) break; 
    }
}
void setFadeSpeed(byte speed) {
	BlinkM_setFadeSpeed(scriptAllBlinkMs, speed);	// Higher numbers means faster fading, 255 == instantaneous fading
}
//------------------------------------
int programTargetIndex(int i, int j) { // find a random point between two items
  int k;
  k = random (i,j+1);
  return k;
}
//------------------------------------
void setupAngles() {
          pointA = programTargetIndex( ceilingIndex, ((floorIndex - ceilingIndex) * 0.3) );	// PointA: find a random point between the ceiling and top 30% from the ceiling
          pointB = programTargetIndex( (floorIndex - (floorIndex * 0.3)), floorIndex );		// PointB: find a random point between 70% and the floor
          pointC = programTargetIndex( ceilingIndex, pointB );								// PointC: find a random point between PointB and the ceiling
          pivotPoint =  programTargetIndex( 3 , 7 );										// PivotPoint: find a random spot between units 3 and 7
              
          gapA = (pointB - pointA) / (pivotPoint -1) ;										// gapA gaps between pointA and pointB is (pB-pA)/(pivotPoint-1)
          gapB = (pointB - pointC) / (10 - pivotPoint) ;									// gapB gaps between pointB and pointC is (pB-pC)/(10 - pivotPoint)
          
		  curtainsOut(100, white, black);
          ballFadeTo(allBlinkMs, blue);

          motor[0][motorTargetIndex] = pointA;
          motor[(pivotPoint-1)][motorTargetIndex] = pointB;									// pivotPoint varies and we need to subtract 1 because arrays are counted from zero
          motor[9][motorTargetIndex] = pointC;
}
//------------------------------------
int calculateSinPosition(byte loopCounter, float sineFrequency) {
  int index = 0;								// prepare the index position to return to PointA
  float transformedSinWave = 0;
  float sinWave = 0;							// prepare a floating point number

  
  sinWave = sin(loopCounter * sineFrequency);	// this is the value, in radians, for this point of the sine wave
  // need to convert it (-1 to +1) into a value that represents a distance between ceiling and floor
  sinWave = sinWave +1;							// add 1: sinWave is now between 0 and 2
  transformedSinWave = ceilingIndex + ((sinWave / 2 ) * (floorIndex - ceilingIndex));// what percent of a max of value "2" is it?
												// this is the gap that we have. Add ceilingIndex to get the value we should go to
  index = transformedSinWave;					// ram floating point number into integer... it will be close enough for union work
  return index;
}
//------------------------------------
void encoderHeightChange(){ // adjust ceiling/floor value by reading encoder
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
void updatePosition(){							// every 5 seconds, adjust the ball           
  RTC.readClock();								// find out what the time is 
  if (abs(RTC.getSeconds()-prevSeconds) > 4) {	// if 5 seconds has passed...
    displayFullTime();							// display the new time
    prevSeconds = RTC.getSeconds();				// save for next time round
    moveTo();
    printStatusMessage(msgUseDial);}     
}
//------------------------------------   
void moveTo() {	//Go to any point - based on currentIndex [motor#][0] go to targetIndex [motor#][1] 
 
for (int i=0 ; i < 10; i++) {	// calculate how many steps (moves) each motor needs to take
  if (motor[i][motorCurrentIndex] > motor[i][motorTargetIndex]) {  // if currentIndex > targetIndex, need to move up CW
    motorDirectionUp(i);    
    motor[i][motorDeltaMoves] = motor[i][motorCurrentIndex] - motor[i][motorTargetIndex];  //move this many steps: current - Target
} else if (motor[i][motorCurrentIndex]  < motor[i][motorTargetIndex]) {  // if currentIndex < targetIndex need to move down CCW
    motorDirectionDown(i);
    motor[i][motorDeltaMoves] = motor[i][motorTargetIndex] - motor[i][motorCurrentIndex]; //move this many steps: target - Current
  }
}
moveBall();  // move all balls
  
for (int i=0 ; i < 10; i++) {  
   motor[i][motorCurrentIndex] = motor[i][motorTargetIndex];
	}       
 doNotRecalibrate = false; // set flag so programs with "do not recalibrate" will work properly
}
//------------------------------------
void motorDirectionUp(byte unit) {		// set motor direction 0 thru 9 to CW (up)
    setRegisterPin((3*unit)+1,HIGH);	// set appropriate 595 register pin
    setRegisterPin((3*unit)+2,LOW); 
}
//------------------------------------
void motorDirectionDown(byte unit) {	// set motor direction 0 thru 9 to CCW (down)
    setRegisterPin((3*unit)+1,LOW);		// set appropriate 595 register pin
    setRegisterPin((3*unit)+2,HIGH); 
}
//------------------------------------
void moveBall() { //spin motor so that each ball that needs to moves the right number of steps
	boolean recalibrationFlag = false;
	
	for (int i=0 ; i < 10; i++) {													// assess motors 0 thru 9 in turn
		recalibrationFlag = false;
		if (motor[i][motorDeltaMoves] != 0) {										// if deltamoves != 0, then we are going to move this motor
			if (motor[i][motorDeltaMoves] != 0) motor[i][motorDeltaMoves] --;		// decrement to account for time spend on ramp up and wind down, which "eats up" one full-power cycle
			printStatusMessage(msgMoving); 
			lcd.print(i);															// print motor number currently moving
																					// ramp up the motor
			pwmMotor(i, 18,  8, 16);												// slow @ 25%
			if (switchValue[i] == false) pwmMotor(i, 13, 13, 10);					// medium @ 50%
			if (switchValue[i] == false) pwmMotor(i,  5, 20, 10);					// fast @ 75%

			// go at full speed				
			while ((motor[i][motorDeltaMoves] > 0) && (switchValue[i] == false)) {	// continue while moves != 0 & switch is false 
				lcd.setCursor(17,3);
				printPositionDigits(motor[i][motorDeltaMoves]);				        // update LCD with current step number
				motor[i][motorDeltaMoves] --;										// decrement delta moves
				pwmMotor(i,  0, 25, 36);											// full speed @ 100%: approx 1 sec according to debug trace
				if (motor[i][motorCurrentIndex] > motor[i][motorTargetIndex]) pwmMotor(i, 0,  25, 3);	// if direction is UP, add a little more nudge
				if (switchValue[i] == true) recalibrationFlag = true;
			} 
			
																					// slow down the motor
			if (switchValue[i] == false) pwmMotor(i,  5, 20, 10);					// fast
			pwmMotor(i, 13, 13, 10, true);											// medium (gets called if recalibration point is reached - mandatory)
			pwmMotor(i, 18,  8, 16, true);											// slow (gets called if recalibration point is reached - mandatory)
			if (motor[i][motorCurrentIndex] > motor[i][motorTargetIndex]) pwmMotor(i, 18,  8, 20);	// if direction is UP, add a little more nudge

			setRegisterPin((3*i), LOW );											// ensure motor is OFF
			writeRegisters();
			
			if (recalibrationFlag == true)  {										// we are at recalibration point and need to unwind				 
				  ballGoTo(i, red);
				  pauseTime(1, true);												// small delay before we reverse - flag true to prevent color change
				  motorDirectionDown(i);											// motor will begin to go down
				  do {
					  pwmMotor(i, 18,  8, 16, true);								// go down slowly
				  } while (switchValue[i] == true);
			
				setRegisterPin((3*i), LOW );										// ensure motor is OFF
				writeRegisters();
				motor[i][motorDeltaMoves] = 0;										// no more moves to make
				motor[i][motorTargetIndex] = 0;										// we are at position 0 (adjusted for CurrentIndex later)
			}
		
		}																			// end of if loop deltamoves != 0
		showTime();																	// keep time up to date
		clearLine(3);																// clear status message
		if (dialHasTurned() == true) break;
	}																				// end of assess motor loop... go to the next motor

  clearRegisters();
  writeRegisters();																	// disable all motors
}
//------------------------------------
void displayFullTime(){
   lcd.setCursor(0, 1);
   lcd.print("Time");
   printLCDDigits(RTC.getHours()); 
   printLCDDigits(RTC.getMinutes());
   printLCDDigits(RTC.getSeconds());
   lcd.print(" ");  // this solves a problem where a digit sometimes gets printed to the right of the time - clear it out instead
   // lcd.print(prevSelectedMenuItem);	// debug code to print current & previous menu item
   // lcd.print("-");
   // lcd.print(selectedMenuItem);
   // lcd.print("  ");
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
    lcd.print(">");                                     // highlight active menu item
    lcd.print(buffer);
	getMessage(msgRecalibrate);							// retrieve the text for "Recalibrate"
	lcd.setCursor(0,2);									// 2nd line, 0 column
	lcd.print(buffer);
	ballFadeTo(allBlinkMs, yellow);
  
	randomizeList();	// call this and refer to sequenceList[i] to get random order of motor moves

	//first, go to what we think of is the ceiling
	for (int i=0 ; i < 10; i++) {						// assign value to all motor targetIndex
		ballFadeTo(sequenceList[i], cyan);
		motor[sequenceList[i]][motorTargetIndex] = ceilingIndex;
		moveTo();										// now go to the ceiling
		if (dialHasTurned() == true) break;				// xxx remove for production code? maybe keep.
         };
 
	// next, for each ball in turn, do a recalibration
	for (int i=0 ; i < 10; i++) {						// assign value to all motor targetIndex
				motor[sequenceList[i]][motorCurrentIndex] = maxRewind;	// limit rewind amount
				motor[sequenceList[i]][motorTargetIndex] = 0;
				ballFadeTo(sequenceList[i], white);
				moveTo();								// go as high as we can till we hit the switch
				ballFadeTo(sequenceList[i], blue);
				motor[sequenceList[i]][motorTargetIndex] = ceilingIndex;
				moveTo();								// go to ceiling
				if (dialHasTurned() == true) break;		// xxx remove for production code? maybe keep.
             };

  //xxx add error code if we leave this and didn't hit the mechanical switch
	clearLine(2);		// clear recalibrate message
  }
//------------------------------------
void recalibratePendant(byte i) {       // Recalibrate pendant i (used when setting ceiling or floor and in falling stars mood)
    clearLine(0);
    getMessage(activeMenuSelection);
    lcd.setCursor(0, 0);
    lcd.print(">");                                     // highlight active menu item
    lcd.print(buffer);
	getMessage(msgRecalibrate);							// retrieve the text for "Recalibrate"
	lcd.setCursor(0,2);									// 2nd line, 0 column
	lcd.print(buffer);
	
	//first, go to what we think of is the ceiling
	motor[i][motorTargetIndex] = ceilingIndex;
	moveTo();										// now go to the ceiling
	// next, do a recalibration
	motor[i][motorCurrentIndex] = maxRewind;	// limit rewind amount
	motor[i][motorTargetIndex] = 0;
	ballFadeTo(i, white);
	moveTo();								// go as high as we can till we hit the switch
	ballFadeTo(i, blue);
	motor[i][motorTargetIndex] = ceilingIndex;
	moveTo();								// go to ceiling
	             
  //xxx add error code if we leave this and didn't hit the mechanical switch
	clearLine(2);		// clear recalibrate message
  }
//------------------------------------
void dropFallingStar(byte i) {			// drop pendant i to 90-100% of floor
	motor[i][motorTargetIndex] =
		programTargetIndex( ((floorIndex - ceilingIndex) * 0.9), floorIndex );	// find a random point between 90% down and the floor (lowest point)
	ballFadeTo(i, white);
	moveTo();
	BlinkM_playScript(i+1, 16, 0, 0);	// play script #16: thunderstorm
}
//------------------------------------
void raiseFallenStar(byte i) {			// raise pendant and do a recalibrate at the same time
	ballFadeTo(i, red);					// go red
	recalibratePendant(i);				// send this pendant to the top
	BlinkM_playScript(i+1, 13, 0, 0);	// play script #13: random blues
}
//------------------------------------
void clearLine(int i){	//clear a specific line
 for (byte j = 0; j <20; j++){
   lcd.setCursor(j,i);
   lcd.print(" ");
 }
}
//------------------------------------
void printStatusMessage(byte i){        //print status message #i at bottom of lcd
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
