/*
 * Kinetic Balls -- David Henshaw, December 2012-
 * 
 * v2 - "Points" basic program control - 12/26/12
 * v1 - LEDs, menu system, rotary encoder - 12/23/12
 * v0 - basic motor, lcd, rtc and led control 12/15/12 
 *
 * Note: This sketch sends to the I2C "broadcast" address of 0, 
 *       so all BlinkMs on the I2C bus will respond
 * Includes code from: http://bildr.org/2012/08/rotary-encoder-arduino/
 * to manage rotary encoder and menuing system
 */

#include "Wire.h"                //I2C protocol
#include "BlinkM_funcs.h"        //BlinkM library
#include "RealTimeClockDS1307.h" //https://github.com/davidhbrown/RealTimeClockDS1307
#include "LiquidCrystal_I2C.h"   //http://arduino-info.wikispaces.com/LCD-Blue-I2C
#include "avr/pgmspace.h"        //for eeprom storage

//EEPROM Storage section
// 20 characters maximum:          12345678901234567890
prog_char menuItem_0[] PROGMEM  = "Run automatically";     //menu items start here
prog_char menuItem_1[] PROGMEM  = "P1 (ladder)";
prog_char menuItem_2[] PROGMEM  = "P2 (points)";
prog_char menuItem_3[] PROGMEM  = "Program 3 (tbd)";
prog_char menuItem_4[] PROGMEM  = "Sleep";
prog_char menuItem_5[] PROGMEM  = "Recalibrate";
prog_char menuItem_6[] PROGMEM  = "Set sleep/wake time";
prog_char menuItem_7[] PROGMEM  = "Set Time";
prog_char menuItem_8[] PROGMEM  = "Stop";
prog_char menuItem_9[] PROGMEM  = "Set ceiling";
prog_char menuItem_10[] PROGMEM = "Set bottom";
prog_char menuItem_11[] PROGMEM = "Rotate & push to set";  //status messages from this point on
prog_char menuItem_12[] PROGMEM = "Recalibrating...";//not being shown xxx
prog_char menuItem_13[] PROGMEM = "Move to ceiling...";
prog_char menuItem_14[] PROGMEM = "Move to bottom...";
prog_char menuItem_15[] PROGMEM = "Sleeping";
prog_char menuItem_16[] PROGMEM = "STOP. Push to start";
prog_char menuItem_17[] PROGMEM = "Moving up...";
prog_char menuItem_18[] PROGMEM = "Moving down...";
prog_char menuItem_19[] PROGMEM = "Menu: Turn dial+wait"; //xxx not currently being used
prog_char menuItem_20[] PROGMEM = "Paused";
// 20 characters maximum:          12345678901234567890

// Then set up a table to refer to the strings:
PROGMEM const char *menuTable[] = {   
  menuItem_0,  menuItem_1,  menuItem_2,  menuItem_3,  menuItem_4,  menuItem_5,  menuItem_6,  menuItem_7,
  menuItem_8,  menuItem_9,  menuItem_10,  menuItem_11,  menuItem_12,  menuItem_13,  menuItem_14,  menuItem_15,
  menuItem_16, menuItem_17, menuItem_18, menuItem_19, menuItem_20  };

// This sketch sends to the I2C "broadcast" address of 0, 
// so all BlinkMs on the I2C bus will respond
#define blinkm_addr 0x00

//Constants:
  const int stepper1Step = 4;           //Digital Pins 4-6 - stepper motor control
  const int stepper1Direction = 5;
  const int stepper1Enable = 6;
  int stepperDelay = 10; // xx use lookup array or min/max
  const int minStepperDelay = 60;        // minimum delay between motor steps orig 15
  const int maxStepperDelay = 150;        // max delay orig 75
  int programPauseTime;
  const int minPauseTime = 5;
  const int maxPauseTime= 25;
  const int recalibrationThreshold = 1000;    // used for IR detection to determine amount that signals blockage
  const int maxIndex = 250;
  const byte maxMenuItems = 10;          //how many menu items there are
  const int encoderGap = 2;             //size of gap from encoder before we pay attention to it
  const byte msgUseDial = 11;            //message pointers for menu follow...
  const byte msgRecalibrate = 12;  
  const byte msgCeiling = 13;
  const byte msgBottom = 14;
  const byte msgSleeping = 15;
  const byte msgStopped = 16;
  const byte msgUp = 17;
  const byte msgDown = 18;
  const byte msgSeeMenu = 19;
  const byte msgPause = 20;
  
//Variables:
  int targetIndex;                      //Index position we need to be at
  int currentIndex;                     //where we think we're at right now
  char buffer[20];                      // max size of message, and max size of lcd screen
  int encoderPin1 = 2;                  //these pins can not be changed 2/3 are special pins
  int encoderPin2 = 3;                  //used for interrupts on rotary encoder
  int encoderSwitchPin = 7;             //push button switch on rotary encoder
  volatile int lastEncoded = 0;         //for interrupt routine
  volatile int encoderValue = 1;        
  int activeMenuSelection;              //which menu item is currently selected?
  int selectedMenuItem;                 //value of menu selection at the time the button was pushed
  int lastEncoderValue = 1;
  int ceilingIndex = 3;                //index for ceiling level
  int bottomIndex = 25;                 //index for bottom level
  int prevSeconds = 99;                // prior value of seconds
  int lastMSB = 0;                     //for interrupts
  int lastLSB = 0;                     //for interrupts
  int i = 0;  //counter  
  int n0 = 0; //counter
  const int stepLoop = 50; // how many pulses of the motor for one index change
  byte sleepHour = 23;                // time to go to sleep
  byte wakeHour = 7;                  // time to wake up
 //float sinWaveIncrement;
 //float sinWaveValue;
 //float delayValue;
 //const float  baseDelay = 25;

byte color_list[][3] = {
  { 0xff, 0xff, 0xff }, // white
  { 0xff, 0x00, 0xff }, // purple
  { 0xff, 0xff, 0x00 }, // orange
  { 0x00, 0xff, 0xff }, // cyan
  { 0xff, 0x00, 0x00 }, // red
  { 0x00, 0x00, 0x00 }, // black
  { 0x00, 0x00, 0xff},  // blue
  { 0xFF, 0xFF, 0x00}   // yellow
};

byte r;  // red
byte g;  // green
byte b;  // blue

const byte white = 0;  // constants to make it easy to refer to colors in the code
const byte red = 4;
const byte orange = 2;
const byte cyan = 3;
const byte blue = 6;
const byte yellow = 7;
const byte black = 5;

LiquidCrystal_I2C lcd(0x27,20,4);      // set the LCD address to 0x27 for a 16 chars and 4 line display

void setup()
{
  Serial.begin(9600);                 // start serial comms for terminal
  lcd.init();                         // initialize the lcd 
  lcd.backlight();                    // turn on the backlight
  BlinkM_beginWithPower();            // turn on LED
  BlinkM_stopScript( blinkm_addr );   // turn off startup script
  
//Configure pins:
  pinMode(stepper1Step, OUTPUT);       // step
  pinMode(stepper1Direction, OUTPUT);  // direction
  pinMode(stepper1Enable, OUTPUT);     // enable
  pinMode(encoderPin1, INPUT);         // rotary encoder
  pinMode(encoderPin2, INPUT);         // interrupt pins
  pinMode(encoderSwitchPin, INPUT);    // push button
  digitalWrite(encoderPin1, HIGH);     // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH);     // turn pullup resistor on
  digitalWrite(encoderSwitchPin, HIGH);// turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);   //call updateEncoder() when any high/low changed seen
  attachInterrupt(1, updateEncoder, CHANGE);   //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 

  //Ensure motor begins in disabled state
  digitalWrite(stepper1Enable, HIGH);    //disable motor   low = enabled, high = disabled
  
  randomSeed(analogRead(A1));          // random seed based on empty analog pin 1
  BlinkM_setFadeSpeed(blinkm_addr, 2);     // Higher numbers means faster fading, 255 == instantaneous fading
  
  //At startup: set time - TEST CODE ONLY
  /*RTC.setHours(16);
  RTC.setMinutes(45);
  RTC.setClock(); */
} 

void loop()
{
/*   BlinkM_fadeToRGB( blinkm_addr, 0x66, 0x33, 0xcc ); //
// Fades to an RGB color
 BlinkM_fadeToRGB(byte addr, byte red, byte grn, byte blu)

// Sets an RGB color immediately
 BlinkM_setRGB(byte addr, byte red, byte grn, byte blu)

// Fades to an HSB color
BlinkM_fadeToHSB(byte addr, byte hue, byte saturation, byte brightness)

// Fades to a random RGB color
static void BlinkM_fadeToRandomRGB(byte addr, byte rrnd, byte grnd, byte brnd)
It takes 3 bytes as arguments,one for each R,G,B channel. Each argument is the range or amount of randomness for each
of the R,G,B channels from which to deviate from the current color.
A setting of 0 for a channel means to not change it at all.
This command is good for creating randomly fading colors like a mood light

// Sets the speed of fading between colors.  
// Higher numbers means faster fading, 255 == instantaneous fading
BlinkM_setFadeSpeed(byte addr, byte fadespeed)

 
  
  BlinkM_playScript(byte addr, byte script_id, byte reps, byte pos)
  The first byte is the script id of the script
to play. A list of the available scripts is below. The second argument is the number of repeats
to play the script. A repeats value of 0 means play the script forever. The last argument is the
script line number to start playing from. A value of 0 means play the script from the start.
To adjust the playback speed of a script that’s running, adjust the fade speed (“Set Fade
Speed”, ‘f’) and time adjust (“Set Time Adjust”, ‘t’) to taste.

BlinkM_stopScript(byte addr)

// Sets the light script playback time adjust
// The timeadj argument is signed, and is an additive value to all
// durations in a light script. Set to zero to turn off time adjust.
BlinkM_setTimeAdj(byte addr, byte timeadj)

 */
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
    //button is not being pushed, which means we can execute code based on it being pushed last time round this loop
    switch (selectedMenuItem) {

      case 1:    // ladder program (up and down while running scripts)
        // recalibrate(); xxx debug code - put this back in!
        do {    // main ladder loop
          do { // head downward
            targetIndex = programTargetIndex(currentIndex, bottomIndex); // find a random place to go between current and bottom
            programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
            pauseTime(programPauseTime);                                // convert to milliseconds xxx add message on pause time
            stepperDelay = random(minStepperDelay, maxStepperDelay);    // set random speed
            BlinkM_playScript(blinkm_addr, 12, 0, 0);  // play script #12: candle
            moveTo(targetIndex);
            ballFadeTo(blue);
            if (abs(lastEncoderValue - encoderValue) > encoderGap) break;
          } while (currentIndex != bottomIndex); // exit when dial has turned. Note < operation.
         if (abs(lastEncoderValue - encoderValue) > encoderGap) break;
         do { // head up
            targetIndex = programTargetIndex(ceilingIndex, currentIndex); // find a random place to go between current and top
            programPauseTime = random(minPauseTime, maxPauseTime);       // in seconds - wait before moving
            pauseTime(programPauseTime);                                // convert to milliseconds xxx add message on pause time
            stepperDelay = random(minStepperDelay, maxStepperDelay);    // set random speed
            BlinkM_playScript(blinkm_addr, 14, 0, 0);  // play script #14: reds
            moveTo(targetIndex);
            ballFadeTo(blue);
            if (abs(lastEncoderValue - encoderValue) > encoderGap) break;
          } while (currentIndex != ceilingIndex); // exit when dial has turned. Note < operation.
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.      
        break;
        
      case 2:   // random points program
        recalibrate();      
        do {
          targetIndex = programTargetIndex(ceilingIndex, bottomIndex);             // find a random place to go
          programPauseTime = random(minPauseTime, maxPauseTime);           // in seconds - wait before moving
          pauseTime(programPauseTime);                                // convert to milliseconds xxx add message on pause time
          stepperDelay = random(minStepperDelay, maxStepperDelay);    // set random speed
          BlinkM_fadeToRandomRGB(blinkm_addr, 0xff, 0xff, 0xff);      // set a random color
          moveTo(targetIndex);
          ballFadeTo(blue);
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.
        break;
        
      case 4:   //sleep mode
        recalibrate();
        printStatusMessage(msgSleeping);
        do {
         ballFadeTo(white);
         delay(500);
         ballFadeTo(black);
         delay(4000);
         showTime();
        } while (abs(lastEncoderValue - encoderValue) < encoderGap); // exit when dial has turned. Note < operation.        
        break;
      
      case 5:  //recalibrate  
        recalibrate();
        break;     
       
      case 6:  // set sleep and wake time 
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
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print("Wake");
        do{ // let's set wake hour...
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              if (encoderValue > lastEncoderValue) {
                wakeHour = addHour(wakeHour);
            }else{ 
                wakeHour = subtractHour(wakeHour);
            }
            }          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set time   
        clearLine(2);
        break;
       
      case 7: //set time
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
       break; 
              
      case 8:  //stop/resume
        //do not recalibrate - just stop whatever is happening right now
        printStatusMessage(msgStopped);
        
        do {
          delay(250);  // just wait until a resume is requested
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed 
        break;
  
      case 9:  //Set Ceiling
        recalibrate();  //first, recalibrate
        printStatusMessage(msgUseDial);
        targetIndex = ceilingIndex; 
        
        do{
          encoderHeightChange();
          lcd.setCursor(0, 2);    //print hours on lcd
          lcd.print("Ceiling:");
          printPositionDigits(targetIndex);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();
        
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        ceilingIndex = targetIndex;              // new value for ceiling
        clearLine(2);
        clearLine(3);
        break;
        
      case 10:  //Set Bottom
        recalibrate();  //first, recalibrate
        printStatusMessage(msgUseDial);
        targetIndex = bottomIndex; 
        
       do{
          encoderHeightChange();
          lcd.setCursor(0, 2);    //print hours on lcd
          lcd.print("Bottom:");
          printPositionDigits(targetIndex);
          lastEncoderValue = encoderValue;
          delay(200);
          updatePosition();           
          
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        bottomIndex = targetIndex;              // new value for bottom
        clearLine(2);
        clearLine(3);
        break;
        
    }
    delay(200); //gives time for button to be released  
    selectedMenuItem =99;


  }else{
    //button is being pushed
    lcd.setCursor(17, 0);
    lcd.print("<<<"); //indicate to user it has been pushed
    selectedMenuItem = activeMenuSelection;
    BlinkM_stopScript( blinkm_addr );   // turn off any script currently playing
  }
  //delay(50); //just here to slow down the output, and show it will work  even during a delay
}
//end of loop



//------------------------------------
void ballFadeTo(byte color) {
  r = color_list[color][0];
  g = color_list[color][1];
  b = color_list[color][2];
  BlinkM_stopScript( blinkm_addr );   // turn off any script currently playing
  BlinkM_fadeToRGB( blinkm_addr, r,g,b );
    
}

//------------------------------------
int programTargetIndex(int i, int j) { // find a random point between two items, > 5 apart
  int k;
  //do {
  k = random (i,j+1);  // xxx loops here
  //} while (abs(k-currentIndex) < 6); // while (abs(i-j)/k >7)
  return k;
}

//------------------------------------
void pauseTime(int k) { // pause for k seconds, including a countdown

  printStatusMessage(msgPause);
  for (int i=k ; i > 0; i--) {
    showTime();
    lcd.setCursor(17,3);
    printPositionDigits(i);
    delay(995);// pause for almost one second
  }
}

//------------------------------------
void encoderHeightChange(){ //adjust ceiling/bottom value by reading encoder
  if (abs(lastEncoderValue - encoderValue) > encoderGap){
    if (encoderValue > lastEncoderValue) {
      targetIndex ++;
      if (targetIndex > maxIndex) targetIndex = maxIndex;             
   }else{ 
      targetIndex --;
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
    moveTo(targetIndex);
    printStatusMessage(msgUseDial);}     
}

//------------------------------------
void moveBall(int i){
//spin motor so that ball moves i number of steps 
    digitalWrite(stepper1Enable, LOW);     //enable    low = enabled, high = disabled
/* sin wave starts at 7.8 and ends at 14.6... difference is 6.8
 # steps = current - target
 increment per step is 10.94-4.7 / steps */
 //Serial.print("@moveBall, i=");
 //Serial.println(i);
 
 //sinWaveIncrement = 6.8/i;
 //sinWaveValue = 7.8; //start point
 
 for (int j=i; j > 0; j--) {
  stepMotor();
  lcd.setCursor(17,3);
  printPositionDigits(j);   // update LCD with current step number
  //delayValue = (sin(sinWaveValue)+1)* baseDelay;   //delay based on sin wave
  //Serial.print(i);
  //Serial.print("=");
  //Serial.println(delayValue);
  //sinWaveValue = sinWaveValue + sinWaveIncrement;

  //delay(delayValue);

 if (analogRead(A0) > recalibrationThreshold)  // we are at recalibration point
   break; // out of this for loop 

  } //end of motor pulse loop
digitalWrite(stepper1Enable, HIGH);    //disable motor   low = enabled, high = disabled
}

//------------------------------------   
//Go to any point - based on currentIndex, go to index k
void moveTo(int k) {
  
  lcd.setCursor(0, 2);  // display "From XXX to YYY @ ZZZ"
  lcd.print("From ");
  printPositionDigits(currentIndex);
  lcd.print(">");
  printPositionDigits(k);
  lcd.print(" @ ");  
  printPositionDigits((maxStepperDelay - stepperDelay));
   
  if (currentIndex > k) {  // need to move up
    printStatusMessage(msgUp);
    digitalWrite(stepper1Direction, LOW); //direction low = ccw, High = cw
    moveBall(currentIndex - k);  //move this many steps
} else if (currentIndex < k) {  // need to move down
    printStatusMessage(msgDown);
    digitalWrite(stepper1Direction, HIGH); //direction low = ccw, High = cw
    moveBall(k - currentIndex);  //move this many steps  
  }
 currentIndex = k;
 
   // unwind slightly if we are at the very top
  if (analogRead(A0) > recalibrationThreshold){  // we are at recalibration point
   currentIndex = 0; 
   // force an unwind
   digitalWrite(stepper1Direction, LOW); //direction low = ccw, High = cw
   digitalWrite(stepper1Enable, LOW);     //enable    low = enabled, high = disabled
   do {
       for (int n0=0 ; n0 < stepLoop; n0++) {
     
       digitalWrite(stepper1Step, HIGH);   // make one step
       delay(stepperDelay);              // pause for effect USED TO BE stepperDelay
       digitalWrite(stepper1Step, LOW);    // reset step
       delay(stepperDelay);
 
  }
   } while (analogRead(A0) > recalibrationThreshold); 
   digitalWrite(stepper1Enable, HIGH);    //disable motor   low = enabled, high = disabled
  }   
 targetIndex = currentIndex;
 showTime();    // keep time up to date
 clearLine(2);  // clear moving to.. message        
 clearLine(3);  // clear status message
}
  
//------------------------------------
void stepMotor(){
  for (int n0=0 ; n0 < stepLoop; n0++) {
  //step once
       digitalWrite(stepper1Step, HIGH);   // make one step
       delay(stepperDelay);              // pause for effect USED TO BE stepperDelay
       digitalWrite(stepper1Step, LOW);    // reset step
       delay(stepperDelay);
       //if ball detector = ture, break out of loop and set currentindex at 0
       if (analogRead(A0) > recalibrationThreshold)  // we are at recalibration point
         break; // out of this for loop 
  }
}

//------------------------------------
void displayFullTime(){
   lcd.setCursor(0, 1);
   lcd.print("Time");
   printLCDDigits(RTC.getHours()); 
   printLCDDigits(RTC.getMinutes());
   printLCDDigits(RTC.getSeconds());
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
  printStatusMessage(msgRecalibrate);
  ballFadeTo(yellow);
  stepperDelay = 4;        // don't go slow
  currentIndex = 255;      // override current index
  moveTo(0);               // go as high as we can till we hit the IR detectors
  //xxx add error code if we leave this and didn't see the ball
  ballFadeTo(blue);
  printStatusMessage(msgCeiling);
  moveTo(ceilingIndex);
  //ballFadeTo(cyan);
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
//retrieve eeprom message and put it into buffer
void getMessage(byte index){ 
  strcpy_P(buffer, (char*)pgm_read_word(&(menuTable[index]))); // Necessary casts and dereferencing, just copy.
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
