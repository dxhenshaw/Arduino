//From bildr article: http://bildr.org/2012/08/rotary-encoder-arduino/
// code to manage rotary encoder and menuing system
//needs to be integrated with main code

#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include <avr/pgmspace.h>
// 20 characters maximum:          12345678901234567890
prog_char menuItem_0[] PROGMEM  = "Run automatically";     //menu items start here
prog_char menuItem_1[] PROGMEM  = "Program 1 (wave)";
prog_char menuItem_2[] PROGMEM  = "Program 2 (flat)";
prog_char menuItem_3[] PROGMEM  = "Program 3 (slope)";
prog_char menuItem_4[] PROGMEM  = "Sleep";
prog_char menuItem_5[] PROGMEM  = "Recalibrate";
prog_char menuItem_6[] PROGMEM  = "Set Hours";
prog_char menuItem_7[] PROGMEM  = "Set Minutes";
prog_char menuItem_8[] PROGMEM  = "Stop";
prog_char menuItem_9[] PROGMEM  = "Set ceiling";
prog_char menuItem_10[] PROGMEM = "Set bottom";
prog_char menuItem_11[] PROGMEM = "Rotate & push to set";  //status messages from this point on
prog_char menuItem_12[] PROGMEM = "Recalibrating...";
prog_char menuItem_13[] PROGMEM = "Move to ceiling...";
prog_char menuItem_14[] PROGMEM = "Move to bottom...";
prog_char menuItem_15[] PROGMEM = "Sleeping";
prog_char menuItem_16[] PROGMEM = "Stopped. Push= start";
// 20 characters maximum:          12345678901234567890

// Then set up a table to refer to your strings:
PROGMEM const char *menuTable[] = 
{   
  menuItem_0,
  menuItem_1,
  menuItem_2,
  menuItem_3,
  menuItem_4,
  menuItem_5,
  menuItem_6,
  menuItem_7,
  menuItem_8,
  menuItem_9,
  menuItem_10,
  menuItem_11,
  menuItem_12,
  menuItem_13,
  menuItem_14,
  menuItem_15,
  menuItem_16  };
  
char buffer[20];    // max size of message, and max size of lcd screen
const int maxMenuItems = 10;    //how many menu items there are
const int encoderGap = 2;       //size of gap from encoder before we pay attention to it
const int msgUseDial = 11;      //message pointers for menu follow...
const int msgRecalibrate = 12;  
const int msgCeiling = 13;
const int msgBottom = 14;
const int msgSleeping = 15;
const int msgStopped = 16;

int encoderPin1 = 2;          //these pins can not be changed 2/3 are special pins
int encoderPin2 = 3;          //used for interrupts
int encoderSwitchPin = 7;     //push button switch on rotary encoder

volatile int lastEncoded = 0;
volatile int encoderValue = 1;
int activeMenuSelection; //which menu item is currently selected?

//int priorActiveMenuSelection; //previous value
int selectedMenuItem; //value of menu selection at the time the button was pushed
int lastEncoderValue = 1;
int ceilingIndex = 10; //index for ceiling level
int bottomIndex = 20; //index for bottom level
int hours = 6; //test for clock
int minutes = 4; //test for clock

int lastMSB = 0;
int lastLSB = 0;
//int n0; //counter variable

  LiquidCrystal_I2C lcd(0x27,20,4);      // set the LCD address 

void setup() {
  Serial.begin (9600);
  lcd.init();                            // initialize the lcd 
  lcd.backlight();
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  pinMode(encoderSwitchPin, INPUT);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderSwitchPin, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);   //call updateEncoder() when any high/low changed seen
  attachInterrupt(1, updateEncoder, CHANGE);  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 

}

void loop(){ 
    
//menu selection  
   if (abs(lastEncoderValue - encoderValue) > encoderGap) //menu selection has changed
 {
  //figure out what the new menu selection is
  if (encoderValue > lastEncoderValue) { //xxx may need to change only when has a difference of > X
    activeMenuSelection ++;// add 1 to menu
    if (activeMenuSelection > maxMenuItems) activeMenuSelection = 0;
  }else{ 
    activeMenuSelection --;// subtract 1 from menu
    if (activeMenuSelection < 0) activeMenuSelection = maxMenuItems;
  }
      clearLine(0);
      getMessage(activeMenuSelection);
      lcd.setCursor(0, 0);
      lcd.print(">"); //highlight active menu item
      lcd.print(buffer);
     
lastEncoderValue = encoderValue;
 }
 
// check if button has been pushed 
  if(digitalRead(encoderSwitchPin)){
    //button is not being pushed, which means we can execute code based on it being pushed last time round this loop
    switch (selectedMenuItem) {

      case 4:   //sleep mode
        recalibrate();
        printStatusMessage(msgCeiling);
        moveTo(ceilingIndex);
        printStatusMessage(msgSleeping);
        
        break;
      
      case 5:  //recalibrate  
        recalibrate();  //call recalibrate routine
        break;      
      
      case 6: //set hours
        printStatusMessage(msgUseDial);
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
            //figure out what the new menu selection is
              if (encoderValue > lastEncoderValue) { 
                hours ++;// add 1 to menu
                if (hours > 23) hours = 0;
            }else{ 
                hours --;// subtract 1 from menu
                if (hours < 0) hours = 23;
            }
            }
          lcd.setCursor(0, 2);
          lcd.print("Hours:");
          lcd.print(hours);
          //xxx add code to update hours on rtc
          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
       break; 
       
      case 7: //set minutes
        printStatusMessage(msgUseDial);
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
            //figure out what the new menu selection is
              if (encoderValue > lastEncoderValue) {
                minutes ++;// add 1 to menu
                if (minutes > 59) minutes = 0;
            }else{ 
                minutes --;// subtract 1 from menu
                if (minutes < 0) minutes = 59;
            }
            }
          //print hours on lcd
          lcd.setCursor(0, 2);
          lcd.print("Minutes:");
          lcd.print(minutes);
          //xxx add code to update minuteson rtc
          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        clearLine(2);
        clearLine(3);
       break; 
       
       
      case 8:  //stop/resume
        //do not recalibrate - just stop whatever is happening right now
        printStatusMessage(msgStopped);
        
        do {
          delay(250);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed 
        break;
  
      case 9:  //Set Ceiling
        recalibrate();  //first, recalibrate
        printStatusMessage(msgCeiling); //moving to ceiling...
        moveTo(ceilingIndex);  //then go to the ceiling
        printStatusMessage(msgUseDial);
        
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              //figure out what the new menu selection is
              if (encoderValue > lastEncoderValue) {
                ceilingIndex ++;// add 1 to menu
                if (ceilingIndex > 59) ceilingIndex = 59;
            }else{ 
                ceilingIndex --;// subtract 1 from menu
                if (ceilingIndex < 0) ceilingIndex = 0;
            }
            }
          //print hours on lcd
          lcd.setCursor(0, 2);
          lcd.print("Ceiling:");
          lcd.print(ceilingIndex);
          //xxx add code to moveTo(ceilingIndex);
          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        clearLine(2);
        clearLine(3);
        break;
        
      case 10:  //Set Bottom
        recalibrate();  //first, recalibrate
        printStatusMessage(msgBottom); //moving to bottom...
        moveTo(bottomIndex);  //then go to the bottom
        printStatusMessage(msgUseDial);
        
        printStatusMessage(msgUseDial);
        do{
            if (abs(lastEncoderValue - encoderValue) > encoderGap){
              //figure out what the new menu selection is
              if (encoderValue > lastEncoderValue) {
                bottomIndex ++;// add 1 to menu
                if (bottomIndex > 59) bottomIndex = 59;
            }else{ 
                bottomIndex --;// subtract 1 from menu
                if (bottomIndex < 0) bottomIndex = 0;
            }
            }
          //print hours on lcd
          lcd.setCursor(0, 2);
          lcd.print("Bottom:");
          lcd.print(bottomIndex);
          //xxx add code to moveTo(bottomIndex);
          
          lastEncoderValue = encoderValue;
          delay(100);
        } while (digitalRead(encoderSwitchPin)); //exit when the button is pressed to set hours
        clearLine(2);
        clearLine(3);
        break;
    }
  
    delay(400); //gives time for button to be released  
    selectedMenuItem =99;


  }else{
    //button is being pushed
    lcd.setCursor(17, 0);
    lcd.print("<<<"); //indicate to user it has been pushed
    selectedMenuItem = activeMenuSelection;
  }
 
 
 
  delay(50); //just here to slow down the output, and show it will work  even during a delay
}
//end of loop


//Recalibrate
void recalibrate() {
  printStatusMessage(msgRecalibrate);
  delay(5000); //for simulation
  }
   
//Go to any point
void moveTo(int i) {
  delay(5000); //for simulation
  //figure out current vs target (=ceiling) then call a generic moveTo routine that takes you to the point
  }

//clear a specific line
void clearLine(int i){
 for (int j = 0; j <20; j++){
   lcd.setCursor(j,i);
   lcd.print(" ");
 }
}

//print message at bottom of lcd
void printStatusMessage(int i){
      //print status message i
      getMessage(i);
      clearLine(3);
      lcd.setCursor(0, 3);
      lcd.print(buffer);
      }
      
//function to retrieve eeprom message
void getMessage(int index){ 
  strcpy_P(buffer, (char*)pgm_read_word(&(menuTable[index]))); // Necessary casts and dereferencing, just copy.
}

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
