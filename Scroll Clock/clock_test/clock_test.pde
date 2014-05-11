/* SCROLL CLOCK
 * David Henshaw - November 2011-
 * v1. Basic timekeeping, LCD control, button logic - 11/6/11
 * v2. Infra-red detectors, StepGenie logic - 11/21/11
 */

#include <Wire.h>
#include <RealTimeClockDS1307.h> //https://github.com/davidhbrown/RealTimeClockDS1307
#include <LiquidCrystal_I2C.h>   //http://arduino-info.wikispaces.com/LCD-Blue-I2C

  //Variables:
  int targetIndex;  //Index position we need to be at
  int currentIndex; //where we think we're at right now
  
  //Constants:
  const int addMinutePin = 2;            //Digital Pin 2 used for time switches
  const int subtractMinutePin = 3;       //Digital Pin 3
  const int indexMinuteMarker = 2;       //Analog Pin 2 - infrared minute marker
  const int indexCalibrationMarker = 3;  //Analog Pin 2 - infrared recalibration marker
  const int stepper1Step = 4;            //Digital Pins 4-9 - stepper motor control
  const int stepper1Direction = 5;
  const int stepper1Enable = 6;
  const int stepper2Step = 7;
  const int stepper2Direction = 8;
  const int stepper2Enable = 9;
  const int stepperDelay = 50;           //delay between step commands
  
  LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()  {
  Serial.begin(9600);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
  //Configure pins:
  pinMode(addMinutePin, INPUT);
  digitalWrite(addMinutePin, HIGH); //turn on internal pull-up resistor
  pinMode(subtractMinutePin, INPUT);
  digitalWrite(subtractMinutePin, HIGH);//turn on internal pull-up resistor
  pinMode(stepper1Step, OUTPUT);       // step
  pinMode(stepper1Direction, OUTPUT);  // direction
  pinMode(stepper1Enable, OUTPUT);     // enable
  pinMode(stepper2Step, OUTPUT);       // step
  pinMode(stepper2Direction, OUTPUT);  // direction
  pinMode(stepper2Enable, OUTPUT);     // enable
  
  //Ensure both motors begin in disabled state
  digitalWrite(stepper1Enable, HIGH);    //disable motor   low = enabled, high = disabled
  digitalWrite(stepper2Enable, HIGH);    //disable motor   low = enabled, high = disabled
   
  //just for testing - make current and target index the same to minimize startup time:
  RTC.readClock();
  calculateTargetIndex(); //Figure out target index
  currentIndex =targetIndex ;
}

void loop()
{
   RTC.readClock();
   calculateTargetIndex(); //Figure out target index
   digitalClockDisplay();  //Print the time, etc
   
   if (digitalRead(addMinutePin) == LOW)
   {
     addMinute();
   }
   else if (digitalRead(subtractMinutePin) == LOW) 
   {
     subtractMinute();
   }
   else if (targetIndex > currentIndex)
   {
     moveForward();
   }
   else if (targetIndex < currentIndex)
   {
     moveBackward();
   }
   

// for elsewhere... 
   if (analogRead(indexCalibrationMarker) == 0) 
   {
     Serial.println("@ zero!");
   }

 delay(1000);
}

//------------------------------------
void moveBackward(){
 //go backward index # of times until current = target 
 // pay special attention to zero marker flag
// Serial.println("Pulse!");

 digitalWrite(stepper2Direction, HIGH); //direction low = ccw, High = cw
 digitalWrite(stepper2Enable, LOW);    //enable   low = enabled, high = disabled

 for (int i=currentIndex; i > targetIndex; i--)
 {
  Serial.println("moveBackward");
   for (int y=0; y < 40; y++)
   {
    if (analogRead(indexMinuteMarker) == 0)  //we have seen the minute marker
   {
     Serial.println("index!");
     break;
   }
     digitalWrite(stepper2Step, HIGH);   // make one step
     delay(stepperDelay);                // pause for effect
     digitalWrite(stepper2Step, LOW);    // reset step
     delay(stepperDelay);
   }
      currentIndex-- ;
      digitalClockDisplay();  //Print the time, etc
     
   //if this loop finishes and y=39, then there was a problem because we didn't see the index marker ***
   
 }
 digitalWrite(stepper2Enable, HIGH);    //disable motor   low = enabled, high = disabled
}

//------------------------------------
void moveForward(){
 //go forward index # of times until current = target 
// Serial.println("Pulse!");

 digitalWrite(stepper1Direction, HIGH); //direction low = ccw, High = cw
 digitalWrite(stepper1Enable, LOW);    //enable   low = enabled, high = disabled

 for (int i=currentIndex; i < targetIndex; i++)
 {
  Serial.println("moveForward");
 //next: code to advance reel by one index marker + update lcd
   for (int y=0; y < 40; y++)
   {
    if (analogRead(indexMinuteMarker) == 0)  //we have seen the minute marker
   {
     Serial.println("index!");
     break;
   }
     digitalWrite(stepper1Step, HIGH);   // make one step
     delay(stepperDelay);                // pause for effect
     digitalWrite(stepper1Step, LOW);    // reset step
     delay(stepperDelay);
   }
      currentIndex++ ;
      digitalClockDisplay();  //Print the time, etc
     
   //if this loop finishes and y=39, then there was a problem because we didn't see the index marker ***
   
 }
 digitalWrite(stepper1Enable, HIGH);    //disable motor   low = enabled, high = disabled
}

//------------------------------------
void calculateTargetIndex(){
   //Given the current time, what index point on the reel should we go to?
   // = Minutes + 1
   // and if Hour<>12, then add 60 * number of hours
  targetIndex = RTC.getMinutes() + 1;
  if( RTC.getHours() < 12) targetIndex = targetIndex + RTC.getHours() * 60;
  if( RTC.getHours() > 12) targetIndex = targetIndex + (RTC.getHours()-12) * 60;     
}

//------------------------------------
void digitalClockDisplay(){
  // digital clock display of the time
   lcd.setCursor(0, 0);
   lcd.print("Time");
   printLCDDigits(RTC.getHours()); 
   printLCDDigits(RTC.getMinutes());
   printLCDDigits(RTC.getSeconds());
   lcd.setCursor(0, 1);
   lcd.print("Posn:");
   lcd.print(targetIndex);
   lcd.print("/");
   lcd.print(currentIndex);
}

//------------------------------------
void printLCDDigits(int digits){
  // utility function for lcd display: prints preceding
  // colon and leading 0
  lcd.print(":");
  if(digits < 10) lcd.print('0');
  lcd.print(digits);
}

//------------------------------------
void addMinute(){
    if (RTC.getMinutes() == 59)
    {
       RTC.setMinutes(0);
       if (RTC.getHours() == 23)
       {
         RTC.setHours(0);
       }
       else
       {
         RTC.setHours(RTC.getHours()+1);
       }
    }
    else
    {  
      RTC.setMinutes(RTC.getMinutes()+1);
    }
      RTC.setClock();
}

//------------------------------------
void subtractMinute(){
    if (RTC.getMinutes() == 0)
    {
       RTC.setMinutes(59);
       if (RTC.getHours() == 0)
       {
         RTC.setHours(23);
       }
       else
       {
         RTC.setHours(RTC.getHours()-1);
       }
    }
    else
    {  
      RTC.setMinutes(RTC.getMinutes()-1);
    }
      RTC.setClock();
}    
