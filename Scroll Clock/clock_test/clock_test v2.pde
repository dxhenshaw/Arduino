/*
 * TimeRTC sketch
 * example code illustrating Time library with real-time clock.
 * David Henshaw November 2011-
 * v1. Basic timekeeping
 * use     adjustTime(long adjustment) to make changes to time in seconds
 */

#include <Time.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t

  //Variables:
  int targetIndex;  //Index position we need to be at
  
  //Constants:
  const int addMinutePin = 2;  //used for time switch
  const int addHourPin = 3;
  const long oneMoreMinute = 60;
  const long oneMoreHour = 3600;
  
void setup()  {
  Serial.begin(9600);
  
  //Configure pins:
  pinMode(addMinutePin, INPUT);
  digitalWrite(addMinutePin, HIGH); //turn on internal pull-up resistor
  pinMode(addHourPin, INPUT);
  digitalWrite(addHourPin, HIGH); 
  
  //Set up communication with clock:
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet)
     Serial.println("Err.1"); //Unable to sync with the RTC
  else
     Serial.println("RTC OK"); //RTC has set the system time
     
   
}

void loop()
{
   digitalClockDisplay();  //Print the time (eventually to LCD)
   calculateTargetIndex(); //Figure out target index and print it
   delay(1000);
   
   int val = digitalRead(addMinutePin); 
   if (val == LOW)
   {
     Serial.println("+Min!");
    time_t t = ????60;
    RTC.set(t);
    adjustTime(t);
   }
   
   val = digitalRead(addHourPin); 
   if (val == LOW)
   {
     Serial.println("-Min!");
    
     adjustTime(-60);
   }
}

void calculateTargetIndex(){
   //Given the current time, what index point on the reel should we go to?
   // = Minutes + 1
   // and if Hour<>12, then add 60 * number of hours
   targetIndex = minute() + 1;
   if( hourFormat12() < 12)
     targetIndex = targetIndex + hourFormat12() * 60;
  Serial.println(targetIndex);
}


void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print("Time: ");
  Serial.print(hour()); //24 hour format
  printDigits(minute());
  printDigits(second());
  Serial.print(" = ");
  //Serial.print(time_t()); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding
  // colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
      
    
