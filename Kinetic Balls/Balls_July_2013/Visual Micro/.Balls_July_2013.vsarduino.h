/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Uno, Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define _VMDEBUG 1
#define ARDUINO 103
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

//
//
void printLCDActiveMood(byte menu);
void randomizeColorList();
boolean dialHasTurned();
byte determineTimeOfDay(byte i);
void setUpChandelier();
void randomLightShow(byte i);
void transition(byte i);
void playScript(byte i);
void fillIn(byte velocity, byte color1);
void curtainsOut(byte velocity, byte color1, byte color2);
void curtainsIn(byte velocity, byte color1, byte color2);
void swipeRight(byte velocity, byte color1, byte color2);
void swipeLeft(byte velocity, byte color1, byte color2);
void swipeRightMeter(byte velocity, byte color1, byte color2, byte steps);
void swipeLeftMeter(byte velocity, byte color1, byte color2, byte steps);
void fadeRight(int velocity, byte color1, byte color2);
void flashAll(byte velocity, byte color1, byte color2);
void christmasLights(int velocity, byte color1, byte color2, byte flips);
void flipColors(byte color1, byte color2);
void ballFadeTo(byte unit,byte color);
void ballGoTo(byte unit,byte color);
void fadeRightSlow(byte speed);
void setFadeSpeed(byte speed);
int programTargetIndex(int i, int j);
void setupAngles();
int calculateSinPosition(byte loopCounter, float sineFrequency);
void encoderHeightChange();
void updatePosition();
void moveTo();
void motorDirectionUp(byte unit);
void motorDirectionDown(byte unit);
void moveBall();
void displayFullTime();
void printPositionDigits(int digits);
void printLCDDigits(int digits);
void recalibrate();
void recalibratePendant(byte i);
void dropFallingStar(byte i);
void raiseFallenStar(byte i);
void clearLine(int i);
void printStatusMessage(byte i);
void addMinute();
void subtractMinute();
byte addHour(byte k);
byte subtractHour(byte k);
void showTime();
void clearRegisters();
void writeRegisters();
void setRegisterPin(int index, int value);
unsigned int read_shift_regs165();
void getMessage(byte index);
void scanfunc( byte addr, byte result );
void updateEncoder();
void pauseForTime(int k, boolean preventRandomLightShow);

#include "C:\Program Files\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Program Files\arduino-1.0.3\hardware\arduino\variants\standard\pins_arduino.h" 
#include "S:\Arduino\Kinetic Balls\Balls_July_2013\Balls_July_2013.ino"
#include "S:\Arduino\Kinetic Balls\Balls_July_2013\BlinkM_funcs.h"
#include "S:\Arduino\Kinetic Balls\Balls_July_2013\LiquidCrystal_I2C.cpp"
#include "S:\Arduino\Kinetic Balls\Balls_July_2013\LiquidCrystal_I2C.h"
#endif
