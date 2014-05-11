#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Arduino Uno
#define __AVR_ATmega328P__
#define 
#define _VMDEBUG 1
#define ARDUINO 103
#define ARDUINO_MAIN
#define __AVR__
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

#include "C:\Program Files\arduino-1.0.3\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Program Files\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_July_2013\Balls_July_2013.ino"
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_July_2013\BlinkM_funcs.h"
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_July_2013\LiquidCrystal_I2C.h"
#endif
