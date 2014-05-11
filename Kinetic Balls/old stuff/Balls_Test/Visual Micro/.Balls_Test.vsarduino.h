#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Arduino Uno
#define __AVR_ATmega328P__
#define _VMDEBUG 1
#define ARDUINO 103
#define __AVR__
#define F_CPU 16000000L
#define __cplusplus
#define __attribute__(x)
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
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

//
//
byte determineTimeOfDay(byte i);
void setUpChandelier();
void randomLightShow(byte i);
void curtainsOut(byte velocity, byte color1, byte color2);
void curtainsIn(byte velocity, byte color1, byte color2);
void swipeRight(byte velocity, byte color1, byte color2);
void swipeLeft(byte velocity, byte color1, byte color2);
void fadeRight(int velocity, byte color1, byte color2);
void flashAll(byte velocity, byte color1, byte color2);
void christmasLights(byte velocity, byte color1, byte color2, byte flips);
void flipColors(byte color1, byte color2);
void ballFadeTo(byte unit,byte color);
void ballGoTo(byte unit,byte color);
int programTargetIndex(int i, int j);
int calculateSinPosition(byte loopCounter, float sineFrequency);
void pauseTime(int k);
void encoderHeightChange();
void updatePosition();
void moveTo();
void motorDirectionUp(int unit);
void motorDirectionDown(int unit);
void moveBall();
void displayFullTime();
void printPositionDigits(int digits);
void printLCDDigits(int digits);
void recalibrate();
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
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_Test\Balls_Test.ino"
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_Test\BlinkM_funcs.h"
#include "\\SERVER\Data\Arduino\Kinetic Balls\Balls_Test\LiquidCrystal_I2C.h"
#endif
