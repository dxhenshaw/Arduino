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
void runAnimation(byte animation);
void runTransition(byte transition);
void swipeRightTransition(uint16_t color);
void swipeTopDownTransition(uint16_t color);
void decreasingCirclesTransition(uint16_t color);
void expandingCirclesTransition(uint16_t color);
void matrixDropTransition();
void paintPlaidLines(byte repeat);
void paintSmileyFace(uint16_t lineColor, uint16_t eyeColor);
void triangleAlertAnimation(byte repeat);
void paintTriangleAlert(uint16_t boxColor, uint16_t exclamationColor );
void paintExclamationMark(uint16_t exclamationColor);
void paintLargeExclamationMark(uint16_t exclamationColor);
void flashingCornerAlertAnimation(byte repeat);
void drawAnimatedBox(uint16_t color);
void paintFourPixels(byte xCoord, byte yCoord, uint16_t color);
void countdownAnimation(byte count);
void paintZeroMpg();
void paintHorizontalBorders();
void paintChevron();
void drawCollidingChevrons();
void animateCollidingChevrons(int i, uint16_t chevron1Color, uint16_t chevron2Color);
void drawOneChevron(int yAxis, uint16_t chevronColor);
void paintEqualizerBars();
void paintOneBar(byte height, byte barNumber);
void paintExpandingCircle(byte repeat);
void paintExpandingBox(byte repeat);
void paintSinWave();
void paintFlyingBoxes();
void paintScrollingBoxes(byte repeat);
void drawOneBox(byte xAxis, byte yAxis, uint16_t boxColor, byte boxSize);
void paintFirework(byte repeat);
void drawFireworkTrail(byte xAxis, byte trailHeight);
void drawInitialExplosion(byte xAxis, byte trailHeight);
void drawFullExplosion(byte xAxis, byte trailHeight, byte explosionSize);
void drawSpiral(uint16_t color);
void drawPixelandDelay(int xAxis, int yAxis, uint16_t color);
void drawHorizontalLine();
void drawHorizontalLineRight(byte yAxis, uint16_t color);
void drawHorizontalLineLeft(byte yAxis, uint16_t color);
void drawHeartBeat();
void drawBouncingBall();
void playPong();
int calculateNewAngle(int angle, int xAxis, int yAxis);
void calculateNewOffsets(int angle);
void drawBox(uint16_t color);
void drawPadle(byte xAxis, byte yAxis);
uint32_t randomColor();
int calculateSinPosition(byte loopCounter, float sineFrequency);
int calculateNextAnimation();
void populateAnimationQueue();

#include "C:\Program Files\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Program Files\arduino-1.0.3\hardware\arduino\variants\standard\pins_arduino.h" 
#include "\\SERVER\Data\Arduino\VisiPack\VisiPack.ino"
#endif
