// VisiPack - David Henshaw & Morgan Renault, March 2014--
// v1 - 03/14 - Includes basic animation and transitions
// v2 - 04/14 - Improvements to animations

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif
#define PIN 6	// data pin to NeoPixel LED strip

// MATRIX DECLARATION - See NeoMatrix documentation for details
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(12, 12, 1, 1, PIN,
  NEO_TILE_TOP   + NEO_TILE_LEFT   + NEO_TILE_ROWS   + NEO_TILE_ZIGZAG +
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB + NEO_KHZ800);	

#define BLACK    0x0000		// Color definitions begin
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

byte const numberAnimations = 21;		// how many total animations are there?
byte nextAnimation = 0;					// next animation to play
byte animationQueue[numberAnimations];	// store animations to prevent duplicates xxx use constant
int xOffset =  0;						// used for playing Pong - directional value
int yOffset = -1;						// Pong - directional value
void setup() {
  matrix.begin();						// get matrix ready			
  matrix.setBrightness(20);				// keep at 20% - batteries seem to not be able to drive at more than this %
  matrix.setTextColor(GREEN);
  matrix.setTextWrap(false);
  randomSeed(analogRead(0));			// to help animation queue from being different each time
  populateAnimationQueue();
}
void loop() {
	matrix.fillScreen(0);					// start with a blank screen
	runAnimation(calculateNextAnimation());	// run the animation
	//runAnimation(19);
	//populateAnimationQueue();
	//calculateNextAnimation();
	runTransition(random(0,5));				// run a random transition from 0 thru 4
}

void runAnimation(byte animation) {					// run animation based on value of byte passed in
	switch (animation) {
	case 0:
		paintPlaidLines(random(10,16));				// between 10 and 15 plaid lines
		break;
	case 1:
		paintSmileyFace(YELLOW, randomColor());		// give eyes random color
		break;
	case 2:
		triangleAlertAnimation(random(4,11));		// 4-10 flashing exclamation marks !
		break;
	case 3:
		flashingCornerAlertAnimation(random(8,16));	// 8-15 flashing exclamation marks !
		break;
	case 4:
		countdownAnimation(random(3,10));			// countdown from 3-9
		break;
	case 5:
		countdownAnimation(3);						// countdown from 3
		paintSmileyFace(GREEN, YELLOW);				// then paint smiley face
		delay(2000);								// and wait 2 seconds
	case 6:
		paintZeroMpg();
		break;
	case 7:
		for (byte i = 0; i < random(3,7); i++) {
			paintChevron();
		}
		break;
	case 8:
		paintEqualizerBars();
		break;
	case 9:
		paintExpandingCircle(random(10,16));
		break;
	case 10:
		paintExpandingBox(random(10,16));
		break;	
	case 11:
		paintSinWave();
		break;	
	case 12:
		paintFlyingBoxes();
		break;
	case 13:
		paintFirework(random(4,6));
		break;	
	case 14:
		paintScrollingBoxes(random(30,51));
		break;	
	case 15:
		for (byte i = 0; i < 5; i++) {
			matrix.fillScreen(0);
			drawSpiral(randomColor());
			drawSpiral(BLACK);
		}
		break;	
	case 16:
		drawHorizontalLine();
		break;	
	case 17:
		for (byte i = 0; i < random(5,9); i++) {
			drawHeartBeat();
		}
		break;	
	case 18:
		drawBouncingBall();
		break;	
	case 19:
		playPong();
		break;	
	case 20:
		for (byte i = 0; i < random(10,21); i++) {
			drawCollidingChevrons();
		}
		break;		
	}
}
void runTransition(byte transition) {					// run animation based on value of byte passed in
	switch (transition) {
	case 0:
		swipeRightTransition(randomColor());			
		break;
	case 1:
		swipeTopDownTransition(randomColor());	
		break;
	case 2:
		for (byte i = 0; i < random(1,5); i++) {
			decreasingCirclesTransition(randomColor());		
		}
		break;
	case 3:
		for (byte i = 0; i < random(1,5); i++) {
			expandingCirclesTransition(randomColor());		
		}
		break;
	case 4:
		matrixDropTransition();		
		break;
	}
}

void swipeRightTransition(uint16_t color) {
	for (byte i = 0; i < 12; i++) {
		//matrix.drawLine(i-1, 0, i-1, 11, BLACK);
		matrix.drawFastVLine(i-1,  0, 12, BLACK);
		//matrix.drawLine(i, 0, i, 11, color);
		matrix.drawFastVLine(i,  0, 12, color);
		matrix.show();
		delay(25);
	}
}
void swipeTopDownTransition(uint16_t color) {
	for (byte i = 0; i < 12; i++) {
		//matrix.drawLine(0, i-1, 11, i-1, BLACK);
		matrix.drawFastHLine(0, i-1, 12, BLACK);
		//matrix.drawLine(0, i, 11, i, color);
		matrix.drawFastHLine(0, i, 12, color);
		matrix.show();
		delay(25);
	}
}
void decreasingCirclesTransition(uint16_t color) {
	for (byte i = 6; i > 0; i--) {
		matrix.drawCircle(5, 5, i+1, BLACK);
		matrix.drawCircle(5, 5, i  , color);
		matrix.show();
		delay(50);
	}
}
void expandingCirclesTransition(uint16_t color) {
	for (byte i = 1; i < 7 ; i++) {
		matrix.drawCircle(5, 5, i-1, BLACK);
		matrix.drawCircle(5, 5, i  , color);
		matrix.show();
		delay(50);
	}
}
void matrixDropTransition() {
	for (byte i = 0; i < 13; i++) {
		for (byte j = 0 ; j < 12; j = j + 2) { // j is column
			matrix.drawFastVLine(j+1, 0, i, BLACK);
			for (byte k = i-3 ; k <= i; k++) { // k is length
				matrix.drawPixel(j, k, GREEN);
				matrix.show();
			}
			matrix.drawFastVLine(j, 0, i-1, matrix.Color(0, random(100, 150), 0)); // random dim green
			matrix.show();
		}
	}
}

void paintPlaidLines(byte repeat) {
	for (byte j = 0; j < repeat; j++) {
		byte row = random(0,11);	// 0 thru 11 (lines are 2 lines thick)
		byte column = random(0,11);
		uint16_t color = randomColor();

		for (byte i = 0; i < 12; i++) {
			matrix.drawPixel(i, row-1, BLACK);
			matrix.drawRect(i, row, 1, 2, color); // stripe on row heading right
			matrix.drawPixel(i, row+2, BLACK);
			matrix.show();
			delay(15);
		}

		color = randomColor();

		for (byte i = 0; i < 12; i++) {
			matrix.drawPixel(column+2, i, BLACK);
			matrix.drawRect(column, i, 2, 1, color); // stripe on row heading down
			matrix.drawPixel(column-1, i, BLACK);
			matrix.show();
			delay(15);
		}
	}
}
void paintSmileyFace(uint16_t lineColor, uint16_t eyeColor) {
	matrix. drawCircle(5, 5, 3, lineColor);						// smile
    matrix. fillRect(0, 0, 12, 6, BLACK);						// erase top half of circle
    matrix. drawCircle(5, 5, 5, lineColor);						// face outline
    matrix. drawRect(3, 3, 2, 2, eyeColor);						// left eye
    matrix. drawRect(7, 3, 2, 2, eyeColor);						// right eye
	matrix.show();
	delay(2000);
}
void triangleAlertAnimation(byte repeat) {
	for (byte i = 0; i < repeat; i++) {
		paintTriangleAlert(RED, RED);
		delay(300);
		paintTriangleAlert(YELLOW, YELLOW);
		delay(300);
	}
}
void paintTriangleAlert(uint16_t boxColor, uint16_t exclamationColor ) {

	//matrix.fillTriangle(6, 2, 1, 11, 10, 11, triangleColor); // large triangle
	byte width = 1;
	// paint exploding box from center
	for (byte i = 1; width < 12; i++) {	// expanding
		matrix.drawRect((7-i), (7-i), width, width, boxColor);
		matrix.show();
		delay(50);
		matrix.fillScreen(0);
		width = width + 2;
	}
	paintLargeExclamationMark(exclamationColor);
	matrix.show();
}
void paintExclamationMark(uint16_t exclamationColor) {
	matrix.drawRect(5, 3, 2, 4, exclamationColor); // top part of !
	matrix.drawRect(5, 8, 2, 2, exclamationColor); // lower part of !
}
void paintLargeExclamationMark(uint16_t exclamationColor) {
	matrix.fillRect(4, 0, 4, 8, exclamationColor); // top part of !
	matrix.fillRect(4, 9, 4, 3, exclamationColor); // lower part of !
}
void flashingCornerAlertAnimation(byte repeat) {
	for (byte i = 0; i < repeat; i++) {
		matrix.fillScreen(0);
		matrix.drawRect(9, 1, 2, 2, RED);		// top right corner red box
		matrix.drawRect(1, 1, 2, 2, RED);		// top left
		matrix.drawRect(9, 9, 2, 2, RED);		// bottom right
		matrix.drawRect(1, 9, 2, 2, RED);		// bottom left
		paintExclamationMark(GREEN);
		drawAnimatedBox(YELLOW);				// paint animated box around each corner box
		delay(100);
		paintExclamationMark(BLACK);
		drawAnimatedBox(BLACK);
		delay(100);
	}
}
void drawAnimatedBox(uint16_t color) {
	int xCoord = 0;
	int yCoord = 3;

	for (yCoord; yCoord > 0; yCoord--) {
		paintFourPixels(xCoord, yCoord, color);
	}
	for (xCoord; xCoord < 3; xCoord++) {
		paintFourPixels(xCoord, yCoord, color);
		}
	for (yCoord; yCoord < 3; yCoord++) {
		paintFourPixels(xCoord, yCoord, color);
	}
	for (xCoord; xCoord > 0; xCoord--) {
		paintFourPixels(xCoord, yCoord, color);
	}
}
void paintFourPixels(byte xCoord, byte yCoord, uint16_t color){
	drawPixelandDelay(xCoord  , yCoord  , color);
	drawPixelandDelay(xCoord+8, yCoord  , color);
	drawPixelandDelay(xCoord  , yCoord+8, color);
	drawPixelandDelay(xCoord+8, yCoord+8, color);
}
void countdownAnimation(byte count) {
	for (byte i = count; i > 0; i--) { 
		matrix.setCursor(4,2);
		matrix.print(i);
		matrix.drawRoundRect(0,0,12,12,2,matrix.Color(random(80, 200), 0, 0));	// random red rectangle
		matrix.show();
		delay(3000/count);	// ensure countdown lasts for 3 seconds
		matrix.fillScreen(0);
	}
}
void paintZeroMpg() {
	paintHorizontalBorders();
	for (int x = matrix.width(); x > -25 ; x--) {	// needs to be int because value goes below 0
		matrix.setCursor(x,2);
		matrix.print(F("0MPG"));
		matrix.show();
		delay(90);
		matrix.fillRect(0, 2, 12, 8, BLACK);		// black out text area before next scroll occurs
	}
}
void paintHorizontalBorders() {
	drawHorizontalLineRight(0, randomColor());
	drawHorizontalLineLeft(10, randomColor());
}
void paintChevron() {
	for (int i = matrix.height(); i > -27; i--) {	// needs to be int because value goes below 0
		drawOneChevron(i   , RED);
		drawOneChevron(i+ 6, YELLOW);
		drawOneChevron(i+12, BLUE);
		drawOneChevron(i+18, GREEN);
		matrix.show();
		delay(60);
		matrix.fillScreen(0);
	}
}
void drawCollidingChevrons() {
	for (int i = matrix.height(); i > -8; i--) {	// needs to be int because value goes below 0
		animateCollidingChevrons(i, GREEN, CYAN);	// left to right
	}
	for (int i = -8; i < matrix.height(); i++) {	// needs to be int because value goes below 0
		animateCollidingChevrons(i, GREEN, CYAN);	// right to left
	}	
}
void animateCollidingChevrons(int i, uint16_t chevron1Color, uint16_t chevron2Color) {
		matrix.fillTriangle( i, 0, i+6, 6, i, 11, chevron1Color);				// draw triangle
		matrix.fillTriangle( i, 3, i+3, 6, i,  8, BLACK);						// draw a black triangle to create a chevron
		matrix.fillTriangle( (12-i), 0, 12-i-6, 6, (12-i), 11, chevron2Color);	// draw triangle
		matrix.fillTriangle( (12-i), 3, 12-i-3, 6, (12-i),  8, BLACK);			// draw a black triangle to create a chevron		
		
		matrix.show();
		delay(15);
		matrix.fillScreen(0);
}
void drawOneChevron(int yAxis, uint16_t chevronColor) {
	matrix.fillTriangle( 5, yAxis,   0, yAxis+8, 10, yAxis+8, chevronColor);	// draw triangle
	matrix.fillTriangle( 5, yAxis+2, 2, yAxis+8,  8, yAxis+8, BLACK);			// draw a black triangle to create a chevron
}
void paintEqualizerBars() {
	byte barValue[5];					// prepare 5 bars
	for (byte i = 0; i < 6; i++) {		// seed initial values
		barValue[i] = random(0,13);
	}

	for (byte j = 0 ; j < 70; j++) {	// do 70 bar cycles
		matrix.fillScreen(0);
		for (byte i = 1; i < 7; i++) {
			paintOneBar(barValue[i-1],i);
			barValue[i-1] = (barValue[i-1] + random(-2,3));
			if (barValue[i-1] > 20) barValue[i-1] = 0; // bar value went below 0 to 255
			if (barValue[i-1] > 13) barValue[i-1] = 12;
		}
		matrix.show();
		delay(100);
	}
}
void paintOneBar(byte height, byte barNumber) {
	matrix.fillRect((barNumber-1)*2, (12-height), 2, height, RED);
}
void paintExpandingCircle(byte repeat) {
	do {
		uint16_t circleColor = randomColor();

		for (byte radius = 0; radius < 6; radius ++) {	// expanding circle
			matrix.fillScreen(0);
			matrix.fillCircle(6, 6, radius, circleColor);
			if (radius > 2) matrix.fillCircle(6, 6, 2, BLACK);	// donut hole
			matrix.show();
			delay(random(60,100));
			
		}
	
		for (byte radius = 5; radius > 0; radius --) {	// decreasing circle
			matrix.fillScreen(0);
			matrix.fillCircle(6, 6, radius, circleColor);
			if (radius > 2) matrix.fillCircle(6, 6, 2, BLACK);	// donut hole
			matrix.show();
			delay(random(20,70));
			
		}
		repeat --;
	} while (repeat > 0);
}
void paintExpandingBox(byte repeat) {
	do {
		uint16_t boxColor = randomColor();

		for (byte width = 1; width < 7; width ++) {	// expanding
			matrix.fillScreen(0);
			matrix.fillRect(0, 0, width, width, boxColor);
			matrix.show();
			delay(random(40,60));
		}
	
		for (byte width = 6; width > 0; width --) {	// decreasing
			matrix.fillScreen(0);
			matrix.fillRect((12-width), (12-width), width, width, boxColor);
			matrix.show();
			delay(random(40,60));
		}
		repeat --;
	} while (repeat > 0);
}
void paintSinWave() {
	float sineFrequency = random(23,50);
    sineFrequency = sineFrequency / 100;								// convert to a number between 2.3 and 4.9
	byte index = 0;
	do {
		for (int i=0; i < 12; i++) {									
			matrix.drawPixel(i, calculateSinPosition(index-1, sineFrequency), BLACK);	// remove prior value
			matrix.drawPixel(i, calculateSinPosition(index, sineFrequency),   WHITE);	// paint new value
			matrix.show();
			index++;
		  }
		delay(25);
		index = index - 11;
	} while (index < 70);
}
void paintFlyingBoxes() {
	byte index = 0;
	byte repeat = 25;
	do {
		index = 0;
		do {
			matrix.fillScreen(0);
			for (byte j = index ; j < (index+13) ; j=j+4) {
				drawOneBox( 0, j        , RED   , 2);
				drawOneBox( 7, j+1      , BLUE  , 2);
				drawOneBox( 3, (12-j)   , GREEN , 2);
				drawOneBox(10, (12-j+1) , YELLOW, 2);
			}
			matrix.show();
			index++;
			delay(125);
			} while (index < 4);
		repeat--;
	} while (repeat > 0);
	}	
void paintScrollingBoxes(byte repeat) {
	byte index = 0;	// was int
	uint16_t box1Color = randomColor();
	uint16_t box2Color = randomColor();
	uint16_t box3Color = randomColor();
	uint16_t box4Color = randomColor();
	byte box1xAxis = random(0,9);
	byte box2xAxis = random(0,9);
	byte box3xAxis = random(0,9);
	byte box4xAxis = random(0,9);

	do {
		index = 0;
		do {
			matrix.fillScreen(0);
			if (index > 0) matrix.fillRect(box1xAxis, 0, 4, index, box1Color);
			drawOneBox(box2xAxis, index   , box2Color, 4);
			drawOneBox(box3xAxis, index+4 , box3Color, 4);
			drawOneBox(box4xAxis, index+8 , box4Color, 4);
			matrix.show();
			delay(50);
			index++;
			} while (index < 5);
		box4Color = box3Color;
		box3Color = box2Color;
		box2Color = box1Color;
		box1Color = randomColor();
		box4xAxis = box3xAxis;
		box3xAxis = box2xAxis;
		box2xAxis = box1xAxis;
		box1xAxis = random(0,9);
		repeat--;
	} while (repeat > 0);
	}	
void drawOneBox(byte xAxis, byte yAxis, uint16_t boxColor, byte boxSize) {
	matrix.fillRect(xAxis, yAxis, boxSize, boxSize, boxColor);
}
void paintFirework(byte repeat) {
	for (byte i = 0; i < repeat; i++) {
		byte xAxis = random(2, 10);
		byte trailHeight = random(5, 12);
		drawFireworkTrail(xAxis, trailHeight);					// draw trail
		drawInitialExplosion(xAxis, trailHeight);				// initial explosion
		drawFullExplosion(xAxis, trailHeight, random(3, 8)); 	// full explosion
	}
}
void drawFireworkTrail(byte xAxis, byte trailHeight) {
	for (byte i = 11; i > (11 - trailHeight) ; i--) {	// draw initial trail
		matrix.drawPixel(xAxis, i  , YELLOW);			// add preceeding brightness to trail
		matrix.drawPixel(xAxis, i+1, WHITE);			// draw regular part of trail
		matrix.show();
		delay(100);
	}
	for (byte i = 11; i > (13 - trailHeight) ; i--) {	// remove most of the trail
		matrix.drawPixel(xAxis, i, BLACK);
		matrix.show();
		delay(70);
	}
	delay(200);
}
void drawInitialExplosion(byte xAxis, byte trailHeight) {	// two hollow circles appear
	matrix.fillCircle(xAxis, (11-trailHeight), 2, YELLOW);
	matrix.show();
	delay(40);
	matrix.drawCircle(xAxis, (11-trailHeight), 4, WHITE);
	matrix.show();
	delay(80);
}
void drawFullExplosion(byte xAxis, byte trailHeight, byte explosionSize) {
	for (byte i = 0; i <= explosionSize; i++) {	// paint explosion
		matrix.drawCircle(xAxis, (11-trailHeight), i, randomColor());
		matrix.show();
		delay(20);
	}
	delay(250);		// hold for fraction of a second
	for (byte i = 0; i <= (explosionSize+1); i++) {	// remove explosion
		matrix.fillCircle(xAxis, (11-trailHeight), i, BLACK);
		matrix.show();
		delay(150);
	}
}
void drawSpiral(uint16_t color) {
	int xCoord = 5;
	int yCoord = 6;
	byte offset = 2;

	for (byte i = 0; i < 3; i++) {				// do yaxis decrement
		int destinationYAxis = yCoord - offset;
		for (yCoord; yCoord > destinationYAxis; yCoord--) {
			drawPixelandDelay(xCoord, yCoord, color);
		}
		yCoord = destinationYAxis;

		int destinationXAxis = xCoord + offset;	// do xaxis increment
		for (xCoord; xCoord < destinationXAxis; xCoord++) {
			drawPixelandDelay(xCoord, yCoord, color);
		}
		xCoord = destinationXAxis;

		offset = (offset + 2);

		destinationYAxis = yCoord + offset;		// do yaxis increment
		for (yCoord; yCoord < destinationYAxis; yCoord++) {
			drawPixelandDelay(xCoord, yCoord, color);
		}
		yCoord = destinationYAxis;

		destinationXAxis = xCoord - offset;		// do xaxis decrement
		for (xCoord; xCoord > destinationXAxis; xCoord--) {
			drawPixelandDelay(xCoord, yCoord, color);
		}
		xCoord = destinationXAxis;

		offset = (offset + 2);
	}	// do this loop again
}
void drawPixelandDelay(int xAxis, int yAxis, uint16_t color) {
	if (xAxis >= 0 && yAxis >= 0 && xAxis < 12 && yAxis < 12) {
		matrix.drawPixel(xAxis, yAxis, color);
		delay(5);
	}
	matrix.show();
}
void drawHorizontalLine() {
	for (byte yAxis = 0; yAxis < 12; yAxis = yAxis + 3) {
		drawHorizontalLineRight(yAxis, randomColor());
		yAxis = yAxis + 3;
		drawHorizontalLineLeft(yAxis, randomColor());
	}
}
void drawHorizontalLineRight(byte yAxis, uint16_t color) {
	for (byte maxXAxis = 11; maxXAxis > 0; maxXAxis--) {
		for (byte i = 0; i < maxXAxis; i++) {
			drawOneBox(i, yAxis, color, 2);
			matrix.show();
			delay(10);
			drawOneBox(i, yAxis, BLACK, 2);
		}
		drawOneBox(maxXAxis-1, yAxis, color, 2);
	}
}
void drawHorizontalLineLeft(byte yAxis, uint16_t color) {
	for (byte maxXAxis = 0; maxXAxis < 12; maxXAxis++) {
		for (byte i = 11; i > maxXAxis; i--) {
			drawOneBox(i, yAxis, color, 2);
			matrix.show();
			delay(10);
			drawOneBox(i, yAxis, BLACK, 2);
		}
		drawOneBox(maxXAxis, yAxis, color, 2);
	}
}
void drawHeartBeat() {
	byte yMax = random(0,3);					// define the characteristics of this heartbeat
	byte yMin = random(9, 12);
	byte xBegin = random(0, 4);
	byte xFirstOffset = random(2,4);
	byte xSecondOffset = random(2,4);
	byte xEndOffset = random(2,4);

	for (int i = 0; i < xBegin; i++) {			// begin flat line
		drawPixelandDelay(i, 6, RED);
		delay(50);
		matrix.drawFastVLine(i-6,0,12,BLACK);	// black out anything to the immediate left
	}

	matrix.drawLine(xBegin, 6, xBegin+xFirstOffset, yMax, RED);
	matrix.show();
	delay(150);
	matrix.drawLine(xBegin+xFirstOffset, yMax, xBegin+xFirstOffset+xSecondOffset, yMin, RED);
	matrix.show();
	delay(250);
	matrix.drawLine(xBegin+xFirstOffset+xSecondOffset, yMin, xBegin+xFirstOffset+xSecondOffset+xEndOffset, 6, RED);
	matrix.show();
	delay(100);

	for (int i = xBegin+xFirstOffset+xSecondOffset+xEndOffset; i < 26; i++) {	// end flat line
		drawPixelandDelay(i, 6, RED);
		delay(50);
		matrix.drawFastVLine(i-14,0,12,BLACK);
	}
}
void drawBouncingBall() {
	drawBox(randomColor());
	
	int angle = 0;	// set direction as upwards
	calculateNewOffsets(angle);
	int xAxis = 5;
	int yAxis = 5;

	for (int i = 0; i < 350; i++){
		matrix.fillCircle(xAxis, yAxis, 1, WHITE);
		matrix.show();
		matrix.fillCircle(xAxis, yAxis, 1, BLACK);

		if (xAxis > 8 || xAxis < 4 || yAxis > 8 || yAxis < 4) {	// have we hit a wall?
			angle = calculateNewAngle(angle, xAxis, yAxis);
			calculateNewOffsets(angle);
		}
		delay(50);
		xAxis = xAxis + xOffset;
		yAxis = yAxis + yOffset;
	}
}
void playPong() {
	int angle = 90;	// right direction to begin with
	calculateNewOffsets(angle);
	int xAxis = 5;
	int yAxis = 5;

	for (int i = 0; i < 350; i++){
		matrix.drawPixel(xAxis, yAxis, WHITE);
		matrix.show();
		matrix.drawPixel(xAxis, yAxis, BLACK);

		if (angle > 180) drawPadle( 0, yAxis);	// if ball coming to the left, move left padle
		if (angle < 180) drawPadle(11, yAxis);	// if ball coming to the right, move right padle

		if (xAxis > 10 || xAxis < 1 || yAxis > 10 || yAxis < 1) {	// have we hit a wall?
			angle = calculateNewAngle(angle, xAxis, yAxis);
			calculateNewOffsets(angle);
		}
		delay(60);
		xAxis = xAxis + xOffset;
		yAxis = yAxis + yOffset;
	}
}
int calculateNewAngle(int angle, int xAxis, int yAxis) {
	if	    (angle ==   0)				{angle = 135;}
	else if (angle ==  45 && xAxis > 9) {angle = 270;}
	else if (angle ==  45 && yAxis < 2) {angle = 135;}
	else if (angle ==  90)				{angle = 315;}
	else if (angle == 135 && xAxis > 9) {angle = 225;}
	else if (angle == 135 && yAxis > 9) {angle =  45;}
	else if (angle == 180)				{angle =  45;}
	else if (angle == 225 && xAxis < 2) {angle = 135;}
	else if (angle == 225 && yAxis > 9) {angle = 315;}
	else if (angle == 270)				{angle =  90;}
	else if (angle == 315 && xAxis < 2) {angle =  45;}
	else if (angle == 315 && yAxis < 2) {angle = 225;}

	return angle;
}
void calculateNewOffsets(int angle){
	switch (angle) {			// what are new x and y offsets?
				case 0:
					xOffset =  0;
					yOffset = -1;
					break;
				case 45:
					xOffset =  1;
					yOffset = random(-2,1);	// to prevent endless back-and-forth
					break;
				case 90:
					xOffset =  1;
					yOffset =  random(-1,1); // was 0;
					break;
				case 135:
					xOffset =  1;
					yOffset =  1;
					break;
				case 180:
					xOffset =  0;
					yOffset =  1;
					break;
				case 225:
					xOffset = -1;
					yOffset = +1;
					break;
				case 270:
					xOffset = -1;
					yOffset =  0;
					break;
				case 315:
					xOffset = -1;
					yOffset = -1;
					break;
			}
}
void drawBox(uint16_t color) {
	byte xAxis = 6;
	byte yAxis = 1;

	do {
		drawPixelandDelay((11-xAxis),  0, color);
		drawPixelandDelay(xAxis,       0, color);
		drawPixelandDelay((11-xAxis), 11, color);
		drawPixelandDelay(xAxis,      11, color);
		xAxis++;
	} while (xAxis < 12);
	do {
		drawPixelandDelay( 0,      yAxis, color);
		drawPixelandDelay(11,      yAxis, color);
		drawPixelandDelay( 0, (11-yAxis), color);
		drawPixelandDelay(11, (11-yAxis), color);
		yAxis++;
	} while (yAxis < 6);
}
void drawPadle(byte xAxis, byte yAxis){
	matrix.drawFastVLine(xAxis, 0, 12, BLACK);
	matrix.drawFastVLine(xAxis, yAxis-1, 3, YELLOW);
}

uint32_t randomColor() {
	uint8_t r = random(255);
    uint8_t g = random(255);
    uint8_t b = random(255);

	return matrix.Color(r, g, b);
}
int calculateSinPosition(byte loopCounter, float sineFrequency) {
  float transformedSinWave = 0;
  float sinWave = 0;							// prepare a floating point number
  sinWave = sin(loopCounter * sineFrequency);	// this is the value, in radians, for this point of the sine wave
  transformedSinWave = ((sinWave + 1) * 6);
  byte index = transformedSinWave;				// ram floating point number into integer... it will be close enough for union work
  return index;
}
int calculateNextAnimation() {
	do {
		byte index = 0;							// start at top of array
		int nextAnimation = 99;					// false value
		do {
			if (animationQueue[index] != 99) {	// if this index position is not bogus one
				nextAnimation = animationQueue[index];
				animationQueue[index] = 99;		// mark as zero for next time round
				return nextAnimation;			// returns value and breaks
			}
			index++;
		} while (index <= numberAnimations);
		if (nextAnimation == 99) populateAnimationQueue(); // is the array full of zeroes? means we need to repopulate it
	} while (nextAnimation == 99);
}
void populateAnimationQueue() {						// queue is empty, so repopulate it
	for (byte i = 0; i < numberAnimations; i++) {	// first insert values into the array
		animationQueue[i] = i;
	}
	for (byte i = 0; i < numberAnimations; i++) {	// now shuffle the array
		byte pos = random(i, numberAnimations);		// between i and max animations - 1
		byte t = animationQueue[i];   
		animationQueue[i] = animationQueue[pos];
		animationQueue[pos] = t;
	}
}