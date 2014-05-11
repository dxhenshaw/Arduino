/*
 * SN74HC165N_shift_reg
 *
 * Program to shift in the bit values from a SN74HC165N 8-bit
 * parallel-in/serial-out shift register.
 *
 * This sketch demonstrates reading in 16 digital states from a
 * pair of daisy-chained SN74HC165N shift registers while using
 * only 4 digital pins on the Arduino.
 *
 * You can daisy-chain these chips by connecting the serial-out
 * (Q7 pin) on one shift register to the serial-in (Ds pin) of
 * the other.
/*
/* 
*/
#define NUMBER_OF_SHIFT_CHIPS   1  // How many shift register chips are daisy-chained.
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8  // Width of data (how many ext lines).
#define PULSE_WIDTH_USEC   15  // Width of pulse to trigger the shift register to read and latch.
#define POLL_DELAY_MSEC   20  // Optional delay between shift register reads.
#define BYTES_VAL_T unsigned int  // change the "int" to "long" If the NUMBER_OF_SHIFT_CHIPS is higher than 2.

int ploadPin165        = 6;  // Connects to Parallel load (= latch) pin (1) the 165
int clockEnablePin165  = 13;  // Connects to Clock Enable pin the 165
int dataPin165         = 5; // Connects to the Q7 (7) pin the 165
int clockPin165        = 4; // Connects to the Clock pin (2)the 165

BYTES_VAL_T pinValues165;
BYTES_VAL_T oldPinValues165;

/* This function is essentially a "shift-in" routine reading the
 * serial Data from the shift register chips and representing
 * the state of those pins in an unsigned integer (or long).
*/
BYTES_VAL_T read_shift_regs165()
{
    byte bitVal165;
    BYTES_VAL_T bytesVal165 = 0;

    /* Trigger a parallel Load to latch the state of the data lines,
    */
    digitalWrite(clockEnablePin165, HIGH);
    digitalWrite(ploadPin165, LOW);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(ploadPin165, HIGH);
    digitalWrite(clockEnablePin165, LOW);

    /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
    for(int i = 0; i < DATA_WIDTH; i++)
    {
        bitVal165 = digitalRead(dataPin165);

        /* Set the corresponding bit in bytesVal.
        */
        bytesVal165 |= (bitVal165 << ((DATA_WIDTH-1) - i));

        /* Pulse the Clock (rising edge shifts the next bit).
        */
        digitalWrite(clockPin165, HIGH);
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(clockPin165, LOW);
    }

    return(bytesVal165);
}

/* Dump the list of zones along with their current status.
*/
void display_pin_values165()
{
    //Serial.print("Pin States:\r\n");

    for(int i = 0; i < DATA_WIDTH; i++)
    {
        //Serial.print("  Pin-");
        //Serial.print(i);
        //Serial.print(": ");

        if((pinValues165 >> i) & 1)
            Serial.print("H");
        else
            Serial.print("L");

        //Serial.print("\r\n");
    }
    Serial.print("\r\n");
}

void setup()
{
    Serial.begin(9600);

    /* Initialize our digital pins...
    */
    pinMode(ploadPin165, OUTPUT);
    pinMode(clockEnablePin165, OUTPUT);
    pinMode(clockPin165, OUTPUT);
    pinMode(dataPin165, INPUT);

    digitalWrite(clockPin165, LOW);
    digitalWrite(ploadPin165, HIGH);

    /* Read in and display the pin states at startup.
    */
    pinValues165 = read_shift_regs165();
    display_pin_values165();
    oldPinValues165 = pinValues165;
}

void loop()
{
    /* Read the state of all zones.
    */
    pinValues165 = read_shift_regs165();

    /* If there was a chage in state, display which ones changed.
    */
    //if(pinValues != oldPinValues)
    //{
        //Serial.print("*Pin value change detected*\r\n");
        
        display_pin_values165();
        oldPinValues165 = pinValues165;
    //}

    delay(POLL_DELAY_MSEC);
}

