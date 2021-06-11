#include <Arduino.h>
#include <avr/power.h>

//------------------------------------------------------------
// Helpers for AVR/ATtiny's programmable clock-divisor
namespace ClkDiv 
{
    uint16_t clock_shifts = 0;

    // NOTE: be careful with this, setting clock_div too high can leave the ATtiny in
    // non-programmable state (clock rates too different from ArduinoISP)
    // - not fatal, but annoying!
    // - I THINK the largest divisor is 64 (resulting in CLK of 125 kHz) before problems occur
    //  - confirmed that divisor of 64 doesn't cause subsequent programming problems
    //  - if have need to use 128 or 256, can test and find out at that point
    void set(uint16_t divisor)
    {
        // valid divisor values are [1, 2, 4, ..., 256] -- 9 valid values
        static const clock_div_t clock_divs[] = {
            clock_div_1,
            clock_div_2,
            clock_div_4,
            clock_div_8,
            clock_div_16,
            clock_div_32,
            clock_div_64,
            clock_div_128,
            clock_div_256
        };

        // Limit divisor to <= 64
        // - this to prevent leaving chip in state where it can't talk to ArduinoISP
        // - remove this line if you really want to set it to 128 or 256
        divisor = divisor > 64 ? 64 : divisor;

        // find first set bit, starting at least-significant
        uint8_t div_index = 0;
        clock_div_t clock_div = clock_divs[div_index];
        while (div_index <= 8)
        {
            if (divisor & 1)
            {
                clock_div = clock_divs[div_index];
                clock_shifts = div_index;
                break;
            }
            divisor = divisor >> 1;
            div_index++;
        }

        clock_prescale_set( clock_div );
    }

    void ms_delay(unsigned long delay_ms)
    {
        delay(delay_ms >> clock_shifts);
    }
} // namespace ClkDiv 

//------------------------------------------------------------
// Circuit Pinout Definitions

// AnnoyATron-circuit pinouts
namespace AnnoyATron
{
    // Pin definition
    static const int LED_GRN = 0;  // one of your LEDs
    static const int LED_RED = 2;  // if your colors don't line up, swap the assignments here
    static const int BUZZER = 1;   // not active while powered by ICSP, flip the switch to test without unplugging
    static const int LDR = A2;     // Light Dependent Resistor
    static const int POT = A3;     // POTentiometer

    // LEDs are common anode so turn on by outputting low
    static const int LED_ON  = LOW;
    static const int LED_OFF = HIGH;
}

// PWM-LED driver pinouts
namespace PwmLedDriver
{
    // Pin definitions
    // - these are the same LED pins as for AnnoyATron
    // - interested to see if the gradation problems observed with RED LED are present here as well
    static const int LED_0 = 0;  // one of the LED outputs
    static const int LED_2 = 2;  // second LED output
    static const int POT = A3;   // POTentiometer -- use to control PWM level

    // chip outputs to Base of NPN Q
    // - high output forward biases Base-Emitter, turning on the LED
    static const int LED_ON  = HIGH;
    static const int LED_OFF = LOW;
}


//------------------------------------------------------------
// Test Classes

// AnnoyATron Test ClkDiv helper
struct Test_ClkDiv
{
    static void setup()
    {
        // set clock divisor (prescaler)
        // - by default, fuses set this to 8
        // - set to 1 for moar fastr! (at cost of chip using more electrons...)
        // - set to up to 64 for (supposedly) moar battery life
        //  - should test/quantify the battery savings
        // - values 128, 256 will probably leave tiny85 in state where it can't talk to defualt ArduinoISP
        //  - will need to modify ArduinoISP code to slow down the SPI_CLOCK...
        ClkDiv::set(1);

        pinMode(AnnoyATron::LED_GRN, OUTPUT);
        pinMode(AnnoyATron::LED_RED, OUTPUT);
    }

    static void loop() {
        digitalWrite(AnnoyATron::LED_GRN, AnnoyATron::LED_ON);
        digitalWrite(AnnoyATron::LED_RED, AnnoyATron::LED_OFF);
        ClkDiv::ms_delay(2000);
        digitalWrite(AnnoyATron::LED_GRN, AnnoyATron::LED_OFF);
        digitalWrite(AnnoyATron::LED_RED, AnnoyATron::LED_ON);
        ClkDiv::ms_delay(3000);
    }
};

// AnnoyATron Test analogWrite and LED brightness
// - recall LEDs are common anode, so low output pin = full brightness
// - suprising results for the red LED -- no real gradation noted and then its OFF for values >= 128
//  - TODO: test other red LED to see if this is feature of red LEDs
// - green LED -- much better behaved, noticeable light output even for value = 254
struct Test_analogWrite
{
    static void setup()
    {
        // set clock divisor (prescaler)
        ClkDiv::set(1);

        pinMode(AnnoyATron::LED_GRN, OUTPUT);
        pinMode(AnnoyATron::LED_RED, OUTPUT);
    }

    static void flash(int pin, unsigned long duration=250)
    {
        digitalWrite(pin, AnnoyATron::LED_ON);
        ClkDiv::ms_delay(duration);
        digitalWrite(pin, AnnoyATron::LED_OFF);
    }

    static void cycle(int pin0, int pin1, uint8_t values[8], unsigned long flash_duration=250)
    {
        // ramps pin0, using pin1 for signalling
        digitalWrite(pin1, AnnoyATron::LED_OFF);
        for( int i=0; i<8; i++ )
        {
            analogWrite(pin0, values[i]);
            ClkDiv::ms_delay(2000);

            // blink the alternate LED to signal a level change
            digitalWrite(pin0, AnnoyATron::LED_OFF);
            flash(pin1, flash_duration);
        }
    }

    static void loop() {

        //--------------------
        // Alternate extremes for RED
        // - wow, seems full brightness to 127, then OFF at 128
        uint8_t xr_values[8] = {0, 125, 126, 127, 0, 128, 130, 132};
        cycle(AnnoyATron::LED_RED, AnnoyATron::LED_GRN, xr_values, 20);

        //--------------------
        // Alternate extremes for GRN
        // - wow, low level visibility right to the end
        uint8_t xg_values[8] = {0, 249, 250, 251, 252, 253, 254, 255};
        cycle(AnnoyATron::LED_GRN, AnnoyATron::LED_RED, xg_values, 20);

        //--------------------
        // Linear ramp (8 values)
        uint8_t lin_values[8] = {0, 36, 73, 109, 146, 182, 219, 255};

        // ramp red LED
        // - RED: looks very bright for first 4 values, and OFF for last 4 values
        cycle(AnnoyATron::LED_RED, AnnoyATron::LED_GRN, lin_values, 20);

        // ramp green LED
        // - GRN: overall not as bright but provides noticeable gradation thru the levels
        cycle(AnnoyATron::LED_GRN, AnnoyATron::LED_RED, lin_values, 20);

        //--------------------
        // Log ramp (8 values)
        uint8_t log_values[8] = {0, 4, 8, 16, 32, 64, 128, 255};

        // ramp red LED
        // - RED: lights for first 6, but brightness gradation not obvious
        cycle(AnnoyATron::LED_RED, AnnoyATron::LED_GRN, log_values, 20);

        // ramp green LED
        // - GRN: linear ramp provides better gradation
        cycle(AnnoyATron::LED_GRN, AnnoyATron::LED_RED, log_values, 20);
    }
};

//------------------------------------------------------------
//Test_ClkDiv test;
Test_analogWrite test;

// put your setup code here, to run once:
void setup() {
    test.setup();
}


// put your main code here, to run repeatedly:
void loop() {
    test.loop();
}

