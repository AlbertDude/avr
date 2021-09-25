#include <Arduino.h>
#include <avr/power.h>

//============================================================
// Helpers

//------------------------------------------------------------
// Helper for AVR/ATtiny's programmable clock-divisor
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

    uint16_t get_clock_shifts()
    {
        return clock_shifts;
    }

    void ms_delay(unsigned long delay_ms)
    {
        delay(delay_ms >> clock_shifts);
    }
} // namespace ClkDiv 

// LED Blinker class
// - toggles specified GPIO pin at specified interval
//  - i.e. cycle period is twice the specified interval
// - calling code needs to call ::Loop() periodically at rate faster than the specified interval
class BlinkerL
{
public:
    BlinkerL(uint8_t pin, unsigned long interval_millis)
        : interval_millis_(interval_millis)
        , pin_(pin)
    {
        // TODO: check that pin is valid GPIO
        pinMode(pin, OUTPUT); // Set specified pin to be an OUTPUT
        prev_toggle_millis_ = 0;
        output_state_ = LOW;
    }

    void Loop()
    {
        unsigned long now = millis();
        unsigned long scaled_interval = interval_millis_ >> ClkDiv::get_clock_shifts();
        if( (prev_toggle_millis_ == 0) || (now >= prev_toggle_millis_ + scaled_interval) )
        {
            output_state_ = (output_state_ == LOW) ? HIGH : LOW;
            digitalWrite(pin_, output_state_); //
            prev_toggle_millis_ = now;
        }
    }

private:
    unsigned long interval_millis_;
    uint8_t pin_;
    unsigned long prev_toggle_millis_;
    int output_state_;
};

//------------------------------------------------------------
// Helper for blinking-LED based debugging
//
// Blinks LED to "display" a numeric value as follows:
// - long OFF, long ON, long OFF (indicating sync)
// - for each digit, N, in the value:
//  - repeat N times:
//    - short ON, short OFF
//  - if more digits, long OFF
// - where SHORT = base_duration
// - where  LONG = 3 * base_duration
// 
// e.g.s:
// "2"   = [LONG_OFF, LONG_ON, LONG_OFF,
//          SHRT_ON, SHRT_OFF, SHRT_ON, SHRT_OFF
//           ]
// "123" = [LONG_OFF, LONG_ON, LONG_OFF,
//           SHRT_ON, SHRT_OFF,  
//           LONG_ON, SHRT_OFF,  SHRT_ON, SHRT_OFF, SHRT_ON, SHRT_OFF,
//           LONG_ON, SHRT_OFF,  SHRT_ON, SHRT_OFF, SHRT_ON, SHRT_OFF, SHRT_ON, SHRT_OFF
//           ]
//
// Implementation details:
// - largest (uint16_t) value: 65535
// - longest value to display (time-wise): 59999
//  - in terms of durations "durs"
//          SYNC    9
//             5    10 =     5*2
//             9    21 = 3 + 9*2
//             9    21 = 3 + 9*2
//             9    21 = 3 + 9*2
//             9    21 = 3 + 9*2
//      = 103 durs
// - with a 500 ms dur, this is over 50 seconds to display!
//
// Consider 8 bit value, longest duration would be "199"
//          SYNC    9
//             1    10 =     1*2
//             9    21 = 3 + 9*2
//             9    21 = 3 + 9*2
//      =  61 durs
//

// TODO: implement and enable PACKED to save mem.
//#define PACKED

class BlinkValue
{
public:
    BlinkValue(uint8_t pin, unsigned long base_duration_ms=500, uint8_t led_off=LOW, uint8_t led_on=HIGH)
        : pin_(pin)
        , base_dur_(base_duration_ms)
        , led_off_(led_off)
        , led_on_(led_on)
    {
        pinMode(pin_, OUTPUT);          // Set specified pin to be an OUTPUT
        digitalWrite(pin_, led_off);    // Initialize it off
        prev_toggle_millis_ = 0;
    }

#ifdef PACKED
    void build_pattern(uint16_t value)
    {
        // reset timing and indices
        prev_toggle_millis_ = 0;
        byte_index_ = 0;
        bit_index_ = 7; // look at MSB first
    }

    void advance_byte_index()
    {
        // todo
    }

    void advance_bit_index()
    {
        if( bit_index_ == 0 )
        {
            bit_index_ = 7;
            advance_byte_index();
        }
        else
            bit_index_ --;
    }

    int get_next_state()
    {
        int ret = (pattern_[byte_index_] >> bit_index_) & 1 == 1 ? led_on_ : led_off_;
        advance_bit_index();
        return ret;
    }
#else
    void pattern_add_long(uint8_t state, int &index)
    {
        pattern_[index] = state;
        index++;
        pattern_[index] = state;
        index++;
        pattern_[index] = state;
        index++;
    }

    void pattern_add_digit(int digit, int &index, bool leading)
    {
        if (!leading)
        {
            pattern_add_long(0, index);
        }
        for( int i=0; i<digit; i++ )
        {
            pattern_[index] = 1;
            index++;
            pattern_[index] = 0;
            index++;
        }
    }

    void build_pattern(uint16_t value)
    { 
        /* Reference python code:
        divisors = [10000, 1000, 100, 10]
        for divisor in divisors:
            digit = value // divisor
            i = self.pattern_add_digit(digit, i, leading)
            if digit > 0:
                leading = False
                value -= digit * divisor

        assert value < 10, "Value is: %d"%value
        assert value >= 0
        digit = value
        i = self.pattern_add_digit(digit, i, leading)
        */

        int i = 0;
        // SYNC
        pattern_add_long(0, i);
        pattern_add_long(1, i);
        pattern_add_long(0, i);

        // digits
        bool leading = true;

        static const int divisors[4] = {10000, 1000, 100, 10};
        for (auto divisor : divisors)
        {
            int digit = value / divisor;
            pattern_add_digit(digit, i, leading);
            if (digit > 0 )
            {
                leading = false;
                value -= digit * divisor;
            }
        }

        // assert value < 10, "Value is: %d"%value
        pattern_add_digit(value, i, leading);

        pattern_len_ = i;

        // reset timing and indices
        prev_toggle_millis_ = 0;
        next_index_ = 0;
    }

    void advance_index()
    {
        next_index_++;
        if( next_index_ >= pattern_len_ )
        {
            next_index_ = 0;
            if (completed_durations_ < 255)
                completed_durations_++;
        }
    }

    int get_next_state()
    {
        int ret = (pattern_[next_index_] == 0) ? led_off_ : led_on_;
        advance_index();
        return ret;
    }
#endif

    void set_value(uint16_t value)
    {
        value_ = value;
        build_pattern(value_);
        completed_durations_ = 0;
    }

    uint8_t get_completed_durations()
    {
        return completed_durations_;
    }

    // periodically call this
    void Loop()
    {
        unsigned long now = millis();
        unsigned long scaled_interval = base_dur_ >> ClkDiv::get_clock_shifts();
        if( (prev_toggle_millis_ == 0) || (now >= prev_toggle_millis_ + scaled_interval) )
        {
            int output_state = get_next_state();
            digitalWrite(pin_, output_state); //
            prev_toggle_millis_ = now;
        }
    }

private:
    uint8_t pin_;
    unsigned long base_dur_;
    uint8_t led_off_;
    uint8_t led_on_;

    unsigned long prev_toggle_millis_;
    uint16_t value_ = 0;
    uint8_t completed_durations_ = 0;

    static const int MAX_PATTERN_LEN = 104;
    static_assert( (MAX_PATTERN_LEN % 8) == 0, "Needs to be even multiple of 8" );
#ifdef PACKED
    // TODO: track pattern_index and then map that to byte_index and bit_offset
    static const int PACKED_PATTERN_LEN = MAX_PATTERN_LEN / 8;
    uint8_t pattern_[PACKED_PATTERN_LEN] = {0};
    int pattern_len_ = 0;
    int next_index_ = 0;
#else
    // simpler impl that uses 104 bytes
    uint8_t pattern_[MAX_PATTERN_LEN] = {0};
    int pattern_len_ = 0;
    int next_index_ = 0;
#endif

}; // class BlinkValue

//------------------------------------------------------------
// Circuit Pinout Definitions

/**
                ╔═══════════════════════╗      
                ║        ATtiny85       ║      
                ║                       ║      
            PB5 ║1 RESET          VCC  8║      
       A3 3 PB3 ║2 PB3*,ADC3      PB2  7║ PB2  2 A1
       A2 4 PB4 ║3 PB4*,ADC2      PB1* 6║ PB1* 1   
                ║4 GND            PB0* 5║ PB0* 0   
                ╚═══════════════════════╝      

    - PWM pins marked with (*)
        - only PB0 & PB1 and PB4 (as of update from ~2018) out of the box
        - to unlock additional PWM channels: http://www.technoblogy.com/show?LE0
    - apparently there is way to reprogram RESET pin for general I/O use?


  Analog Reads:
    pinMode(A_pin, INPUT); // PB_pins: A1, A2, A3
    int analogRead(A_pin); // returns [0, 1023]

  "Analog" Writes:
    pinMode(PB_pin,OUTPUT);    // PB_pins: PB0, PB1, PB4
    analogWrite(PB_pin, v);    // [0, 255], actually PWM output

  Digital Reads/Writes:
    pinMode(PB_pin,  INPUT);   // PB_pins: PB0, PB1, PB2, PB3, PB4
    int digitalRead(PB_pin, v);// LOW, HIGH
    pinMode(PB_pin, OUTPUT);   // PB_pins: PB0, PB1, PB2, PB3, PB4
    digitalWrite(PB_pin, v);   // LOW, HIGH

**/

// AnnoyATron-circuit PCB pinouts
// - NOTE: red LED is connected to PB2 which does not support PWM output
//  - i.e. analogWrite() won't work
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

// PWM-LED driver breadboard pinouts
// Features:
// - A3 analog input: read POT value to control PWM LED level
// - PB0 PWM output: to output transistor for LED drive
// - PB2 digital output: signal LED to blink out PWM level
/*
                ╔═════════════════════════╗      
          +5    ║ ◯       ATtiny85        ║ +5   
          ▲     ║                         ║ ▲      
          │     ║1 RESET            VCC  8║─┘       ↗↗ 
      10k,R◀────║2 PB3*,ADC3,A3  A1,PB2  7║──R,xxx──▶─┐                     
          │     ║3 PB4*,ADC2,A2     PB1* 6║           ▽ Gnd          
          ▽  ┌──║4 GND              PB0* 5║             
             ▽  ╚═════════════════════════╝      

                                               +5
                                               ▲ 
                                               │ 
                                               R,10
                                               │ 
                                               ▽↘
                                               │ 
                                            B / C
                            PB0  ───R,1.5k──-|   
                                              \ E
                                               │
                                               ▽ Gnd
*/
namespace PwmLedDriver
{
    // Pin definitions
    static const int PWMLED = PB0;  // PWM LED output
    static const int POT = A3;      // POTentiometer -- use to control PWM level
    static const int SIGLED = PB2;  // signalling LED output

    // chip outputs to Base of NPN Q
    // - high output forward biases Base-Emitter, turning on the LED
    static const int LED_ON  = HIGH;
    static const int LED_OFF = LOW;
}


//------------------------------------------------------------
// Test Classes


// PWM LED (w/ BlinkValue) test
// - sampling driven by the signalling LED, when it completes it's duration
//  - read POT level
//  - set PWM level
//  - set signalling LED
struct Test_PwmLed
{
#if 1
    static const uint8_t sig_led = PwmLedDriver::SIGLED;
    static const uint8_t led_off = PwmLedDriver::LED_OFF;
    static const uint8_t led_on  = PwmLedDriver::LED_ON;
    static const int     pwm_led = PwmLedDriver::PWMLED;
    static const int     pot     = PwmLedDriver::POT;
#else
    static const uint8_t sig_led = AnnoyATron::LED_RED;
    static const uint8_t led_off = AnnoyATron::LED_OFF;
    static const uint8_t led_on  = AnnoyATron::LED_ON;
    static const int     pwm_led = AnnoyATron::LED_GRN;
    static const int     pot     = AnnoyATron::POT;
#endif

    static BlinkValue *bv;

    static void setup()
    {
        // set clock divisor (prescaler) -- otherwise it probably gets set to 8
        ClkDiv::set(1);

        static BlinkValue _bv(sig_led, 500, led_off, led_on);
        bv = &_bv;
        pinMode(pwm_led, OUTPUT);
    }

    static void loop() 
    {
//      bv->Loop();
//      if (bv->get_completed_durations() > 0)
        {
            int value = analogRead(pot); // returns [0, 1023]
            value >>= 2;    // reduce to 8 bit value
//          bv->set_value(value);
            analogWrite(pwm_led, value);
        }
        ClkDiv::ms_delay(200);
    }
};
BlinkValue *Test_PwmLed::bv;

// Test ClkDiv
struct Test_ClkDiv
{
    static const int     led_red = AnnoyATron::LED_RED;
    static const int     led_grn = AnnoyATron::LED_GRN;
    static const uint8_t led_off = AnnoyATron::LED_OFF;
    static const uint8_t led_on  = AnnoyATron::LED_ON;

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

        pinMode(led_grn, OUTPUT);
        pinMode(led_red, OUTPUT);
    }

    static void loop() 
    {
        digitalWrite(led_grn, led_on);
        digitalWrite(led_red, led_off);
        ClkDiv::ms_delay(2000);
        digitalWrite(led_grn, led_off);
        digitalWrite(led_red, led_on);
        ClkDiv::ms_delay(3000);
    }
};

// BlinkerL test (AnnoyATron)
// - blink RED & GRN LEDs at different rates via BlinkerL
// - slow blink RED (2000 ms)
// - fast blink GRN ( 250 ms)
//  - expect only be visible during RED=OFF
struct Test_BlinkerL
{
    static const int     led_red = AnnoyATron::LED_RED;
    static const int     led_grn = AnnoyATron::LED_GRN;

    static BlinkerL *red;
    static BlinkerL *grn;

    static void setup()
    {
        // set clock divisor (prescaler) -- otherwise it probably gets set to 8
        ClkDiv::set(1);

        // some goofiness 
        // - would have preferred using placement-new
        // - but seems placement-new isn't supported by Arduino compiler?
        static BlinkerL _red(led_red, 1000);
        static BlinkerL _grn(led_grn,  125);
        red = &_red;
        grn = &_grn;
    }

    static void loop() 
    {
        red->Loop();
        grn->Loop();
    }
};
BlinkerL *Test_BlinkerL::red;
BlinkerL *Test_BlinkerL::grn;


// BlinkValue test
// - on reset reads POT value (scaled to 8 bit value)
// - blink value via RED LED
struct Test_BlinkValue
{
    static const uint8_t sig_led = AnnoyATron::LED_RED;
    static const uint8_t led_off = AnnoyATron::LED_OFF;
    static const uint8_t led_on  = AnnoyATron::LED_ON;
    static const int     pot     = AnnoyATron::POT;

    static BlinkValue *bv;

    static void setup()
    {
        // set clock divisor (prescaler) -- otherwise it probably gets set to 8
        ClkDiv::set(1);

        static BlinkValue _bv(sig_led, 500, led_off, led_on);
        bv = &_bv;


        int value = analogRead(pot); // returns [0, 1023]
        value >>= 2;    // reduce to 8 bit value
        _bv.set_value(value);
    }

    static void loop() 
    {
        bv->Loop();
    }
};
BlinkValue *Test_BlinkValue::bv;


// AnnoyATron Test analogWrite and LED brightness
// - recall LEDs are common anode, so low output pin = full brightness
// - for the red LED -- no real gradation noted and then its OFF for values >= 128
//  - this is because red LED is connected to PB2 which doesn't support PWM output!
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

    static void loop() 
    {
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
// Uncomment desired test

//Test_ClkDiv test;
//Test_analogWrite test;
//Test_BlinkerL test;
//Test_BlinkValue test;
Test_PwmLed test;

// put your setup code here, to run once:
void setup() 
{
    test.setup();
}


// put your main code here, to run repeatedly:
void loop() 
{
    test.loop();
}

