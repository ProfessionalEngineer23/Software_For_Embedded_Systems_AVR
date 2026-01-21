/*
 * ATmega328P - Tilt interrupt + passive buzzer tone + alternating LEDs
 *
 * PD2 (INT0)  : tilt input, internal pull-up enabled (switch -> GND on tilt)
 * PD3 (OC2B)  : passive buzzer output (Timer2 Fast PWM ~2 kHz)
 * PD7         : red LED output (ON = HIGH)
 * PB1         : green LED output (ON = HIGH)
 *
 * On each tilt (falling edge on PD2), exactly one LED turns ON (alternating),
 * and the passive buzzer beeps for ~200 ms using Timer2 PWM on OC2B.
 * A simple debounce masks INT0 briefly, then re-enables it.
 */

// #define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

/* -------------------- Globals set inside ISR/main loop -------------------- */
volatile uint8_t tilt_event = 0;  // Set by INT0 ISR when a tilt is detected
volatile uint8_t next_is_red = 1; // Toggles which LED to light next (1=red)

/* ---------------------------- LED helper funcs ---------------------------- */
static inline void leds_off(void) {
    // Turn both LEDs OFF (assumes active-high wiring)
    PORTD &= ~(1 << PORTD7);  // PD7 low  -> red OFF
    PORTB &= ~(1 << PORTB1);  // PB1 low  -> green OFF
}

static inline void set_red(void) {
    leds_off();
    PORTD |= (1 << PORTD7);   // PD7 high -> red ON
}

static inline void set_green(void) {
    leds_off();
    PORTB |= (1 << PORTB1);   // PB1 high -> green ON
}

/////////////////////////// Passive buzzer tone using Timer2 PWM ///////////////////////////
 * We generate a ~2 kHz square wave on OC2B (PD3) using Fast PWM with TOP=0xFF.
 * Frequency ≈ F_CPU / (N * 256). With F_CPU=16 MHz and prescaler N=32:
 * f ≈ 16e6 / (32 * 256) ≈ 1953 Hz (audible and fine for most piezos).
 *
 * OC2B (PD3) is driven in non-inverting mode at ~50% duty (OCR2B=128).
 * While tone is ON, Timer2 controls PD3; when OFF, we stop Timer2 and drive PD3 low.
 */
static inline void tone2_start_2khz(void) {
    // Ensure PD3 is an output (also done in io_init, but harmless here)
    DDRD  |= (1 << DDD3);

    // COM2B1:0 = 10 (non-inverting on OC2B)
    // WGM22:0  = 0b011 (Fast PWM, TOP=0xFF)
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);

    // Prescaler = /32  -> CS22:CS21:CS20 = 0:1:1
    TCCR2B = (1 << CS21) | (1 << CS20);

    // 50% duty for a square wave
    OCR2B  = 128;
}

static inline void tone2_stop(void) {
    // Stop Timer2 and disconnect OC2B
    TCCR2B = 0;
    TCCR2A = 0;
    OCR2B  = 0;

    // Making sure buzzer pin is low when tone is off
    PORTD &= ~(1 << PORTD3);
}

/* /////////////////////////// External INT0 (PD2) ///////////////////////////
 * Trigger on falling edge: with internal pull-up enabled, a tilt closing to GND
 * produces a high->low transition -> ISR fires once per tilt.
 *
 * Debounce approach:
 *  - In ISR: set flag and MASK INT0 (EIMSK.INT0=0) to ignore bounces.
 *  - In main loop: after handling event, wait ~50 ms, clear EIFR.INTF0,
 *    then UNMASK INT0 to allow the next event.
 */
ISR(INT0_vect) {
    // Optional: sanity check that PD2 really reads low
    if ((PIND & (1 << PIND2)) == 0) {
        tilt_event = 1;
        EIMSK &= ~(1 << INT0);  // Mask INT0 until we debounce in main
    }
}

/* /////////////////////////// One-time initialization /////////////////////////// */
static void io_init(void) {
    /* --- Tilt input PD2 with internal pull-up --- */
    DDRD  &= ~(1 << DDD2);     // PD2 as input
    PORTD |=  (1 << PORTD2);   // enable pull-up (PD2 idles HIGH)

    /* --- Buzzer PD3 and red LED PD7 as outputs --- */
    DDRD  |= (1 << DDD3) | (1 << DDD7);

    /* --- Green LED PB1 as output --- */
    DDRB  |= (1 << DDB1);

    /* --- Start with everything OFF --- */
    leds_off();
    PORTD &= ~(1 << PORTD3);   // buzzer pin low
	
	DDRD  |=  (1<<DDD1);   // PD1 (TXD) as output
    DDRD  &= ~(1<<DDD0);   // PD0 (RXD) as input
    PORTD |=  (1<<PORTD0); // pull-up on RXD (ok if adapter drives it)

    /* Note: Keep MCUCR.PUD = 0 (default) so pull-ups are not globally disabled. */
}

static void extint_init(void) {
    /* INT0 sense control: falling edge
     * EICRA:
     *   ISC01:ISC00 = 1:0  -> falling edge on INT0 (PD2)
     */
    EICRA = (1 << ISC01);

    /* Clear any pending external interrupt flag on INT0 by writing a '1' */
    EIFR  = (1 << INTF0);

    /* Enable INT0 in the External Interrupt Mask Register */
    EIMSK = (1 << INT0);

    /* Enable global interrupts */
    sei();
}

/////////////////////////// main /////////////////////////// 
int main(void) {
    io_init();
    extint_init();

    usartInit();
    // Optional: enable RX interrupt if you plan to use it
    // usartEnableRxInt();
    // sei();  // only if you use USART RX interrupts

    for (;;) {
		usartSendString("Hello, World!\r\n");
        if (tilt_event) {
            if (next_is_red) { set_red();   next_is_red = 0; }
            else             { set_green(); next_is_red = 1; }

            tone2_start_2khz();
            _delay_ms(100);
            tone2_stop();

            _delay_ms(50);
            EIFR  = (1 << INTF0);
            EIMSK |= (1 << INT0);
            tilt_event = 0;
        }
	
        /* Example: echo a received byte (polling) */
        if (usartCharReceived()) {
            char c = usartReadChar();
            usartSendChar(c);            // echo
        }
    }
}
/*
To Compile: 
avr-gcc -mmcu=atmega328p -DF_CPU=1000000UL -Os Tilty.c usart1.c -o Tilty.elf
avr-objcopy -O ihex -R .eeprom Tilty.elf Tilty.hex

To upload: avrdude -c stk500 -p m328p -P /dev/ttyACM0 -U flash:w:Tilty.hex:i

// To view serial terminal:
gtkterm -p /dev/ttyACM1 -s 9600

*/
