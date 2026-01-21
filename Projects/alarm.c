// Name: Niki Mardari
// Purpose: Simple alarm system with buzzer and LEDs, unlockable via USART

// main.c — ATmega328P @ 1 MHz, USART 9600, buzzer + alternating LEDs
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

// Pins:
// PD7 -> Red LED (active-high)
// PB1 -> Green LED (active-high)
// PD3 -> Buzzer on OC2B (Timer2 PWM)

static inline void io_init(void) {
    // UART pins
    DDRD  |=  (1 << DDD1);   // PD1 TXD out
    DDRD  &= ~(1 << DDD0);   // PD0 RXD in
    PORTD |=  (1 << PORTD0); // pull-up on RXD

    // LEDs
    DDRD  |=  (1 << DDD7);   // PD7 output
    DDRB  |=  (1 << DDB1);   // PB1 output

    // Buzzer default low
    DDRD  |=  (1 << DDD3);
    PORTD &= ~(1 << PORTD3);
}

static inline void leds_off(void) {
    PORTD &= ~(1 << PORTD7);
    PORTB &= ~(1 << PORTB1);
}

static inline void leds_both_on(void) {
    PORTD |=  (1 << PORTD7);
    PORTB |=  (1 << PORTB1);
}

// simple helpers to set one LED on, the other off
static inline void red_on_green_off(void) {
    PORTD |=  (1 << PORTD7);   // red ON
    PORTB &= ~(1 << PORTB1);   // green OFF
}
static inline void green_on_red_off(void) {
    PORTD &= ~(1 << PORTD7);   // red OFF
    PORTB |=  (1 << PORTB1);   // green ON
}

/* Buzzer 2 kHz using Timer2 Fast PWM (TOP = OCR2A)
   f = F_CPU / (N * (OCR2A+1))
   For F_CPU=1 MHz, use N=8, OCR2A=62 → ~1984 Hz (~2 kHz) */
static inline void tone_start_2khz(void) {
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, OC2B non-inverting
    TCCR2B = (1 << WGM22)  | (1 << CS21);                 // prescaler /8
    OCR2A  = 62;
    OCR2B  = (OCR2A + 1) / 2;  // 50% duty cycle
}
static inline void tone_stop(void) {
    TCCR2B = 0;
    TCCR2A = 0;
    OCR2B  = 0;
    PORTD &= ~(1 << PORTD3);
}

///////////////////////////////////// main /////////////////////////////////////
int main(void) {
    io_init();
    usartInit();

    static char unlocked_msg[] = "\r\nUnlocked!\r\n";
    static char locked_msg[] = "\r\nAlarm!\r\n";

    for (;;) {
        usartSendString(locked_msg);
        // If ANY char is received then stop buzzer and turn both LEDs on forever
        if (usartCharReceived()) {
            (void)usartReadChar();  // read & discard (any char unlocks)
            tone_stop();
            leds_both_on();
            usartSendString(unlocked_msg);
            for(;;) { /* stay unlocked */ }
        }

        // Cycle 1: RED on, GREEN off while buzzing
        red_on_green_off();
        tone_start_2khz();
        _delay_ms(250);
        tone_stop();
        leds_off();
        _delay_ms(250);

        // Cycle 2: GREEN on, RED off while buzzing
        green_on_red_off();
        tone_start_2khz();
        _delay_ms(250);
        tone_stop();
        leds_off();
        _delay_ms(250);
    }
}

/* To compile:
avr-gcc -mmcu=atmega328p -DF_CPU=1000000UL -Os alarm.c usart.c -o alarm.elf
avr-objcopy -O ihex -R .eeprom alarm.elf alarm.hex
*/

// To upload: avrdude -c stk500 -p m328p -P /dev/ttyACM0 -U flash:w:alarm.hex:i

// To access serial terminal:  gtkterm -p /dev/ttyACM1 -s 9600