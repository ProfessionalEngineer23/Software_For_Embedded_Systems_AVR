// Measure frequency using Timer1 external clock on T1 (PD5).
// Gate time = 1 s (made from 4 × 250 ms Timer0 ticks).

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "usart.h"

static volatile uint16_t pulseCount = 0;   // 1 s snapshot
static volatile uint8_t  sampleReady = 0;  // flag for main

ISR(TIMER0_COMPA_vect)
{
    static uint8_t quarters = 0;           // count 250 ms ticks
    if (++quarters >= 4) {                 // ~1 second has passed
        quarters = 0;

        // Snapshot and clear Timer1 (external pulse counter)
        uint16_t snap = TCNT1;
        TCNT1 = 0;

        pulseCount = snap;                 // counts per 1 s == Hz
        sampleReady = 1;
    }
}

static void hardwareSetup(void)
{
    // --- T1 pin (PD5) as input, no pull-up ---
    DDRD  &= ~(1 << DDD5);
    PORTD &= ~(1 << PORTD5);

    // --- Timer1: external clock source on T1 (PD5), rising edge ---
    // CS12:CS11:CS10 = 111 -> External clock on T1, rising edge
    TCCR1A = 0;
    TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
    TCNT1  = 0;

    // --- Timer0: CTC ~250 ms tick ---
    // f_timer0 = F_CPU/1024 = 1_000_000/1024 ≈ 976.56 Hz
    // 0.25 s ≈ 244 ticks -> OCR0A = 243
    TCCR0A = (1 << WGM01);                 // CTC
    TCCR0B = (1 << CS02) | (1 << CS00);    // prescaler 1024
    OCR0A  = 243;                          // ~249.6 ms
    TIMSK0 = (1 << OCIE0A);                // enable compare A ISR
}

static void print_u16(uint16_t v) {
    char b[6]; uint8_t i=0;
    if (!v) { usartSendChar('0'); return; }
    while (v) { b[i++]='0'+(v%10); v/=10; }
    while (i--) usartSendChar(b[i]);
}

int main(void)
{
    usartInit();               // 9600 8N1 @ 1 MHz (U2X)
    hardwareSetup();
    sei();

    usartSendString("\r\nT1 external clock on PD5, 1s gate (4x250ms)\r\n");

    while(1) 
	{
        if (sampleReady) {
            sampleReady = 0;   // one print per second
            usartSendString("Freq = ");
            print_u16(pulseCount);         // 1 count = 1 Hz
            usartSendString(" Hz\r\n");
        }
    }
}
