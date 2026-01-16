// main.c — ATmega328P @ 1 MHz, USART 9600, buzzer + alternating LEDs
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "usart1.h"

static volatile int iTemperature[2];

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
	DDRD  |= (1 << DDB2); ////OPTIONAL PD2 as output

    // Buzzer default low
    DDRD  |=  (1 << DDD3);
    PORTD &= ~(1 << PORTD3);
	
	DDRD &= ~(1<<DDD2);
	PORTD |= (1 << PORTD2);    // Pull up ON
	EICRA = (1 << ISC01) | (1 << ISC00);  // rising edge triggers INT0
	EIFR  |= (1 << INTF0);                // clear any pending flag
    EIMSK = (1 << INT0);                  // enable INT0 interrupt
	
	 TCCR1B = (1 << CS10); // Timer1 Free running
	
}

static inline void leds_off(void) {
    PORTD &= ~(1 << PORTD7);
    PORTB &= ~(1 << PORTB1);
}

static inline void leds_both_on(void) {
    PORTD |=  (1 << PORTD7);
    PORTB |=  (1 << PORTB1);
}

static inline void red_on_green_off(void) {
    PORTD |=  (1 << PORTD7);   // red ON
    PORTB &= ~(1 << PORTB1);   // green OFF
}
static inline void green_on_red_off(void) {
    PORTD &= ~(1 << PORTD7);   // red OFF
    PORTB |=  (1 << PORTB1);   // green ON
}

/* Buzzer ~2 kHz using Timer2 Fast PWM (TOP = OCR2A)
   f = F_CPU / (N * (OCR2A+1))
   For F_CPU=1 MHz, use N=8, OCR2A=62 → ~1984 Hz (~2 kHz) */
static inline void tone_start_2khz(void) {
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, OC2B non-inverting
    TCCR2B = (1 << WGM22)  | (1 << CS21);                 // prescaler /8
    OCR2A  = 62;
    OCR2B  = (OCR2A + 1) / 2;  // ~50% duty
}
static inline void tone_stop(void) {
    TCCR2B = 0;
    TCCR2A = 0;
    OCR2B  = 0;
    PORTD &= ~(1 << PORTD3);
}

static inline void sos_signal(void)
{
	for(int i = 0; i < 3; i++)
	{
	tone_start_2khz();
	_delay_ms(200);
	tone_stop();
	_delay_ms(300);
	}
}

ISR(INT0_vect)
{
	uint16_t t = TCNT1;
	iTemperature[0] = t++;      // 1..2
    iTemperature[1] = (5-(t));

	/*
	uint16_t t = TCNT1;                 // grab timer1 counter
    iTemperature[0] = (t & 1) + 1;      // 1..2
    iTemperature[1] = ((t >> 1) & 1) + 1;
	*/
}

void main(void){
	
	io_init();

	sei();
	
	while(1)
	{

	leds_both_on();
	int iTemp0, iTemp1;
		
	cli();
	iTemp0 = iTemperature[0];
	iTemp1 = iTemperature[1];
	//_delay_ms(1000);
	sei();
		
	if (iTemp0 == iTemp1){tone_start_2khz();}
	else{tone_stop();}		
		/*
		int volatile x = PIND & (1<<DD2);
		if(x){sos_signal();}
		else{tone_stop();}
		*/
		
	}
}








/* To compile:
avr-gcc -mmcu=atmega328p -DF_CPU=1000000UL -Os NuclearReactorCode.c usart1.c -o NuclearReactorCode.elf
avr-objcopy -O ihex -R .eeprom NuclearReactorCode.elf NuclearReactorCode.hex
*/

// To upload: avrdude -c stk500 -p m328p -P /dev/ttyACM0 -U flash:w:NuclearReactorCode.hex:i

// To access serial terminal:  gtkterm -p /dev/ttyACM1 -s 9600