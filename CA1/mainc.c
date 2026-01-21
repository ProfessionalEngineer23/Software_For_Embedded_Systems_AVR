///////////////////////////// CA1 /////////////////////////////
// Purpose: mainc.c code to read frequency usinng timer1 and interrupts
// Student Name: Niki Mardari

///////////////////////////// MACROS /////////////////////////////

#define F_CPU 1000000UL

///////////////////////////// Libraries /////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include "usart.h"

///////////////////////////// Global Variables /////////////////////////////

//ISR-shared state (volatile)
volatile uint16_t edge_count = 0;    // For countinng edges within 1 second window
volatile uint16_t pulseCount = 0;    // To store snapshot for main loop

///////////////////////////// Functions /////////////////////////////

void hardwSetup()
{
    // External Interrupt 0 on rising edge
    EICRA = (1 << ISC01) | (1 << ISC00);  // rising edge triggers INT0
    EIMSK = (1 << INT0);                  // enable INT0 interrupt

    // Timer1: Set for 1 second intervals using CTC mode
	// 1Mhz too big for 16 bit timer!
	//
	// Calculations:
    // F_CPU = 1 MHz, prescaler = 64 
    // 1000000/64 = 15625 Hz
	// Timer Resolution = 1/F_CPU 
	// Target Timer Frequency = (F_CPU / (prescaler * target time)) - 1
	// ((1000000)/(64*1)) = 15624
	// Theore, 
	// Nquist Theorem 
	
	// Enabling CTC (Clear Timer on Compare) and prescalar of 64
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  
    OCR1A  = 15624;
	// Enable Timer1 Compare A interrupt so it triggers 2nd ISR on interrupt
    TIMSK1 = (1 << OCIE1A);  
}

// 1st ISR
// INT0 ISR for counting rising edges on PD2/INT0
ISR(INT0_vect)
{
 edge_count++;
}

// 2nd ISR
// Timer1 Compare Match A ISR. INterupts every 1 seconds
ISR(TIMER1_COMPA_vect)
{
    // Copy and clear the counter atomically
    pulseCount = edge_count;
    edge_count = 0;
}

///////////////////////////// Main code!! /////////////////////////////

int main(void)
{
	hardwSetup();
	
	// PD2 (INT0) as input with pull-up
    DDRD  &= ~(1 << DDD2);		// D2 as input
	PORTD &= ~(1 << PORTD2);    // Pull up off for Func Genereator 
	
    // USART 9600 8N1 @ 1 MHz
    usartInit();
	
	// enable global interrupts
    sei();  

    char buffer[32];
    uint16_t lastCount = 0;

    while (1)
    {
//		if(lastCount != pulseCount)
//		{
			lastCount = pulseCount;

            // Store lastCount value with string formating in buffer
            sprintf(buffer, "Freq = %u Hz\r\n", lastCount);
			// Send string in buffer
            usartSendString(buffer);
//		}
	}
    return 0;
}

// avr-gcc -mmcu=atmega328p -DF_CPU=1000000UL -Os mainc.c usart.c -o mainc.elf
// avr-objcopy -O ihex -R .eeprom mainc.elf mainc.hex

// To upload: avrdude -c stk500 -p m328p -P /dev/ttyACM0 -U flash:w:mainc.hex:i
// To access serial terminal:  gtkterm -p /dev/ttyACM1 -s 9600
