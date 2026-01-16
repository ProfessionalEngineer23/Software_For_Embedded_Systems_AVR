///////////////////////////// CA1 /////////////////////////////
// Purpose: Count button presses using INT0 and print via USART
// Student Name: Niki Mardari
// ID: B00159642
///////////////////////////////////////////////////////////////

///////////////////////////// MACROS /////////////////////////////
#define F_CPU 1000000UL

///////////////////////////// Libraries /////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include "usart.h"

///////////////////////////// Global Variables /////////////////////////////
// ISR-shared state (volatile)
volatile uint32_t press_count = 0;   // Increments when INT0 sees falling edge
volatile uint8_t changed = 0;        // Flag to tell main loop to print

///////////////////////////// ISRs /////////////////////////////
// INT0 ISR: fires on falling edge at PD2 (button press to GND)
ISR(INT0_vect)
{
    press_count++;
    changed = 1;
}

///////////////////////////// Functions /////////////////////////////
// Configure PD2 with internal pull-up and INT0 on falling edge
void int0_init(void)
{
    DDRD  &= ~(1 << PD2);        // PD2 input
    PORTD |=  (1 << PD2);        // enable pull-up (button to GND)

    EICRA  =  (1 << ISC01);      // ISC01=1, ISC00=0 for falling edge
	EICRA  &= ~(1 << ISC00);
    EIFR   =  (1 << INTF0);      // clear any pending INT0 flag
    EIMSK  =  (1 << INT0);       // enable INT0
}

///////////////////////////// Main /////////////////////////////
int main(void)
{
    usartInit();     // 9600 bps at 1 MHz is set in usart.c
    int0_init();
    sei();           // enable all global interrupts

    usartSendString("Press button to start counting\r\n");

    char buf[40];
    while(1)
    {
        if (changed)
        {
            changed = 0;
            // Use sprintf to format the count
            sprintf(buf, "Count: %lu\r\n", (unsigned long)press_count);
            usartSendString(buf);
        }
    }
}
