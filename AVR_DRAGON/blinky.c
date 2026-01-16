#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    // Set PB1 as output
    DDRB |= (1 << DDB1);

    while (1) {
		
        // LED ON
        PORTB |= (1 << PORTB1);
#ifndef DEBUG
        _delay_ms(2000);
#endif
        // LED OFF
        PORTB &= ~(1 << PORTB1);
#ifndef DEBUG
        _delay_ms(2000);
#endif
    }
}
