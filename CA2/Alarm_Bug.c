// Name: Niki Mardari
// Purpose: Show a false alarm caused by a shared-data race between the INT0 ISR and the main loop.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

static volatile int iTemperatures[2] = {0, 0};

ISR(INT0_vect)
{
    iTemperatures[0]++;
    iTemperatures[1]++;
}

static inline void AlarmOn(void)  { PORTB |=  (1 << PB1); } // Turn LED On
static inline void AlarmOff(void) { PORTB &= ~(1 << PB1); } // Turn LED Off

static void pins_setup(void)
{
    DDRB  |=  (1 << DDB1);   // Set PB1 as output
    AlarmOff();

    DDRD  &= ~(1 << DDD2);   // PD2 as input
    PORTD |=  (1 << PORTD2); // enable pull-up (PD2 idles HIGH)

    // Trigger INT0 on FALLING edge (ISC01:ISC00 = 10b)
    EICRA &= ~((1 << ISC00) | (1 << ISC01)); // clear registers
    //EICRA |=  (1 << ISC01);
    EIMSK |=  (1 << INT0); // Enable INT0
}

int main(void)
{
    pins_setup();
    sei();
	
    while (1)
    {
        volatile int iTemp0, iTemp1;
		
		// cli();
		//ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		//{
        iTemp0 = iTemperatures[0];
		#ifndef DEBUG
		// Making the interrupt window longer to increase chance of tear
        _delay_ms(500); 
		#endif
		
        iTemp1 = iTemperatures[1];
		// sei();
		//}
		
        if (iTemp0 != iTemp1)
        {
            AlarmOn();
			#ifndef DEBUG
            _delay_ms(100); //Blink LED
			#endif
            AlarmOff();
        }
    }
}

// To compile: avr-gcc -mmcu=atmega328p -DF_CPU=1000000UL -Os Alarm_Bug.c -o Alarm_Bug.elf
// avr-objcopy -O ihex -R .eeprom Alarm_Bug.elf Alarm_Bug.hex

// To upload: avrdude -c stk500 -p m328p -P /dev/ttyACM0 -U flash:w:Alarm_Bug.hex:i

// Terminal: gtkterm -p /dev/ttyACM1 -s 9600
