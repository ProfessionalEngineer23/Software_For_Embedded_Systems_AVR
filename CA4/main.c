/**
 * @file main.c
 * 
 * @author Tiago Lobao
 * 
 * @brief Blink example based on the example for atmega328p
 * https://github.com/feilipu/avrfreertos/blob/master/UnoBlink/main.c
 *
 * Modified BT 05/12/24
 * Modified NM 04/12/25
 */

// 1 system tick is 15 ms (apparently)

// vTaskDelay(250/portTICK_PERIOD_MS);  
// Would be: 250/15 = 16 ticks 
// So 16 * 15 = 240ms delay

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <usart1.h>
#include "semphr.h"
#include <avr/interrupt.h>

// Global Semaphores
SemaphoreHandle_t xSemaphore0; // Semaphore for Task A
SemaphoreHandle_t xSemaphore1; // Semaphore for Task B
SemaphoreHandle_t xSemaphore2; // Semaphore for UsartSendString

// Tasks declarations
/*-----------------------------------------------------------*/
static void TempUpdate(void *pvParameters);
static void AlarmTask(void *pvParameters);
/*-----------------------------------------------------------*/

// Array to hold global temperature values
static volatile int iTemperatures[2] = {0, 0};

// Interrupt that wakes task 1 to execute 
ISR(INT0_vect)
{

	// This is where the smeaphore are given to task1
	xSemaphoreGive(xSemaphore0);
	PORTD ^=  (1 << PD5);

}

// Setting up pins for LEDs, pull up resistor for button and setting interrupt to trigger on falling edge 
static void pins_setup(void)
{
    DDRB  |=  (1 << DDB1);   // Set PB1 as output
	DDRB  |=  (1 << DDB5);	 // Set PB5 as output
 	DDRB  |=  (1 << DDB3);   // Set PB3 as output
	DDRD  |=  (1 << DDD5);   // Set PD5 as output

    DDRD  &= ~(1 << DDD2);   // PD2 as input
    PORTD |=  (1 << PORTD2); // enable pull-up (PD2 idles HIGH)
    
    // Trigger INT0 on FALLING edge (ISC01:ISC00 = 10b)
    EICRA &= ~((1 << ISC00) | (1 << ISC01)); // clear registers
    EICRA |=  (1 << ISC01);
    EIMSK |=  (1 << INT0); // Enable INT0
}

///////////////////////////// Main Function /////////////////////////////
int main(void)
{
	// Initializing the semaphores 
	// Semaphore 0
	xSemaphore0 = xSemaphoreCreateBinary();
	// No give for Semaphore 0 because ISR controls the giving
	// Semaphore 1
	xSemaphore1 = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore1); // Giving make the semaphore available
	// Semaphore 2
	xSemaphore2 = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore2); // Giving make the semaphore available
	
	usartInit();
	pins_setup();
	sei();
	
    	xTaskCreate( 
		TempUpdate
		,  (const char *)"Update"
		,  256						// Stack Size
		,  NULL						// Task parameter 
		,  4 						// Priority, RTOS runs highest priority first 
		,  NULL );					// Task Handle
	
		xTaskCreate(
		AlarmTask
		,  (const char *)"Compare"
		,  256
		,  NULL
		,  3
		,  NULL );

	vTaskStartScheduler();    //This never returns... control handed to the RTOS
}

/*-----------------------------------------------------------*/
///////////////////////////// Task Definitions /////////////////////////////

// Task1
static void TempUpdate(void *pvParameters)
{
	// Infinite for loop because we want this loop running forever 
	// Without this the task will run once and end
	// The RTOS keeps the tasks running and switches between the two tasks
	
	for(;;)
	{
		// Take semaphore for incrementing temps
		xSemaphoreTake(xSemaphore0, portMAX_DELAY);
		// Take semahore for printing to the usart 
		xSemaphoreTake(xSemaphore1, portMAX_DELAY);
		iTemperatures[0]++;
    	iTemperatures[1]++;
		// Toggle LED to show when the ISR runs 
		PORTB ^=  (1 << PB1);
		xSemaphoreGive(xSemaphore1);
	
		char buffer[50];
		sprintf(buffer, "\nTemp1: %d\nTemp2: %d\n", iTemperatures[0], iTemperatures[1]);
	
		xSemaphoreTake(xSemaphore2, portMAX_DELAY);
		usartSendString(buffer);
		xSemaphoreGive(xSemaphore2);
	
		vTaskDelay(250/portTICK_PERIOD_MS);
	}
}
/*---------------------------------------------------------------------------*/

// Task2
static void AlarmTask(void *pvParameters)
{
	for(;;)
	{
	// Local variables that we copy into
		int iTemp0, iTemp1;

		// Take semaphore for copying into the local variables
		
		//xSemaphoreTake(xSemaphore1, portMAX_DELAY);
    	iTemp0 = iTemperatures[0];
		vTaskDelay(2000/portTICK_PERIOD_MS);	// This delay was used for testing the race condition
    	iTemp1 = iTemperatures[1];
		PORTB ^= (1<<5); // PB5 LED toggle to see that the variables have been copied and compared 
		//xSemaphoreGive(xSemaphore1);

	char buffer[50];
	
    if(iTemp0 != iTemp1)
    {		
		sprintf(buffer, "\nAlarm!\n");

		xSemaphoreTake(xSemaphore2, portMAX_DELAY);
		usartSendString(buffer);
		xSemaphoreGive(xSemaphore2);

		// Alarm On
    	 PORTB |=  (1 << PB3); // Turn on LED 
    	vTaskDelay(250/portTICK_PERIOD_MS);
	}
	
	else
	{
		// Alarm Off
		PORTB &= ~(1 << PB3); // Turn off LED 
		sprintf(buffer, "\nNo Alarm\n");
		xSemaphoreTake(xSemaphore2, portMAX_DELAY);
		usartSendString(buffer);
		xSemaphoreGive(xSemaphore2);
	}
	
	vTaskDelay(250/portTICK_PERIOD_MS); // Without this delay the usart would be spammed and the checking would happen very quickly hogging the core without giving a chance to change from a different task
	}
}

// Use make to compile 
