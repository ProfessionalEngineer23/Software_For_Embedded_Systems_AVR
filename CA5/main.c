/**
 * Queue Challenge for the Atmega328p FreeRTOS
 *
 * Task 1 (CLI):
 *   Should Echo characters typed on USART
 *   When it sees "delay <number>" and Enter, it sends <number> to Task 2 via a queue
 *   If it receives a Message from Task 2, it prints it
 *
 * Task 2 (Blink):
 *   Should blink LED on PORTB1 with current delay
 *   When it receives a new delay value, it updates blink rate and notifies Task 1
 *   Every 100 blinks, it sends a message to Task 1
 *
 * Created by: Niki Mardari 02/12/2025
*/

#include "FreeRTOS.h"
#include "task.h"
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "semphr.h"
#include "usart1.h"

// Constant variables defined 
static const uint8_t buf_len = 12;       // max size of one typed line
static const char command[] = "delay ";  // note the space. This is the keyword used to parse the string
static const uint8_t blink_max = 100;    // message every 100 blinks
static const int delay_queue_len = 1;    // 1 so that can overwrite old value
static const int msg_queue_len = 5;	     // Size of Message Queue

// Message struct for task 2 to send back to task 1
typedef struct Message {
  char body[20];			// For "Blinked" text
  uint16_t count;			// Count value for the new delay
} Message;

// Globals (queues)
static QueueHandle_t delay_queue;
static QueueHandle_t msg_queue;

// Task prototypes
static void doCLI(void *parameters);
static void blinkLED(void *parameters);

////////////////////////////// main function //////////////////////////////
int main(void)
{
  usartInit();
  usartSendString("Enter: delay <ms>\r\n");

  // Queues
  delay_queue = xQueueCreate(delay_queue_len, sizeof(int)); // Queue for the delay messages
  msg_queue = xQueueCreate(msg_queue_len, sizeof(Message)); // Queue for the Blinked 100 message

  // Tasks:
  // Task 1: Reads UART messages aka delay 100...
  xTaskCreate(
  		  doCLI, // Function to be called    
  		  "CLI", // Name of task
  		  256,   // Stack size
  		  NULL,  // Parameters to pass
  		  1, 	 // Task priority 
  		  NULL); // Task handle
  		  
  // Task 2: Sends Blinks the LED and sends message to msg_queue which passes to Task 1  
  xTaskCreate(
  		  blinkLED, 
  		  "BLINK", 
  		  256, 
  		  NULL, 
  		  1, 
  		  NULL);

  vTaskStartScheduler(); //This never returns... control handed to the RTOS

}

// Task 1: CLI (echo + parse "delay <ms>" + print messages)
static void doCLI(void *parameters)
{
  (void)parameters; // Unused parameters. Keeping the compiler happy 

  Message rcv_msg; // Message type variable to store received BLinked message from task 2 
  char c; // For storing each character received from UART
  char buf[buf_len]; // Buffer to store entire command received
  uint8_t idx = 0; // Used to cycle through buf for storing each char 
  const uint8_t cmd_len = (uint8_t)strlen(command); // Length of the command keyword or string "delay"

  memset(buf, 0, sizeof(buf)); // Reset the buffer with zeros

  while(1) 
  {
    // Print any message from blink task
    if (xQueueReceive(msg_queue, &rcv_msg, 0) == pdTRUE) {

      usartSendString("Message: ");
      usartSendString(rcv_msg.body);

	  // Parse Queue message and send it to terminal
      usartSendString(" ");
      char tmp[12];
      itoa((int)rcv_msg.count, tmp, 10);
      usartSendString(tmp);

      usartSendString("\r\n");
    }

    // Read USART
    if (usartCharReceived()) 
    {
      c = usartReadChar();

      // Echo back
      usartSendChar(c);

      // Enter pressed (CR or LF) then  parse the line
      if (c == '\r' || c == '\n') 
      {
        usartSendString("\r\n");
        buf[idx] = '\0';

        // If starts with "delay "
        if (memcmp(buf, command, cmd_len) == 0) 
        {
          // Parse the value
          char *tail = buf + cmd_len; // pointer arithmetic, buffer starts at [0] and we set the point at position [6]
          int led_delay = abs(atoi(tail)); // abs and atoi to Convert ascii to integer
          
          // Queue length = 1 so overwrite always keeps latest
          xQueueOverwrite(delay_queue, &led_delay);
        }

        // reset line buffer
        memset(buf, 0, sizeof(buf));
        idx = 0;
      }
      else {
        // Store if there is still space
        if (idx < (buf_len - 1)) {
          buf[idx++] = c;
        }
      }
    }

    vTaskDelay(1); // Blocking task 1 To give task 2 a chance to run
  }
}

// Task 2: Blink LED on PORTB1 (update delay + message every 100 blinks)
static void blinkLED(void *parameters)
{
  (void)parameters;

  Message msg; // To send Blinking message
  int led_delay = 500; // led delay value, default 500ms
  uint8_t counter = 0; // For counting blinks

  // Set PB1 as output
  DDRB |= (1 << PB1);

  while(1) 
  {
    // Check for new delay (non-blocking)
    if (xQueueReceive(delay_queue, &led_delay, 0) == pdTRUE) 
    {
      strcpy(msg.body, "New delay(ms):");
      msg.count = (uint16_t)led_delay;
      xQueueSend(msg_queue, &msg, 0);
    }

    // Need to blink using ticks
    TickType_t ticks = led_delay / portTICK_PERIOD_MS; // Convert to ticks because vTaskDelay expects ticks value so 500/15 = 33 ticks

	// Turn on LED
    PORTB |= (1 << PB1);
    vTaskDelay(ticks);

	// Turn off LED
    PORTB &= ~(1 << PB1);
    vTaskDelay(ticks);

    // message every 100 blinks
    counter++;
    if (counter >= blink_max) 
    {
      strcpy(msg.body, "Blinked:");
      msg.count = counter;   // should be 100
      xQueueSend(msg_queue, &msg, 0);
      counter = 0;
    }
  }
}
