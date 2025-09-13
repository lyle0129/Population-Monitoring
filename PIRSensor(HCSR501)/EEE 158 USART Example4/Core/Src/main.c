//code originated from Sir De Villa in EEE 158


/*
 * System configuration/build:
 * 	- Clock source == HSI (~16 MHz)
 * 		- No AHB & APB1/2 prescaling
 *	- Inputs:
 * 		- Active-LO NO-wired pushbutton @ PC13
 * 		- USART Input @ PA3 (USART2_RX)
 * 	- Outputs:
 * 		- Active-HI LED @ PA5
 *		- USART Output @ PA2 (USART2_TX)
 *
 * NOTE: This project uses the CMSIS standard for ARM-based microcontrollers;
 * 	 This allows register names to be used without regard to the exact
 * 	 addresses.
 */

#include "usart.h"
#include "usart1.h"
#include "usart6.h"
#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stdbool.h>// C99 bool
#include <stm32f4xx.h>	// Header for the specific device family

#include <stdio.h>	// Needed for snprintf()
#include <string.h>	// Needed for memcmp()
#include <ctype.h> // Include the header for isdigit function>
#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////

/*
 * IRQ data shared between the handlers and main()
 *
 * As asynchronous access is possible, all members are declared volatile.
 */
volatile struct
{
	/*
	 * If set, the button has been pressed.
	 * 
	 * This should be cleared in main().
	 */
	unsigned int pressed;
	
	/*
	 * Number of system ticks elapsed
	 */
	unsigned int nr_tick;
	
} irq_data;

/*
 * Handler for the interrupt
 *
 * This function name is special -- this name is used by the startup code (*.s)
 * to indicate the handler for this interrupt vector.
 */
void delay(int);
void EXTI15_10_IRQHandler(void)
{
	/*
	 * The hardware setup has PC13 being active-low. This must be taken
	 * into consideration to maintain logical consistency with the
	 * rest of the code.
	 */
	if (!(GPIOC->IDR & 0x2000))
		irq_data.pressed = 1;

	// Re-enable reception of interrupts on this line.
	EXTI->PR = (1 << 13);
}

// Handler for the system tick
void SysTick_Handler(void)
{
	irq_data.nr_tick += 1;
	SysTick->VAL = 0;
}

////////////////////////////////////////////////////////////////////////////

// Function to initialize the system; called only once on device reset
static void do_sys_config(void)
{
	
	/*
	 * Enable tick counting; the idea is to allow main() to perform
	 * periodic tasks.
	 */
	SysTick->LOAD = (20000-1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL  = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick
	
	//for on-board LED initialization
	RCC->AHB1ENR |= (1<<0); // Enables GPIOA peripheral
	RCC->AHB1ENR |= (1<<1); // Enables GPIOB peripheral

	GPIOA->MODER &= ~(1<<11); // PA5 as Output, MODER = 0b01
	GPIOA->MODER |= (1<<10); // PA5 as Output, MODER = 0b01
	GPIOA->OTYPER &= ~(1<<5); // Sets GPIOA, PIN 5 as push-pull
	GPIOA->ODR &= ~(1<<5); // PA5 initially LOW

	//for motion sensor initialization

	//Set GPIOA, PIN 10 as Input (MODER[21:20] = 00)
	GPIOA->MODER &= ~(1<<21); // clear bit 21
	GPIOA->MODER &= ~(1<<20); // clear bit 20

	//Set GPIOA, PIN 10 as no pull-up, no pull-down
	GPIOA->PUPDR &= ~(1<<21); // clear bit 21
	GPIOA->PUPDR &= ~(1<<20); // clear bit 20


	// Do the initialization of USART last.
	usart1_init();
	usart2_init();
	usart6_init();

}

//---------------------------------- Global variables --------------------------------------//

// Define a global variable to store the times (used for internal clocking : wont delay your program)
volatile uint32_t last_wifi_count_time = 0; // For Wifi setup, resending after time elapsed and have failed to receive "OK"
volatile uint32_t last_send_data_time = 0; // For sending of data, the delay between sends

// Define the delay (in milliseconds) between counting counts
const uint32_t WIFI_COUNT_DELAY_MS = 5000; // For Wifi setup, resending after time elapsed and have failed to receive "OK"
const uint32_t SEND_DATA_DELAY = 15000; //Delay for sending the data -- makes use of internal clock
// Additional delay for wifi_status 5 and 7
const uint32_t ADDITIONAL_WIFI_DELAY_MS = 2000;// this is used for cipsend and sending the data itself

// The heart of the program
int main(void)
{
	struct usart_rx_event usart1_evt;	// USART event
	struct usart_rx_event usart6_evt;	// USART event

	/*
	 * Buffer for storing data from the USART; necessary to properly parse
	 * multi-byte datagrams (eg. ANSI escape sequences).
	 */
	char		rxb6_data[100];
	unsigned int	rxb6_idx  = 0;
	unsigned int	rxb6_size = 0;
	char *s = NULL;
	uint8_t wifi_status = 0;
	uint8_t wifi_setup = 0; //make this into 1 if the wifi has been setup already
	char local_buf1[100] = {0}; //storing the data do be sent
	char local_buf2[30] = {0}; // storing the length of data cipsend to be sent
	char wifi_buf[30] = {0}; //for debugging only
	unsigned int 	motionDetected = 0;
	unsigned int	holdValue = 0;

	// Configure the system
	do_sys_config();

	// Wifi Reset
	if (!usart6_tx_is_busy()) {
		usart6_tx_send("AT+RST\r\n", strlen("AT+RST\r\n"));
		while (usart6_tx_is_busy());
		delay(5000); // need to wait since this has no OK to receive
	}
	/*
	 * Microcontroller main()'s are expected to never return; hence, the
	 * infinite loop.
	 */
	for (;;) {

		/////////////////////////////////////////////////////////////
		// Get the current time (for internal clock ticking = ito lang ang ginagamit ko rin para magresend when it failed to receive 'OK')
		uint32_t current_time_wifi = irq_data.nr_tick * 10; // Assuming ticks are every 10ms
		uint32_t current_send_data_time = irq_data.nr_tick * 10; // Assuming ticks are every 10ms

		//---------------------------- ESP8266 Configuration -------------------------------------//
		//configuring the esp8266 here
		uint32_t delay_time = WIFI_COUNT_DELAY_MS;
		if (wifi_status == 5 || wifi_status == 7) {
			delay_time += ADDITIONAL_WIFI_DELAY_MS;
		}
		if (current_time_wifi - last_wifi_count_time >= delay_time && wifi_status < 8) {
			rxb6_size = rxb6_idx = 0;
			if (wifi_status == 0){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT\r\n", strlen("AT\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
				}
			}

			//Set to client mode
			if (wifi_status == 1){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT+CWMODE=1\r\n", strlen("AT+CWMODE=1\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
				}
			}
			// join network
			if (wifi_status == 2){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT+CWJAP=\"JRS\",\"JarJarPlaysYT\"\r\n", strlen("AT+CWJAP=\"JRS\",\"JarJarPlaysYT\"\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
				}
			}

			// TCP/UDP connection to single
			if (wifi_status == 3){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT+CIPMUX=0\r\n", strlen("AT+CIPMUX=0\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
				}
			}

			//================== Sending Data Portion ====================//
			// TCP setting
			if (wifi_status == 4){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n", strlen("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
					wifi_setup = 1;
				}
			}

			// Preparing the sensor that we will send has the info on how long the data we are sending
			if (wifi_status == 5){
				// Clear the buffer
				memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
				rxb6_size = rxb6_idx = 0;
				sprintf (local_buf1, "GET /update?api_key=3B2EYQ6U0BX62HES&field1=%d\r\n", holdValue);
				int len = strlen (local_buf1);
				sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
				if (!usart6_tx_is_busy()) {
					usart6_tx_send(local_buf2, sizeof(local_buf2));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
				}
			}
			// sending data with the API key
			if (wifi_status == 7 ){

				if (!usart6_tx_is_busy()) {
					// Clear the buffer
					memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
					rxb6_size = rxb6_idx = 0;
					usart6_tx_send(local_buf1, sizeof(local_buf1));
					while (usart6_tx_is_busy());
					last_wifi_count_time = current_time_wifi;

				}
			}

		}
		//----------------------- Portion where receiving "OK" in my code ------------------------//
		//---------------------------- DO NOT EDIT ANYTHING HERE ---------------------------------//

		if (wifi_status < 8) {//will turn this off when not supposed to be receiving from esp8266
		// Check for any data received via USART6. This is for the ESP8266
			do {
				if (!usart6_rx_get_event(&usart6_evt))
					// Nothing to do here
					break;
				else if (!usart6_evt.valid)
					break;

				/*
				 * [1] If an IDLE is detected, update the size.
				 *
				 * [2] If no data is present, we're done.
				 */
				if (usart6_evt.is_idle) {
					rxb6_size = rxb6_idx;
					break;
				} else if (!usart6_evt.has_data) {
					break;
				}

				// Store the data
				if (rxb6_idx >= sizeof(rxb6_data)) {
					rxb6_size = rxb6_idx;
					break;
				}
				rxb6_data[rxb6_idx++] = usart6_evt.c;
				rxb6_data[rxb6_idx] = '\0';
				break;
			}while (0);


			// Checking if "OK" is received
			// after naging 8 ng wifi_status no need to check if okay was received
			if ((s = strstr(rxb6_data, "OK")) && (wifi_status != 5)) {
				// Clear the buffer and increment wifi_status
				memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
				rxb6_size = rxb6_idx = 0;
				wifi_status += 1;
				s = NULL;
			}
			if ((wifi_status == 5) && (s = strstr(rxb6_data, ">"))) {
				// Clear the buffer and increment wifi_status
				memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
				rxb6_size = rxb6_idx = 0;
				wifi_status += 2;
				s = NULL;
			}
		}
//-------------------------------------------------------------------------------------------------------------------------------------------------------//
		// ---------------------------------- PLACE YOUR CODE HERE ---------------------------//
		if (wifi_setup == 1) {

			//do your running code computations here then after the delay has been met it will automatically send data
			// NOTE: !! change yung variable ng nasa wifi_status == 5, its says your_variable !!

			if(GPIOA->IDR & (1<<10)){ // if motion is detected
				// motion_count++;
				GPIOA->ODR |= (1<<5); // PA5 to HIGH
				motionDetected = 1;
				holdValue = 1; //holds value 1 until it gets sent to Thingspeak
			}
			else{	// if motion is NOT detected
				GPIOA->ODR &= ~(1<<5); // PA5 to LOW
				motionDetected = 0;
			}


			// Send the received data
			if (wifi_status == 8 && current_send_data_time - last_send_data_time >= SEND_DATA_DELAY) {
				// Record the current time
				last_send_data_time = current_send_data_time;
				wifi_status -= 4;
				holdValue = 0;
			}

			}
		//--------------------------------------------------------------------------------------//
		}
	// This line is supposed to never be reached.
	return 1;
}

//delay; not accurate but just there for the initial reset, no need to be accurate since arbitrary chosen naman
void delay(int d){
	int i;
	for(; d>0 ;d--){
		for(i =0; i<2657;i++);
	}
}

