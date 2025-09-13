/**
 * @file	main.c
 * @brief	Module 06: Universal Asynchronous Receiver/Transmitter
 *
 * @author	Alberto de Villa <abdevilla@up.edu.ph>
 * @date	03 Dec 2023
 * @copyright
 * Copyright (C) 2022-2023. This source code was created as part of the author's
 * official duties with the Electrical and Electronics Engineering Institute,
 * University of the Philippines <https://eee.upd.edu.ph>
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
	////////////////////////////////////////////////////////////////////

	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA

	GPIOA->MODER &= ~(0b11 << 10);	// Set PA5 as input...
	GPIOA->MODER |=  (0b10 << 10);	// ... then set it as alternate function.
	GPIOA->OTYPER &= ~(1 << 5);	// PA5 = push-pull output
	GPIOA->OSPEEDR &= ~(0b11 << 10);	// Fast mode (needed for PWM)
	GPIOA->OSPEEDR |=  (0b10 << 10);

	/*
	 * For PA5 -- where the LED on the Nucleo board is -- only TIM2_CH1
	 * is PWM-capable, per the *device* datasheet. This corresponds to
	 * AF01.
	 */
	GPIOA->AFR[0] &= ~(0x00F00000);	// TIM2_CH1 on PA5 is AF01
	GPIOA->AFR[0] |=  (0x00100000);

	////////////////////////////////////////////////////////////////////

	/*
	 * In this setup, TIM2 is used for brightness control (see reason
	 * above), while TIM1 is used to do input-capture on PA9 via
	 * Channel 2.
	 *
	 * The internal clock for TIM2 is the same as the APB1 clock;
	 * in turn, this clock is assumed the same as the system (AHB) clock
	 * without prescaling, which results to the same frequency as HSI
	 * (~16 MHz).
	 *
	 * TIM1, on the other hand, uses the APB2 clock; in turn, no prescaling
	 * is done here, which also yields the same value as HSI (~16 MHz).
	 */
	RCC->APB1ENR	|= (1 << 0);	// Enable TIM2
	RCC->APB2ENR	|= (1 << 0);	// Enable TIM1

	/*
	 * Classic PWM corresponds to the following:
	 * 	- Edge-aligned	(CMS  = CR1[6:5] = 0b00)
	 * 	- Upcounting	(DIR  = CR1[4:4] = 0)
	 *	- Repetitive	(OPM  = CR1[3:3] = 0)
	 * 	- PWM Mode #1	(CCxS[1:0] = 0b00, OCMx = 0b110)
	 * 		- These are in CCMRy; y=1 for CH1 & CH2, and y=2 for
	 * 		  CH3 & CH4.
	 */
	TIM2->CR1 &= ~(0b1111 << 0);
	TIM2->CR1 &= ~(1 << 0);		// Make sure the timer is off
	TIM2->CR1 |=  (1 << 7);		// Preload ARR (required for PWM)
	TIM2->CCMR1 = 0x0068;		// Channel 1 (TIM2_CH1)

	/*
	 * Per the Nyquist sampling theorem (from EEE 147), to appear
	 * continuous the PWM frequency must be more than twice the sampling
	 * frequency. The human eye (the sampler) can usually go up to 24Hz;
	 * some, up to 40-60 Hz. To cover all bases, let's use 500 Hz.
	 *
	 * In PWM mode, the effective frequency changes to
	 *
	 * (f_clk) / (ARR*(PSC+1))
	 *
	 * since each period must be able to span all values on the interval
	 * [0, ARR). For obvious reasons, ARR must be at least equal to one.
	 */
	TIM2->ARR	= 100;		// Integer percentages; interval = [0,100]
	TIM2->PSC	= (320 - 1);	// (16MHz) / (ARR*(TIM2_PSC + 1)) = 500 Hz

	// Let main() set the duty cycle. We initialize at zero.
	TIM2->CCR1	= 0;

	/*
	 * The LED is active-HI; thus, its polarity bit must be cleared. Also,
	 * the OCEN bit must be enabled to actually output the PWM signal onto
	 * the port pin.
	 */
	TIM2->CCER	= 0x0001;

	////////////////////////////////////////////////////////////////////

	// Pushbutton configuration
	RCC->AHB1ENR |= (1 << 2);	// Enable GPIOC

	GPIOC->MODER &= ~(0b11 << 26);	// Set PC13 as input...
	GPIOC->PUPDR &= ~(0b11 << 26);	// ... without any pull-up/pull-down (provided externally).

	////////////////////////////////////////////////////////////////////

	/*
	 * Enable the system-configuration controller. If disabled, interrupt
	 * settings cannot be configured.
	 */
	RCC->APB2ENR |= (1 << 14);

	/*
	 * SYSCFG_EXTICR is a set of 4 registers, each controlling 4 external-
	 * interrupt lines. Within each register, 4 bits are used to select
	 * the source connected to each line:
	 * 	0b0000 = Port A
	 * 	0b0001 = Port B
	 * 	...
	 * 	0b0111 = Port H
	 *
	 * For the first EXTICR register:
	 *	Bits 0-3   correspond to Line 0;
	 * 	Bits 4-7   correspond to Line 1;
	 *	Bits 8-11  correspond to Line 2; and
	 * 	Bits 12-15 correspond to Line 3.
	 *
	 * The 2nd EXTICR is for Lines 4-7; and so on. Also, the line numbers
	 * map 1-to-1 to the corresponding bit in each port. Thus, for example,
	 * a setting of EXTICR2[11:8] == 0b0011 causes PD6 to be tied to
	 * Line 6.
	 *
	 * For this system, PC13 would end up on Line 13; thus, the
	 * corresponding setting is EXTICR4[7:4] == 0b0010. Before we set it,
	 * mask the corresponding interrupt line.
	 */
	EXTI->IMR &= ~(1 << 13);		// Mask the interrupt
	SYSCFG->EXTICR[3] &= (0b1111 << 4);	// Select PC13 for Line 13
	SYSCFG->EXTICR[3] |= (0b0010 << 4);

	/*
	 * Per the hardware configuration, pressing the button causes a
	 * falling-edge event to be triggered, and a rising-edge on release.
	 * Since we are only concerned with presses, just don't trigger on
	 * releases.
	 */
	EXTI->RTSR &= ~(1 << 13);
	EXTI->FTSR |=  (1 << 13);

	/*
	 * Nothing more from the SCC, disable it to prevent accidental
	 * remapping of the interrupt lines.
	 */
	RCC->APB2ENR &= ~(1 << 14);

	////////////////////////////////////////////////////////////////////

	/*
	 *
	 * Per the STM32F411xC/E family datasheet, the handler for EXTI_15_10
	 * is at 40.
	 *
	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
	 * and so on. Each index has 8 bits denoting the priority in the top
	 * 4 bits; lower number = higher priority.
	 *
	 * Position 40 in the NVIC table would be at IPR[10][7:0]; or,
	 * alternatively, just IP[40].
	 */
	NVIC->IP[40] = (1 << 4);
	NVIC->IP[6]  = (0b1111 << 4);	// SysTick; make it least-priority

	/*
	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
	 * I{S/C}ER[1], and so on -- one bit per line.
	 *
	 * ISER is written '1' to enable a particular interrupt, while ICER is
	 * written '1' to disable the same interrupt. Writing '0' has no
	 * effect.
	 *
	 * Position 25 in the NVIC table would be at I{S/C}ER[0][25:25]; while
	 * position 27 would be at I{S/C}ER[0][27:27].
	 */
	NVIC->ISER[0] = (1 << 6);	// Note: Writing '0' is a no-op
	NVIC->ISER[1] = (1 << 8);	// Note: Writing '0' is a no-op
	EXTI->IMR |= (1 << 13);		// Unmask the interrupt on Line 13
	TIM2->EGR |= (1 << 0);		// Trigger an update on TIM2
	TIM2->CR1 |= (1 << 0);		// Activate both timers
	irq_data.pressed = 0;

	/*
	 * Enable tick counting; the idea is to allow main() to perform
	 * periodic tasks.
	 */
	SysTick->LOAD = (20000-1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL  = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick
	
	// Do the initialization of USART last.
	usart1_init();
	usart2_init();
	usart6_init();

}


/////////////////////////////////////////////////////////////////////////////

char* computeCoordinateString(uint8_t low_byte, uint8_t high_byte) {
    // Combine low and high bytes to form a 16-bit unsigned integer
    uint16_t coordinate = low_byte + (high_byte * 256);
    // Declare mm as int16_t to hold signed 16-bit integer value
    int16_t mm;
    // Convert to signed 16-bit integer
    // Check if coordinate is greater than or equal to 32768
    if (coordinate >= 32768) {
        // Convert to signed 16-bit integer by subtracting 32768
        mm = (int16_t)coordinate - 32768;
    } else {
        // Convert to signed 16-bit integer by negating the value
        mm = -1 * coordinate;
    }
    // Allocate memory for the string
        char* result = (char*)malloc(50 * sizeof(char));
        if (result == NULL) {
            perror("Memory allocation failed");
            exit(EXIT_FAILURE);
    }
    // Format the coordinate into a stringaq
    sprintf(result, "%d mm", mm);
    return result;
}

char* computeSpeedString(uint8_t low_byte, uint8_t high_byte) {
    // Combine low and high bytes to form a 16-bit unsigned integer
    uint16_t coordinate = low_byte + (high_byte * 256);
    // Declare mm as int16_t to hold signed 16-bit integer value
    int16_t mm;
    // Convert to signed 16-bit integer
    // Check if coordinate is greater than or equal to 32768
    if (coordinate >= 32768) {
        // Convert to signed 16-bit integer by subtracting 32768
        mm = (int16_t)coordinate - 32768;
    } else {
        // Convert to signed 16-bit integer by negating the value
        mm = -1 * coordinate;
    }
    // Allocate memory for the string
        char* result = (char*)malloc(50 * sizeof(char));
        if (result == NULL) {
            perror("Memory allocation failed");
            exit(EXIT_FAILURE);
    }
    // Format the coordinate into a string
    sprintf(result, "%d cm/s", mm);
    return result;
}

void sendEscapeSeq() {
    const char* escape_seq = "\x1B[2J\x1B[H"; // ANSI escape sequence to clear screen
    usart2_tx_send(escape_seq, strlen(escape_seq));
    while (usart2_tx_is_busy());
}

void bufclr (char *buf){
	int len = strlen (buf);
	for (int i=0; i<len; i++) buf[i] = '\0';
}

// The heart of the program
int main(void)
{
	struct usart_rx_event usart1_evt;	// USART event
	struct usart_rx_event usart6_evt;	// USART event

	/*
	 * Buffer for storing data from the USART; necessary to properly parse
	 * multi-byte datagrams (eg. ANSI escape sequences).
	 */
	char		rxb_data[30];
	unsigned int	rxb_idx  = 0;
	unsigned int	rxb_size = 0;
	uint8_t	count = 0;
	unsigned int	population = 0;
	char		rxb6_data[100];
	unsigned int	rxb6_idx  = 0;
	unsigned int	rxb6_size = 0;
	char *s = NULL;
//	unsigned int	wifi_status = 0;
	uint8_t wifi_status = 0;
	uint8_t wifi_setup = 0; //make this into 1 if the wifi has been setup already
	char local_buf1[100] = {0};
	char local_buf2[30] = {0};
	char wifi_buf[30] = {0}; //for debugging

	// Define a global variable to store the time of the last population count
	volatile uint32_t last_population_count_time = 0;
	volatile uint32_t last_wifi_count_time = 0; // For Wifi setup, resending after time elapsed and have failed to receive "OK"
	volatile uint32_t last_send_data_time = 0; // For sending of data, the delay between sends
	volatile uint32_t wifi_status_time = 0; // For sending of data, the delay between sends

	// Define the delay (in milliseconds) between population counts
	const uint32_t POPULATION_COUNT_DELAY_MS = 1000;
	const uint32_t WIFI_COUNT_DELAY_MS = 5000; // For Wifi setup, resending after time elapsed and have failed to receive "OK"
	const uint32_t SEND_DATA_DELAY = 50000; //Delay for sending the data -- makes use of internal clock
	// Additional delay for wifi_status 5 and 7
	const uint32_t ADDITIONAL_WIFI_DELAY_MS = 2000;
	const uint32_t WIFI_STATUS_TIMEOUT = 100000;

	// Configure the system
	do_sys_config();

	// Wifi Reset
	if (!usart6_tx_is_busy()) {
		usart6_tx_send("AT+RST\r\n", strlen("AT+RST\r\n"));
		while (usart6_tx_is_busy());
		delay(5000);
	}
	/*
	 * Microcontroller main()'s are expected to never return; hence, the
	 * infinite loop.
	 */
	for (;;) {

		/////////////////////////////////////////////////////////////
		// Get the current time
		uint32_t current_time = irq_data.nr_tick * 10; // Assuming ticks are every 10ms
		uint32_t current_time_wifi = irq_data.nr_tick * 10; // Assuming ticks are every 10ms
		uint32_t current_send_data_time = irq_data.nr_tick * 10; // Assuming ticks are every 10ms

		// When Nearing to the limit of uint32_t reset everything to zero to fix wrap-around
		if (current_time >= 4000000000){
			// reseting when time reaches at this point
			NVIC_SystemReset();

//			// Reseting my variables into zero didnt work me thinks?
//			current_time = 0;
//			current_time_wifi = 0;
//			current_send_data_time = 0;
//			last_population_count_time = 0;
//			last_wifi_count_time = 0; // For Wifi setup, resending after time elapsed and have failed to receive "OK"
//			last_send_data_time = 0; // For sending of data, the delay between sends
//			wifi_status_time = 0; // For sending of data, the delay between sends
		}

		//---------------------------- ESP8266 Configuration -------------------------------------//
		//configuring the esp8266 here
		uint32_t delay_time = WIFI_COUNT_DELAY_MS;
		if (wifi_status == 5 || wifi_status == 7) {
			delay_time += ADDITIONAL_WIFI_DELAY_MS;
		}
		if (current_time_wifi - last_wifi_count_time >= delay_time && wifi_status < 8) {
			// Clear the buffer
//			memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
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
//					usart6_tx_send("AT+CWJAP=\"EEE192-429\",\"EEE192_Room429\"\r\n", strlen("AT+CWJAP=\"EEE192-429\",\"EEE192_Room429\"\r\n"));
//					while (usart6_tx_is_busy());
					usart6_tx_send("AT+CWJAP=\"Lyle\",\"selenium\"\r\n", strlen("AT+CWJAP=\"Lyle\",\"selenium\"\r\n"));
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

			// TCP setting
			if (wifi_status == 4){
				if (!usart6_tx_is_busy()) {
					usart6_tx_send("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n", strlen("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n"));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
					wifi_setup = 1;
					wifi_status += 1;
					delay(1500);
				}
			}

			//================== Sending Data Portion ====================//
			// Preparing the sensor that we will send
			if (wifi_status == 5){
				// Clear the buffer
				memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
				rxb6_size = rxb6_idx = 0;
//				sprintf (local_buf1, "GET /update?api_key=DOMEQU194VW41HBN&field2=%d\r\n", population);
				sprintf (local_buf1, "GET /update?api_key=HPSO01RYLJ4EC7VD&field1=%d\r\n", population);
				int len = strlen (local_buf1);
				sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
				if (!usart6_tx_is_busy()) {
					usart6_tx_send(local_buf2, sizeof(local_buf2));
					while (usart6_tx_is_busy());
					//Record the current time
					last_wifi_count_time = current_time_wifi;
					wifi_status += 2;
					delay(1500);
				}
			}
			// sending API key
			if (wifi_status == 7 ){

				if (!usart6_tx_is_busy()) {
					// Clear the buffer
					memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
					rxb6_size = rxb6_idx = 0;
					usart6_tx_send(local_buf1, sizeof(local_buf1));
					while (usart6_tx_is_busy());
					last_wifi_count_time = current_time_wifi;
					wifi_status += 1;
					delay(1500);

				}
			}

		}
		//------------------------------------------------------------------------------------//

		//Checker if it has been staying in a single status for a long time
		if ((current_time_wifi - wifi_status_time) >= WIFI_STATUS_TIMEOUT) {
//			sendEscapeSeq();
//			if (!usart2_tx_is_busy()) {
//				usart2_tx_send("RESETING NOW", strlen("RESETING NOW"));
//				while (usart2_tx_is_busy());
//			}
			NVIC_SystemReset();
		}
		// Check for any data received via USART1. [USART 1 IS USED FOR RECEIVING FROM LD2450]
		do {
			if (!usart1_rx_get_event(&usart1_evt))
				// Nothing to do here
				break;
			else if (!usart1_evt.valid)
				break;

			/*
			 * [1] If an IDLE is detected, update the size.
			 *
			 * [2] If no data is present, we're done.
			 */
			if (usart1_evt.is_idle) {
				rxb_size = rxb_idx;
				break;
			} else if (!usart1_evt.has_data) {
				break;
			}

			// Store the data
			if (rxb_idx >= sizeof(rxb_data)) {
				rxb_size = rxb_idx;
				break;
			}
			rxb_data[rxb_idx++] = usart1_evt.c;
			count += 1;
			rxb_data[rxb_idx] = '\0';
			break;
		}while (0);



		//----------------------- Portion where receiving "OK" in my code ------------------------//

		if (wifi_status <= 8) {//will turn this off when not supposed to be receiving from esp8266
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
			if (wifi_status < 4) {
				if ((s = strstr(rxb6_data, "OK")) && (wifi_status != 5)) {
					// Clear the buffer and increment wifi_status
					memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
					rxb6_size = rxb6_idx = 0;
					wifi_status += 1;
					s = NULL;
					last_send_data_time = current_send_data_time;
					wifi_status_time = current_time_wifi;
				}
				if ((wifi_status == 5) && (s = strstr(rxb6_data, ">"))) {
					// Clear the buffer and increment wifi_status
					memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
					rxb6_size = rxb6_idx = 0;
					wifi_status += 2;
					s = NULL;
					wifi_status_time = current_time_wifi;
				}
			}
			// after naging 8 ng wifi_status no need to check if okay was received
			if ((wifi_status == 8) && (s = strstr(rxb6_data, "CLOSED"))) {
				// Clear the buffer and increment wifi_status
				memset(rxb6_data, 0, sizeof(rxb6_data)); // Clear the buffer
				rxb6_size = rxb6_idx = 0;
				wifi_status += 1;
				s = NULL;
				wifi_status_time = current_time_wifi;
				bufclr(local_buf1);
				bufclr(local_buf2);
			}

		}

		// ---------------------------------- LD2450 Portion of the code ---------------------------//
		if (count == 30 && wifi_setup == 1) {// since my UART is sending 30 bytes of data I have a counter above and after the counter has been filled I will send out yung data ko to the uart to display
			//introduced a checker for the header and footer to verify that I am getting the data values in between
			if (rxb_data[0] == 170 && rxb_data[1] == 255 && rxb_data[2] == 3 && rxb_data[3] == 0 && rxb_data[28] == 85 && rxb_data[29] == 204) {
				//Object 1
				char* pos1x = computeCoordinateString(rxb_data[4], rxb_data[5]);
				char* pos1y = computeCoordinateString(rxb_data[6], rxb_data[7]);
				char* speed1 = computeSpeedString(rxb_data[8], rxb_data[9]);
				int pos1y_counter = atoi(pos1y); // Convert string to integer
				int pos1x_counter = atoi(pos1x); // Convert string to integer
				int speed1_counter = atoi(speed1);
				if (pos1x_counter >= 800 && pos1x_counter <= 1300){
					if (pos1y_counter > 800) {
						if (current_time - last_population_count_time >= POPULATION_COUNT_DELAY_MS) {
						// Record the current time
							last_population_count_time = current_time;
							if (speed1_counter > 0){
								population -= 1;
							} else if (speed1_counter < 0) {
								population += 1;
							}
						}
					}
				}
				// Allocate memory for the string
				char* population_str = (char*)malloc(50 * sizeof(char));
				if (population_str == NULL) {
					perror("Memory allocation failed");
					exit(EXIT_FAILURE);
				}
//				// Format the coordinate into a string
//				sprintf(population_str, "%d people", population);
//				// Send the received data
//				if (wifi_status == 9 && current_send_data_time - last_send_data_time >= SEND_DATA_DELAY) {
//					// Record the current time
//					last_send_data_time = current_send_data_time;
//					wifi_status -= 5;
//				}
//				if (wifi_status == 8 && current_send_data_time - last_send_data_time >= SEND_DATA_DELAY) {
//					// Record the current time
//					last_send_data_time = current_send_data_time;
//					bufclr(local_buf1);
//					bufclr(local_buf2);
//					wifi_status -= 4;
//				}
//				sendEscapeSeq();
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("room population count: ", 22);
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(population_str,strlen(population_str));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("Obect 1: {x, y, speed}", 22);
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//				    while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos1x, strlen(pos1x));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//				    usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//				    while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos1y, strlen(pos1y));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//		            while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(speed1, strlen(speed1));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//			        while (usart2_tx_is_busy());
//				}

				//Object 2
				char* pos2x = computeCoordinateString(rxb_data[12], rxb_data[13]);
				char* pos2y = computeCoordinateString(rxb_data[14], rxb_data[15]);
				char* speed2 = computeSpeedString(rxb_data[16], rxb_data[17]);
				// Send the received data
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("Obect 2: {x, y, speed}", 22);
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos2x, strlen(pos2x));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					 while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos2y, strlen(pos2y));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(speed2, strlen(speed2));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}

				//Object 3
				char* pos3x = computeCoordinateString(rxb_data[20], rxb_data[21]);
				char* pos3y = computeCoordinateString(rxb_data[22], rxb_data[23]);
				char* speed3 = computeSpeedString(rxb_data[24], rxb_data[25]);
				// Send the received data
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("Obect 3: {x, y, speed}", 22);
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos3x, strlen(pos3x));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(pos3y, strlen(pos3y));
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send("\r\n", 2); // Carriage return and newline characters to move to the next line
//					while (usart2_tx_is_busy());
//				}
//				if (!usart2_tx_is_busy()) {
//					usart2_tx_send(speed3, strlen(speed3));
//					while (usart2_tx_is_busy());
//				}

				rxb_size = rxb_idx = 0;  // Reset the index to clear the contents
				count = 0;
				// Free memory allocated for pos1x and pos1y
				free(pos1x);
				free(pos1y);
				free(pos2x);
				free(pos2y);
				free(pos3x);
				free(pos3y);
				free(speed1);
				free(speed2);
				free(speed3);
				free(population_str);
				}else {
			        // Shift the data in the array to the left by one position
					//doing this to shift sensor data back incase nagmove from wifi delays
			        for (int i = 0; i < 29; i++) {
			            rxb_data[i] = rxb_data[i + 1];
			        }
			        rxb_idx--;
			        count--;
			    }
			}
		/////////////////////////////////////////////////////////////////////////////////////////
		}
	// This line is supposed to never be reached.
	return 1;
}

//delay; but do not use!! this messes up the time it receives data in UART
void delay(int d){
	int i;
	for(; d>0 ;d--){
		for(i =0; i<2657;i++);
	}
}

