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
 *
 * TODO: Modify the information here as appropriate!
 */

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
#include <math.h>

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
	// See the top of this file for assumptions used in this initialization.

	
	////////////////////////////////////////////////////////////////////

	/// FROM HERE REMOVE

//	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA
//
//	GPIOA->MODER &= ~(0b11 << 10);	// Set PA5 as input...
//	GPIOA->MODER |=  (0b10 << 10);	// ... then set it as alternate function.
//	GPIOA->OTYPER &= ~(1 << 5);	// PA5 = push-pull output
//	GPIOA->OSPEEDR &= ~(0b11 << 10);	// Fast mode (needed for PWM)
//	GPIOA->OSPEEDR |=  (0b10 << 10);
//
//	/*
//	 * For PA5 -- where the LED on the Nucleo board is -- only TIM2_CH1
//	 * is PWM-capable, per the *device* datasheet. This corresponds to
//	 * AF01.
//	 */
//	GPIOA->AFR[0] &= ~(0x00F00000);	// TIM2_CH1 on PA5 is AF01
//	GPIOA->AFR[0] |=  (0x00100000);
//
//	////////////////////////////////////////////////////////////////////
//
//	/*
//	 * In this setup, TIM2 is used for brightness control (see reason
//	 * above), while TIM1 is used to do input-capture on PA9 via
//	 * Channel 2.
//	 *
//	 * The internal clock for TIM2 is the same as the APB1 clock;
//	 * in turn, this clock is assumed the same as the system (AHB) clock
//	 * without prescaling, which results to the same frequency as HSI
//	 * (~16 MHz).
//	 *
//	 * TIM1, on the other hand, uses the APB2 clock; in turn, no prescaling
//	 * is done here, which also yields the same value as HSI (~16 MHz).
//	 */
//	RCC->APB1ENR	|= (1 << 0);	// Enable TIM2
//	RCC->APB2ENR	|= (1 << 0);	// Enable TIM1
//
//	/*
//	 * Classic PWM corresponds to the following:
//	 * 	- Edge-aligned	(CMS  = CR1[6:5] = 0b00)
//	 * 	- Upcounting	(DIR  = CR1[4:4] = 0)
//	 *	- Repetitive	(OPM  = CR1[3:3] = 0)
//	 * 	- PWM Mode #1	(CCxS[1:0] = 0b00, OCMx = 0b110)
//	 * 		- These are in CCMRy; y=1 for CH1 & CH2, and y=2 for
//	 * 		  CH3 & CH4.
//	 */
//	TIM2->CR1 &= ~(0b1111 << 0);
//	TIM2->CR1 &= ~(1 << 0);		// Make sure the timer is off
//	TIM2->CR1 |=  (1 << 7);		// Preload ARR (required for PWM)
//	TIM2->CCMR1 = 0x0068;		// Channel 1 (TIM2_CH1)
//
//	/*
//	 * Per the Nyquist sampling theorem (from EEE 147), to appear
//	 * continuous the PWM frequency must be more than twice the sampling
//	 * frequency. The human eye (the sampler) can usually go up to 24Hz;
//	 * some, up to 40-60 Hz. To cover all bases, let's use 500 Hz.
//	 *
//	 * In PWM mode, the effective frequency changes to
//	 *
//	 * (f_clk) / (ARR*(PSC+1))
//	 *
//	 * since each period must be able to span all values on the interval
//	 * [0, ARR). For obvious reasons, ARR must be at least equal to one.
//	 */
//	TIM2->ARR	= 100;		// Integer percentages; interval = [0,100]
//	TIM2->PSC	= (320 - 1);	// (16MHz) / (ARR*(TIM2_PSC + 1)) = 500 Hz
//
//	// Let main() set the duty cycle. We initialize at zero.
//	TIM2->CCR1	= 0;
//
//	/*
//	 * The LED is active-HI; thus, its polarity bit must be cleared. Also,
//	 * the OCEN bit must be enabled to actually output the PWM signal onto
//	 * the port pin.
//	 */
//	TIM2->CCER	= 0x0001;
//
//	////////////////////////////////////////////////////////////////////
//
//	// Pushbutton configuration
//	RCC->AHB1ENR |= (1 << 2);	// Enable GPIOC
//
//	GPIOC->MODER &= ~(0b11 << 26);	// Set PC13 as input...
//	GPIOC->PUPDR &= ~(0b11 << 26);	// ... without any pull-up/pull-down (provided externally).
//
//	////////////////////////////////////////////////////////////////////
//
//	/*
//	 * Enable the system-configuration controller. If disabled, interrupt
//	 * settings cannot be configured.
//	 */
//	RCC->APB2ENR |= (1 << 14);
//
//	/*
//	 * SYSCFG_EXTICR is a set of 4 registers, each controlling 4 external-
//	 * interrupt lines. Within each register, 4 bits are used to select
//	 * the source connected to each line:
//	 * 	0b0000 = Port A
//	 * 	0b0001 = Port B
//	 * 	...
//	 * 	0b0111 = Port H
//	 *
//	 * For the first EXTICR register:
//	 *	Bits 0-3   correspond to Line 0;
//	 * 	Bits 4-7   correspond to Line 1;
//	 *	Bits 8-11  correspond to Line 2; and
//	 * 	Bits 12-15 correspond to Line 3.
//	 *
//	 * The 2nd EXTICR is for Lines 4-7; and so on. Also, the line numbers
//	 * map 1-to-1 to the corresponding bit in each port. Thus, for example,
//	 * a setting of EXTICR2[11:8] == 0b0011 causes PD6 to be tied to
//	 * Line 6.
//	 *
//	 * For this system, PC13 would end up on Line 13; thus, the
//	 * corresponding setting is EXTICR4[7:4] == 0b0010. Before we set it,
//	 * mask the corresponding interrupt line.
//	 */
//	EXTI->IMR &= ~(1 << 13);		// Mask the interrupt
//	SYSCFG->EXTICR[3] &= (0b1111 << 4);	// Select PC13 for Line 13
//	SYSCFG->EXTICR[3] |= (0b0010 << 4);
//
//	/*
//	 * Per the hardware configuration, pressing the button causes a
//	 * falling-edge event to be triggered, and a rising-edge on release.
//	 * Since we are only concerned with presses, just don't trigger on
//	 * releases.
//	 */
//	EXTI->RTSR &= ~(1 << 13);
//	EXTI->FTSR |=  (1 << 13);
//
//	/*
//	 * Nothing more from the SCC, disable it to prevent accidental
//	 * remapping of the interrupt lines.
//	 */
//	RCC->APB2ENR &= ~(1 << 14);
//
//	////////////////////////////////////////////////////////////////////
//
//	/*
//	 *
//	 * Per the STM32F411xC/E family datasheet, the handler for EXTI_15_10
//	 * is at 40.
//	 *
//	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
//	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
//	 * and so on. Each index has 8 bits denoting the priority in the top
//	 * 4 bits; lower number = higher priority.
//	 *
//	 * Position 40 in the NVIC table would be at IPR[10][7:0]; or,
//	 * alternatively, just IP[40].
//	 */
//	NVIC->IP[40] = (1 << 4);
//	NVIC->IP[6]  = (0b1111 << 4);	// SysTick; make it least-priority
//
//	/*
//	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
//	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
//	 * I{S/C}ER[1], and so on -- one bit per line.
//	 *
//	 * ISER is written '1' to enable a particular interrupt, while ICER is
//	 * written '1' to disable the same interrupt. Writing '0' has no
//	 * effect.
//	 *
//	 * Position 25 in the NVIC table would be at I{S/C}ER[0][25:25]; while
//	 * position 27 would be at I{S/C}ER[0][27:27].
//	 */
//	NVIC->ISER[0] = (1 << 6);	// Note: Writing '0' is a no-op
//	NVIC->ISER[1] = (1 << 8);	// Note: Writing '0' is a no-op
//	EXTI->IMR |= (1 << 13);		// Unmask the interrupt on Line 13
//	TIM2->EGR |= (1 << 0);		// Trigger an update on TIM2
//	TIM2->CR1 |= (1 << 0);		// Activate both timers
//	irq_data.pressed = 0;
//
//	/*
//	 * Enable tick counting; the idea is to allow main() to perform
//	 * periodic tasks.
//	 */
//
//	/// UNTIL HERE REMOVE

	SysTick->LOAD = (20000-1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL  = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick
	
	// Do the initialization of USART last.
//	usart1_init();
//	usart2_init();
	usart6_init();

}

//NOTE insert yung mga initializations niyo and copy the necessary files for your initializations

///////////////////////////// START FOR MQ2 /////////////////////////////

uint16_t adc_value = 0x0000;

void GPIO_init(void);
void ADC_init(void);
void ADC_enable(void);
void ADC_startconv(void);
void ADC_waitconv(void);
int ADC_GetVal(void);
void SmokeSense(int);

void GPIO_init(void) {
	//enable GPIOA clock
	RCC->AHB1ENR |= (1 << 0);

	//configure PA1 to analog mode
	GPIOA->MODER |= (1 << 3);
	GPIOA->MODER |= (1 << 2);

	//configure PA8 as output
	GPIOA->MODER &= ~(1 << 17);
	GPIOA->MODER |= (1 << 16);

	//configure PA8 as push-pull output
	GPIOA->OTYPER &= ~(1 << 8);

	//set PA8 as initially high
	GPIOA->ODR |= (1 << 8);
}

void ADC_init(void) {
	//enable adc clock
	RCC->APB2ENR |= (1 << 8);

	//prescaler = 2
	ADC->CCR &= ~(1 << 16);
	ADC->CCR &= ~(1 << 17);

	//configure ADC resolution
	ADC1->CR1 &= ~(1 << 25);
	ADC1->CR1 &= ~(1 << 24);

	//Configure to Scan mode
	ADC1->CR1 |= (1 << 8);
	//Enable Interrupt for EOC
	ADC1->CR1 |= (1 << 5);
	//configure sampling time
	ADC1->SMPR2 &= ~(1 << 5);
	ADC1->SMPR2 &= ~(1 << 4);
	ADC1->SMPR2 |= (1 << 3);
	//end of conversion selection
	ADC1->CR2 &= ~(1 << 10);

	//configure data alignment
	ADC1->CR2 &= ~(1 << 11);

	//total number of conversions in the channel conversion sequence
	ADC1->SQR1 &= ~(1 << 23);
	ADC1->SQR1 &= ~(1 << 22);
	ADC1->SQR1 &= ~(1 << 21);
	ADC1->SQR1 &= ~(1 << 20);

	//assign channel for first conversion
	ADC1->SQR3 |= (1 << 0);

	//cont conversion mode
	ADC1->CR2 |= (1 << 1);
}

void ADC_enable(void) {
	ADC1->CR2 |= (1 << 0); //enable the adc
	delay(1); // required to ensure adc stable
}

void ADC_startconv(void) {
	ADC1->CR2 |= (1 << 30);
}

void ADC_waitconv(void) {
	//wait for the end of conversion
	while (!((ADC1->SR) & (1 << 1))) {
		;
	}
}

int ADC_GetVal(void) {
	return ADC1->DR; //read the value contained at the data register
}

void SmokeSense(int PPM_int){ // for pre-deployment testing
	if (
			PPM_int >= 300
			) {
		GPIOA->ODR &= ~(1 << 8);
	} else {
		GPIOA->ODR |= (1 << 8);
	}
}

float getPPM(int adc_value){
	float voltage = ((float)adc_value / 4095) * 5;
	float RS_in_Air = (5 - voltage) / voltage;
	float RS_RO_Ratio = RS_in_Air / 2.686;	// RO = 2.686
	float PPM = (float)(pow(10, ((log10(RS_RO_Ratio) - 1.617856412) / -0.44340257)));	// b = 1.617856412, m = -0.44340257
	return PPM;
}

///////////////////////////// END FOR MQ2 /////////////////////////////

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

	///////////////////////////// START FOR MQ2 /////////////////////////////

	float PPM_int; // Declare PPM variable here
	GPIO_init();
	ADC_init();

	///////////////////////////// END FOR MQ2 /////////////////////////////

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
					//WiFi("EEE192-429", "EEE192_Room429");
					//WiFi("neocastillo", "cocacola1");
					usart6_tx_send("AT+CWJAP=\"neocastillo\",\"cocacola1\"\r\n", strlen("AT+CWJAP=\"neocastillo\",\"cocacola1\"\r\n"));// follows the format ssid and password; specifically set for room 429 na ito
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
				// deployment: Y4ZG3F1672I2BWZH
				// testing: Q3EOIMT0WYFZDISW
				sprintf (local_buf1, "GET /update?api_key=Q3EOIMT0WYFZDISW&field1=%d\r\n", (int)round(PPM_int));
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

			//running code computations here then after the delay has been met it will automatically send data

			///////////////////////////// START FOR MQ2 /////////////////////////////
			//start ADC conversion
			ADC_enable();
			//start conversion
			ADC_startconv();
			//wait for conversion to finish
			ADC_waitconv();
			//store converted data to a variable
			adc_value = ADC_GetVal();

			PPM_int = round(getPPM(adc_value));

			SmokeSense(PPM_int); // for pre-deployment testing
			///////////////////////////// END FOR MQ2 /////////////////////////////

			// Send the received data
			if (wifi_status == 8 && current_send_data_time - last_send_data_time >= SEND_DATA_DELAY) {
				// Record the current time
				last_send_data_time = current_send_data_time;
				wifi_status -= 4;
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

