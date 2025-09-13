/**
 * @file	usart.c
 * @brief	Library code: Universal Synchronous/Asynchronous
 * 		Receiver/Transmitter
 *
 * @author	Alberto de Villa <abdevilla@up.edu.ph>
 * @date	03 Dec 2023
 * @copyright
 * Copyright (C) 2022-2023. This source code was created as part of the author's
 * official duties with the Electrical and Electronics Engineering Institute,
 * University of the Philippines <https://eee.upd.edu.ph>
 */

/*
 * Changelog:
 * 	[03 Dec 2023] Cleanup
 * 	[29 Nov 2022] Initial version
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

#include "usart1.h"
#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stdbool.h>// C99 bool
#include <stm32f4xx.h>	// Header for the specific device family
#include <string.h>

////////////////////////////////////////////////////////////////////////////

// Pending descriptor for USART2's transmission
static volatile const char   *fifo_tx_buf1 = 0;
static volatile unsigned int  fifo_tx_len1 = 0;

/*
 * Handler for interrupts from USART2
 *
 * This function name is special -- this name is used by the startup code (*.s)
 * to indicate the handler for this interrupt vector (USART2).
 */
#define RXFIFO_LEN1	64		// Must be a power of two
#define TXFIFO_LEN1	32		// Must be a power of two
static volatile unsigned int idx_rxf_11 = 0;
static volatile unsigned int idx_rxf_21 = 0;
static volatile unsigned int ctr_rxf1 = 0;
static volatile struct usart1_rx_event fifo_rx1[RXFIFO_LEN1];
void USART1_IRQHandler(void)
{
	/*
	 * Normally, there shouldn't be much code within an IRQ handler.
	 * However, the microcontroller family has a single IRQ handler for
	 * *all* USART2 events. Hence, some processing MUST be made here.
	 */
	
	/*
	 * Notes:
	 * [1] SR and DR must both be read to clear any interrupt status.
	 * [2] SR must be read *before* DR; otherwise, certain events would be
	 *     lost.
	 */

	unsigned int val_sr1 = USART1->SR; // val_sr contains the configuration of status register
	unsigned int val_dr1 = USART1->DR; // val_dr contains the configuration of data register

	struct usart1_rx_event evt = {
		.valid = 0
	};

	/*
	 * RX is more-urgent than TX, because we could lose data if we don't
	 * act fast enough. Thus, process it first.
	 */

	if (val_sr1 & (1 << 5)) {
		/*
		 * There is an event for FIFO (RXNE is set or bit 5 of SR is 1). Process it and its
		 * allies.
		 */
		evt.c     = (val_dr1 & 0xff); // puts data (lower 8 bits of DR) into the character attribute of evt
		evt.valid = 1;
		evt.has_data = 1;
		
		// Parity errors can only be detected here.
		if (val_sr1 & (1 << 1)) {
			/*
			 * Framing error detected (FE bit 1 is set)
			 * 
			 * Special case: If DR is zero, we assume a break and
			 * thus no further interrupts will occur on reception
			 * until IDLE is detected (which has its own interrupt
			 * line).
			 */
			if (val_dr1 == 0 ) {
				//if data register is empty
				evt.is_break = 1;
				evt.has_data = 0;
				USART1->CR1 &= ~(1 << 5); //inhibits interrupt because there is no data yet
			} else {
				// Other Frame error
				evt.err_frame = 1;
			}
		}
		
		if (val_sr1 & (1 << 0)) {
			// Parity error detected (PE bit 0 of SR is set)
			evt.err_parity = 1;
			evt.err_frame = 1;
			evt.has_data = 0;
		}
	}
	
	/*
	 * Check for any other RX errors/events that may have been detected by
	 * the peripheral.
	 */
	if (val_sr1 & (1 << 4)) {
		/*
		 * IDLE line is detected bit 4 is 1
		 * 
		 * Re-enable the RXNE interrupt.
		 */
		evt.is_idle  = 1;
		evt.valid = 1;
		// set bit 5 RXNEIE to 1
		USART1->CR1 |= (1 << 5); // interrupt is generated whenever read data register is not empty and is ready to be read i.e RXNE bit 5 = 1
	}
	
	// Put it into the RX FIFO queue
	if (evt.valid) {
		fifo_rx1[idx_rxf_11++] = evt; //stores evt at fifo_rx buffer at index 0 then increments indx by 1
		if (idx_rxf_11 >= RXFIFO_LEN1)
			idx_rxf_11 = 0; // meaning buffer is full
		if (ctr_rxf1 < RXFIFO_LEN1)
			++ctr_rxf1; // increments counter for receiveing buffer by 1 if buffer is not full
	}
	
	/////////////////////////////////////////////////////////////////////
	
	/*
	 * The interrupt may have been generated as well for transmit events.
	 * Send out the next byte in the queue, if any.
	 */
	if (val_sr1 & (1 << 7)) {
		/*
		 * Transmit buffer empty. If there are any pending characters,
		 * load them into the queue.
		 * 
		 * Because TXE bit 7 is only cleared by writing to the USART2_DR
		 * register i.e. not empty, the send routine just fills up the FIFO and
		 * enables TXEIE, which causes this routine to pick up the
		 * byte/s.
		 */
		if (fifo_tx_len1 > 0) {
			// Queue not empty
			USART1->DR = *(fifo_tx_buf1++); //just iterates through the contents of the transmitter buffer
			--fifo_tx_len1; //subtracts the length of buffer
		} else {
			// Queue empty; disable the TXE event
			USART1->CR1 &= ~(1 << 7); //disable interrupt
			fifo_tx_len1 = 0;
			fifo_tx_buf1 = 0;
		}
	}
}

////////////////////////////////////////////////////////////////////////////

// Initialize USART1
void usart1_init(void)
{
	// Initialize the variables.
	idx_rxf_11 = 0;
	idx_rxf_21 = 0;
	ctr_rxf1 = 0;
	memset(fifo_rx1, 0, sizeof(fifo_rx1)); //fills a block with 0, starting address, value, and size of the memory block to be filled i.e. this initializes the buffer
	
	// Configure the GPIO first before configuring the USART.
	
	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA; needed for both PA10 & PA9
	
	GPIOA->MODER &= ~(0b11 << 20);	// Set PA10 as input...
	GPIOA->MODER |=  (0b10 << 20);	// ... then set it as alternate function.
	GPIOA->MODER &= ~(0b11 << 18);	// Set PA9 as input...
	GPIOA->MODER |=  (0b10 << 18);	// ... then set it as alternate function.
	
	/*
	 * PA9 is used for TX output of the USART. As a result, we need to do
	 * the following:
	 * 
	 * - Push-pull output to get symmetrical drive; and
	 * - High-speed operation to avoid distorting the signal due to slew-
	 *   rate limiting.
	 */
	GPIOA->OTYPER &= ~(1 << 9);	// PA9 = push-pull output
	GPIOA->OSPEEDR |= (0b11 << 18);	// High-speed mode
	
	/*
	 * P10 is used for RX input of the USART. To enable break detection
	 * (that is, the remote peer is disconnected), activate the pull-down.
	 * 
	 * In an USART, the break condition is represented by the 'space' level
	 * (LO) being sent continuously for more than a frame period.
	 */
	GPIOA->PUPDR &= ~(0b11 << 20);
	GPIOA->PUPDR |=  (0b10 << 20);
	
	/*
	 * Set the AFR values:
	 * 
	 * USART1_TX = AF07 @ PA9
	 * USART1_RX = AF07 @ PA10
	 */
	GPIOA->AFR[1] &= ~(0x00000FF0);
	GPIOA->AFR[1] |=  (0x00000770);
	
	/////////////////////////////////////////////////////////////////////
	
	RCC->APB2ENR  |= (1 << 4);	// Enable USART1 peripheral clock enabled bit 17
	RCC->APB2RSTR |= (1 << 4);	// Reset the whole peripheral resets usart 2
	RCC->APB2RSTR &= ~(1 << 4);
	
	/*
	 * Disable both transmitters and receivers for now, while
	 * (re-)configuration is in progress.
	 */

	 //pins 9 and 10 are in usart1 
	USART1->CR1 &= ~(0b11 << 2); //disable receiver
	USART1->CR1 |=  (1 << 13); // usart enable 
	
	/*
	 * In USARTs, the baud rate and bit rate are, by convention, one and
	 * the same. This is because the first modems could send only one
	 * bit per baud period.
	 * 
	 * Per Section 19.3.4 of the STM32F4xxx family datasheet (USART), the
	 * baud rate is given by
	 * 
	 * baud = f_clk,usart / (8 * (2 - OVER8) * USARTDIV)
	 * 
	 * If no APB prescaling is used -- as is done in this setup --
	 * f_clk,usart is the same as the system clock frequency (=HSI, ~16MHz).
	 * 
	 * --- OVER8 ---
	 * To avoid spurious edge detection, the receiver always applies
	 * digital filtering to its input, in a manner similar to that used for
	 * timers in input-capture mode. (Quick recap: N consecutive HIs or LOs
	 * need to be detected to successfully read a HI or LO, respectively.)
	 * 'OVER8' determines whether N=8 (set) or N=16 (cleared). The former
	 * allows higher speeds, but with reduced noise immunity; while the
	 * latter is slower, but more resistant to line noise. More often,
	 * epsecially for externally-facing poirts, N=16 is used to reduce
	 * the risk of data corruption.
	 * 
	 * --- USART_DIV ---
	 * The upper 12 bits of the register always form the mantissa, while
	 * the lower 4 bits form the fraction. Depending on OVER8 (discussed
	 * above), the fraction base is either 1/16 (cleared) or 1/8 (set).
	 * 
	 * For example, suppose we have f_clk,usart = 16MHz and a desired baud
	 * rate of 38.4 kBaud. With OVER8=0, the numerical divisor would be
	 * 
	 * div = (16e6) / (8 * 2 * 38400) = 26.04166667
	 * 
	 * Since the fraction base is 1/16 = 0.0625, which is not an integer
	 * multiplier/divisor of 0.04166667, we will not be able to
	 * exactly represent this into USARTDIV. The two nearest values are
	 * 26.0000 and 26.0625, with the latter being nearer. Thus, our
	 * effective divisor is (26 + 1/16); its encoding in USARTDIV would be
	 * 
	 * 0000 0001 1010 _ 0001 = 0x01A1
	 * 
	 * This brings us to one important aspect of USARTs -- all receivers
	 * must be able to handle some tolerances, which is affected by the
	 * filtering provided by OVER8. For the example above, the actual baud
	 * rate with the selected divisor would be
	 * 
	 * baud = (16e6) / (8 * 2 * 26.0625) = 38369.30456
	 * 
	 * which represents a 0.08% error. If we used 26 for USARTDIV, the
	 * resulting baud rate would be 38461.53846, which represents a -0.16%
	 * error. Not bad, but still higher. These mismatches have worse
	 * effects at higher speeds, which ultimately limits the baud rate of
	 * simple UARTs to the hundred-kBaud range; though there are some
	 * that can reach 1.5Mbaud. Above such speeds, external clock
	 * synchronization must be used.
	 */
	USART1->BRR &= ~(0x0000FFFF);
	USART1->BRR |=  (0x0000003F);
	USART1->CR1 &= ~(1 << 15);		// OVER8 = 0
	USART1->CR2 &= ~(0b11 << 12);		// One (1) stop bit
	USART1->CR1 &= ~(1 << 10);		// No parity
	
	/*
	 * USARTs can also operate in synchronous mode, where data signals are
	 * synchronized with a separate external clock signal. Because the
	 * receiver no longer has to recover clocking information from the
	 * data signal, very high rates are possible in synchronous mode
	 * (like SPI).
	 * 
	 * PC-style UARTs have no such concept; for compatibility, this mode
	 * must be disabled (CLKEN=0).
	 */
	USART1->CR2 &= ~(1 << 11);
	
	/*
	 * A feature of modern microcontroller USARTs is multiprocessor
	 * communication, which causes the peripheral to append an extra data
	 * bit indicating to the receiver whether the character is address (set)
	 * or data (cleared). 
	 * 
	 * For compatibility with PC-style UARTs, this mode must not be used;
	 * however, the correct value for M depends on whether parity is used
	 * or not (PCEN). If PCEN is set, then the highest bit of the data
	 * register is ignored as it becomes the parity bit. To retain 8-bit
	 * mode, M must be cleared if parity is disabled, or set if parity
	 * is enabled.
	 * 
	 * In our configuration, no parity is used; hence, M must be cleared.
	 */
	USART1->CR1 &= ~(1 << 12);
	
	/*
	 * To facilitate higher speeds, modern USARTs support the use of
	 * hardware-based flow control, where two new lines -- Ready-to-Send
	 * (RTS) and Clear-to-send (CTS) -- are used to manage data flow on
	 * the TX/RX lines.
	 * 
	 * This system uses no such flow control; thus, disable them both.
	 */
	USART1->CR3 &= ~(0b11 << 8);
	
	//////////////////////////////////////////////////////////////////////
	
	/*
	 *
	 * Per the STM32F411xC/E family datasheet, USART2 has the handler at
	 * Position 38.
	 *
	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
	 * and so on. Each index has 8 bits denoting the priority in the top
	 * 4 bits; lower number = higher priority.
	 *
	 * Position 38 in the NVIC table would be at IPR[9][23:16]; or,
	 * alternatively, just IP[38].
	 *
	 * NOTE: Generally, priorities are ranked in the following order of
	 * 	 descending precedence:
	 * 
	 * 	- External events
	 * 	- Timer events
	 * 	- Communications events
	 */

	NVIC->IP[37] = (3 << 4);

	/*
	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
	 * I{S/C}ER[1], and so on -- one bit per line.
	 *
	 * ISER is written '1' to enable a particular interrupt, while ICER is
	 * written '1' to disable the same interrupt. Writing '0' has no
	 * effect.
	 *
	 * Position 37 in the NVIC table would be at I{S/C}ER[1][5:5].
	 */
	NVIC->ISER[1] = (1 << 5);	// Note: Writing '0' is a no-op
	
	/*
	 * THESE NEED TO BE DONE LAST. Enable the interrupts, followed by the
	 * actual receiver and transmitter duo.
	 */
	USART1->CR1 |= (0b1111 << 2);
	return;
}

// De-initialize the USART, freeing up resources for use by other peripherals
void usart1_exit(void){
	/*
	 * Disable the interrupts, and wait for any transmissions to complete.
	 */
	USART1->CR1 &= ~(0b1101 << 4);
	NVIC->ICER[1] = (1 << 5);		// Note: Writing '0' is a no-op
	while (!(USART1->SR & (1 << 6)));
	
	// Disable the rest of USART2
	USART1->CR1 = 0;
	RCC->APB2RSTR |= (1 << 4);	// Reset the whole peripheral
	RCC->APB2RSTR &= ~(1 << 4);
	RCC->APB2ENR  &= ~(1 << 4);	// Disable USART2 peripheral
	
	// Revert the GPIO pins to input.
	GPIOA->MODER &= ~(0b11 << 20);	// Set PA2 as input...
	GPIOA->MODER &= ~(0b11 << 18);	// Set PA3 as input...
	GPIOA->OTYPER  &= ~(1 << 9);
	GPIOA->OSPEEDR &= ~(0b11 << 18);
	GPIOA->PUPDR &= ~(0b11 << 20);
	GPIOA->AFR[1] &= ~(0x0000FF00);
	
	/*
	 * This time, we can't disable the peripheral because we don't know if
	 * someone else is using said peripheral.
	 */
	return;
}

// Get an event from the RX queue
bool usart1_rx_get_event(struct usart1_rx_event *evt)
{
	if (ctr_rxf1 > 0 && evt != 0) {
		*evt = fifo_rx1[idx_rxf_21++];
		--ctr_rxf1;
		if (idx_rxf_21 >= RXFIFO_LEN1)
			idx_rxf_21 = 0;
		return true;
	} else {
		return false;
	}
}

// Enqueue a buffer to be transmitted
bool usart1_tx_is_busy(void)
{
	return (fifo_tx_buf1 != 0 || fifo_tx_len1 > 0);
}
bool usart1_tx_send(const char *buf, unsigned int len)
{
	if (fifo_tx_buf1 != 0 || fifo_tx_len1 > 0)
		return false;
	
	fifo_tx_buf1 = buf;
	fifo_tx_len1 = len;
	USART1->CR1 |= (1 << 7);
	return true;
}
