/**
 * @file	usart.h
 * @brief	Prototypes: Universal Synchronous/Asynchronous
 * 		Receiver/Transmitter library
 *
 * @author	Alberto de Villa <abdevilla@up.edu.ph>
 * @date	03 Dec 2023
 * @copyright
 * Copyright (C) 2022-2023. This source code was created as part of the author's
 * official duties with the Electrical and Electronics Engineering Institute,
 * University of the Philippines <https://eee.upd.edu.ph>
 */

#ifndef EEE158_USART_HDR_
#define EEE158_USART_HDR_

/*
 * Changelog:
 * 	[03 Dec 2023] Cleanup
 * 	[29 Nov 2022] Initial version
 */

#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stdbool.h>// C99 bool

#if __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////

/**
 * Structure describing a received event (or byte) from the USART
 * 
 * The reason we don't deal with 
 */
struct usart_rx_event {
	
	/**
	 * Received byte, if any
	 * 
	 * This member is valid only if 'has_data' is set.
	 */
	char	c;
	
	/**
	 * If set, then this event structure is valid
	 * 
	 * Applications must always check that this bit is set before touching
	 * other fields in the structure.
	 */
	unsigned int valid:1;
	
	/**
	 * If set, a framing error occurred
	 * 
	 * Although a break is also a framing error, it has its own bit; in
	 * such a case, this bit will be cleared.
	 */
	unsigned int err_frame:1;
	
	/**
	 * If set, a parity error occurred
	 * 
	 * This bit implies 'err_frame' being set, too.
	 */
	unsigned int err_parity:1;
	
	/**
	 * If set, a valid data byte is present at 'c'
	 */
	unsigned int has_data:1;
	
	/**
	 * If set, the receiver has detected an IDLE line.
	 * 
	 * This can be used to determine whether the BREAK condition has
	 * disappeared.
	 */
	unsigned int is_idle:1;
	
	/**
	 * If set, the receiver has detected a break.
	 * 
	 * In this case, no further communications will occur until an IDLE
	 * is detected.
	 */
	unsigned int is_break:1;
	
	// Reserved to align the structure on a 32-bit boundary
	unsigned int __reserved:18;
};

/**
 * Get an event from USART2
 * 
 * \param[out]	evt	Address of a buffer to receive the event
 * 
 * \return	TRUE if an event was returned, FALSE if not
 */
bool usart2_rx_get_event(struct usart_rx_event *evt);

////////////////////////////////////////////////////////////////////////////

/// Initialize the USART2 peripheral
void usart2_init(void);

/// Shutdown the USART2 peripheral
void usart2_exit(void);

/// Send a break on USART2's TX line
void usart2_tx_send_break(void);

/**
 * Send a buffer on USART2
 * 
 * \param[in]	buf	Buffer to transmit
 * \param[in]	len	Number of bytes to transmit
 * 
 * \return	TRUE if the buffer was successfully enqueued for transmission,
 * 		FALSE otherwise
 * 
 * WARNING: The buffer must NOT be modified while the transfer is taking place!
 */
bool usart2_tx_send(const char *buf, unsigned int len);

/**
 * Check if the transmitter is currently busy
 * 
 * \return	TRUE if the transmitter is busy, FALSE if not
 */
bool usart2_tx_is_busy(void);

#endif // EEE158_USART_HDR_
