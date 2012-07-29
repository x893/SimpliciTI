#ifndef uart_h
#define uart_h

#include <stdbool.h> /* supports bool, true, and false */
#include <stddef.h>  /* supports NULL macro */
#include "bsp.h"

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/* ------------------------------------------------------------------------------------------------
 *                          MACROS AND DEFINES FOR ALL 8051 VARIANTS
 * ------------------------------------------------------------------------------------------------
 */

#if	( defined __IAR_SYSTEMS_ICC__ )	&& \
	( defined __ICCARM__ )			&& \
	( defined BSP_BOARD_STM32 )

	#define USART_NUMBER		USART2
	#define USART_IRQ_HANDLER	USART2_IRQHandler

#endif /* defined ( __IAR_SYSTEMS_ICC__ && __ICCARM__ && BSP_BOARD_STM32 ) */

/* ------------------------------------------------------------------------------------------------
 *                                Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* this type represents a function to call for each data character being
 * transmitted across the uart.  it will be called at the character data rate.
 * the user should return true if there are still more characters to send and
 * false if this is the last character to send.  the character to send should
 * be placed at the position pointed to by the passed parameter */
typedef bool ( *uart_get_tx_data_type )( unsigned char* );

/* this type represents a function to call for each data character that is
 * received from the uart.  it will be called at the character data rate.
 * the character recieved is passed as the parameter to the function.
 * the user should return true if it is willing to accept more data, a return
 * value of false indicates the user is closing this message and no longer 
 * wants to accept data from the uart. */
typedef bool ( *uart_put_rx_data_type )( unsigned char );


/* ------------------------------------------------------------------------------------------------
 *                                Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */

/* initializes the uart for operation.  this function should be called before
 * any other uart functions are called. */
void uart_init( void );

/* attempts to begin a uart transmission.  if <handler> is NULL or there is
 * currently another message being sent, false is returned.  otherwise, true
 * is returned indicating the message will be sent immediately.
 * the handler passed must be able to respond to character by character
 * requests from the isr.  see the description of uart_get_tx_data_type above
 * for more details. */
bool uart_tx_message( uart_get_tx_data_type handler );

/* attempts to begin a uart reception.  if <handler> is NULL or there is
 * currently another message being received, false is returned.  otherwise,
 * true is returned indicating the handler has been accepted for any new
 * data received.  the handler passed must be able to respond to character by
 * character requests from the isr.  see the description of
 * uart_put_rx_data_type above for more details. */
bool uart_rx_message( uart_put_rx_data_type handler );

#endif /* uart_h */
