#include "bsp.h"
#include "uart_stm32.h"

static uart_get_tx_data_type uart_tx_handler = NULL;
static uart_put_rx_data_type uart_rx_handler = NULL;

void uart_tx_irq( void );
void uart_rx_irq( void );

/******************************************************************************
 *	IRQs for STM32 using IAR
 */
#if ( defined __IAR_SYSTEMS_ICC__ ) &&	\
	( defined __ICCARM__ ) &&			\
	( defined BSP_BOARD_STM32 )

void USART_IRQ_HANDLER( void );
void USART_IRQ_HANDLER( void )
{
	bspIState_t istate;
	istate = __bsp_GET_ISTATE__();
	BSP_ENABLE_INTERRUPTS();

	uart_tx_irq( );
	uart_rx_irq( );

	__bsp_RESTORE_ISTATE__(istate);
	return;
}

#else
	#error "Undefined target processor or unknown compiler."
#endif

/******************************************************************************
 * Interrupt Service Routines (ISRs)
 */
/******************************************************************************
 * @fn          uart_tx_irq
 *
 * @brief       TX interrupt service routine
 *
 * input parameters
 *
 * output parameters
 *
 * @return   
 */
void uart_tx_irq( void )
{
	unsigned char c;
	uart_get_tx_data_type handler;
  
	BSP_CRITICAL_STATEMENT( handler = uart_tx_handler );
  
	/* if a handler exists */ 
	if( handler != NULL )
	{
		if( (*handler)( &c ) != false ) /* if this is not the last byte to send */
		{
			bspIState_t intState;
			BSP_ENTER_CRITICAL_SECTION( intState );

			/* only reset the interrupt flag if we have additional data to send
			 * that way, if we are done then the interrupt is still pending and
			 * will be immediately entered upon re-enabling it.
			 */
			//! UART_IRQ_FLAG_CLR( USART_NUMBER, TX ); /* eset the interrupt */

			BSP_EXIT_CRITICAL_SECTION( intState );
		}
		else
		{
			bspIState_t intState;
			BSP_ENTER_CRITICAL_SECTION( intState );
      
			/* we're done sending data.  since we left the interrupt pending,
			 * disable it so we don't re-enter the isr.  the interrupt will be
			 * re-enabled when there is another message to send.
			 */
			//! UART_IRQ_DISABLE( USART_NUMBER, TX );
      
			/* no more data to send, reset the handler to flag not busy */
			uart_tx_handler = NULL;
      
			BSP_EXIT_CRITICAL_SECTION( intState );
		}

		//! UART_SEND( USART_NUMBER, c ); /* send the byte */
	}
	else /* if no handler exists?!?!?!? */
		/* something went wrong, disable interrupts so we don't get stuck here */
		//! UART_IRQ_DISABLE( UART_NUMBER, TX );

	return;
}

/******************************************************************************
 * @fn          uart_rx_irq
 *
 * @brief       RX interrupt service routine
 *
 * input parameters
 *
 * output parameters
 *
 * @return   
 */
void uart_rx_irq( void )
{
	uart_put_rx_data_type handler;
  
	/* read in the received data, this will clear the interrupt also */
	unsigned char c = 0; //! UART_RECEIVE( UART_NUMBER );
  
	BSP_CRITICAL_STATEMENT( handler = uart_rx_handler );
  
	if( handler != NULL ) /* if a handler exists to receive data */
		if( ( *handler)( c ) == false ) /* if the user is done receiveing */
			/* indicate the receiver is available */
			BSP_CRITICAL_STATEMENT( uart_rx_handler = NULL );
	return;
}

/******************************************************************************
 * GLOBAL FUNCTIONS
 */ 

/******************************************************************************
 * @fn          uart_init
 *
 * @brief       Configures UART and sets up transmit and receive interrupts
 *
 * input parameters
 *
 * output parameters
 *
 * @return   
 */
void uart_init( void )
{
	volatile uint16_t i;

	/* make sure the handler functions are cleared in case we are re-initialized */
	uart_tx_handler = NULL;
	uart_rx_handler = NULL;

	/* initialize the uart interface for operations */
	/*
	UART_INIT( UART_NUMBER,
		UART_FLOW_CONTROL,		// enable/disable flow control
		UART_PARITY_MODE,		// enable/disable parity
		UART_STOP_BITS,			// number of stop bits
		UART_BAUD_RATE );		// baud rate to use
	*/   
	i = UART_BAUD_RATE >> 5; /* delay approximately 1 bit time */
	while( --i != 0 ) /* give the uart some time to initialize */
		; /* null statement */

	/* set the interrupt flag so that a transmit interrupt will be pending
	 * that way when a message is sent and the irq is enabled, the interrupt
	 * will happen immediately to start the transmission
	 */
	//	UART_IRQ_FLAG_SET( UART_NUMBER, TX ); // set the interrupt

	/* enable receive interrupts, they are always welcome. */
	// UART_IRQ_ENABLE( UART_NUMBER, RX );

	return;
}

/******************************************************************************
 * @fn          uart_tx_message
 *
 * @brief       Installs transmit handler if no message currently being sent
 *
 * input parameters
 * @param   handler - UART transmit handler
 *
 * @return   Status of the operation.
 *           true                 Transmit handler successfully installed
 *           false                Message being sent or handler is invalid
 *                                
 */
bool uart_tx_message( uart_get_tx_data_type handler )
{
	bspIState_t  intState;
	bool status = false; /* assume failure initially */

	/* updates required, store interrupt state and disable interrupts */
	BSP_ENTER_CRITICAL_SECTION(intState);

	/* if no message is currently being sent and handler looks valid */
	if( uart_tx_handler == NULL && handler != NULL )
	{
		uart_tx_handler = handler; /* install the handler */

		/* once the handler has been setup, enable the interrupt.
		 * this will cause the message to begin transmission
		 */
		// UART_IRQ_ENABLE( UART_NUMBER, TX ); 

		status = true; /* indicate success */    
	}
	BSP_EXIT_CRITICAL_SECTION(intState); /* restore interrupt state */
	return status; /* indicate status */
}

/******************************************************************************
 * @fn          uart_rx_message
 *
 * @brief       Installs receive handler if no message currently being received
 *
 * input parameters
 * @param   handler - UART receive handler
 *
 * @return   Status of the operation.
 *           true                 Receive handler successfully installed
 *           false                Message being received or handler is invalid
 *                                
 */
bool uart_rx_message( uart_put_rx_data_type handler )
{
	bspIState_t intState;
	bool status = false;  /* assume failure initially */
  
	/* updates required, store interrupt state and disable interrupts */
	BSP_ENTER_CRITICAL_SECTION(intState);

	/* if no message is being received and the handler looks valid */
	if( uart_rx_handler == NULL && handler != NULL )
	{
		uart_rx_handler = handler; /* install the handler */
		status = true; /* indicate success */
	}
	BSP_EXIT_CRITICAL_SECTION(intState); /* restore interrupt state */
	return status; /* indicate status */
}
