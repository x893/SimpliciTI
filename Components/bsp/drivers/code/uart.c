/**************************************************************************************************
  Filename:       uart.c
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This file supports the SimpliciTI-compatible UART driver.

  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include "bsp.h"
#include "uart.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
static uart_get_tx_data_type uart_tx_handler = NULL;
static uart_put_rx_data_type uart_rx_handler = NULL;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
void uart_tx_irq( void );
void uart_rx_irq( void );

/******************************************************************************
 * IRQs for all 8051 variants using IAR
 */

#if ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC8051__ )

/* vector all interrupts to common routines */
#pragma vector = UART_IRQ_VECTOR( UART_NUMBER_0, TX )
__idata_reentrant __interrupt void uart_0_tx_irq( void );
#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_0, TX )
__idata_reentrant __interrupt void uart_0_tx_irq( void )
{
	/* work-a-round for XCH instruction */
	bspIState_t istate;
	istate = __bsp_GET_ISTATE__();
	BSP_ENABLE_INTERRUPTS();

	uart_tx_irq( );

	/* work-a-round for XCH instruction */
	__bsp_RESTORE_ISTATE__(istate);

	return;
}

#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_1, TX )
__idata_reentrant __interrupt void uart_1_tx_irq( void );
#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_1, TX )
__idata_reentrant __interrupt void uart_1_tx_irq( void )
{
	/* work-a-round for XCH instruction */
	bspIState_t istate;
	istate = __bsp_GET_ISTATE__();
	BSP_ENABLE_INTERRUPTS();

	uart_tx_irq( );

	/* work-a-round for XCH instruction */
	__bsp_RESTORE_ISTATE__(istate);
  
	return;
}

#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_0, RX )
__idata_reentrant __interrupt void uart_0_rx_irq( void );
#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_0, RX )
__idata_reentrant __interrupt void uart_0_rx_irq( void )
{
	/* work-a-round for XCH instruction */
	bspIState_t istate;
	istate = __bsp_GET_ISTATE__();
	BSP_ENABLE_INTERRUPTS();

	uart_rx_irq( );

	/* work-a-round for XCH instruction */
	__bsp_RESTORE_ISTATE__(istate);
  
	return;
}

#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_1, RX )
__idata_reentrant __interrupt void uart_1_rx_irq( void );
#pragma vector=UART_IRQ_VECTOR( UART_NUMBER_1, RX )
__idata_reentrant __interrupt void uart_1_rx_irq( void )
{
	/* work-a-round for XCH instruction */
	bspIState_t istate;
	istate = __bsp_GET_ISTATE__();
	BSP_ENABLE_INTERRUPTS();

	uart_rx_irq( );

	/* work-a-round for XCH instruction */
	__bsp_RESTORE_ISTATE__(istate);
  
	return;
}


/******************************************************************************
 * IRQs for MSP430+CCxxxx using IAR
 */

#elif ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC430__ )

#pragma vector=USCIAB0TX_VECTOR
__interrupt void uart_tx_enter_irq( void );
#pragma vector=USCIAB0TX_VECTOR
__interrupt void uart_tx_enter_irq( void )
{
#if( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE )
  /* if remote device can accept more data */ 
  if( UART_CTS_GET_STATE( ) == UART_CTS_DEASSERTED )
    uart_tx_irq( ); /* handle the transmit request */
  
  else /* if remote device is holding us off then disable interrupts,
        * they will get re-enabled when the CTS interrupt is triggered.
        * this way we don't get stuck in an infinite loop because we are not
        * handling the transmit interrupt, we will get back here once the
        * CTS interrupt is generated because it will enable interrupts again. */
    UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX );
#else
  uart_tx_irq( ); /* handle the transmit request */ 
#endif
  return;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void uart_rx_enter_irq( void );
#pragma vector=USCIAB0RX_VECTOR
__interrupt void uart_rx_enter_irq( void )
{
  uart_rx_irq( );
  return;
}

#if( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE )
#pragma vector=INFIX( PORT, UART_CTS_PORT_NUM, _VECTOR )
__interrupt void uart_cts_irq( void );
#pragma vector=INFIX( PORT, UART_CTS_PORT_NUM, _VECTOR )
__interrupt void uart_cts_irq( void )
{
  /* if a handler exists then a message must be currently in the buffer so
   * it is safe to re-enable interrupts */
  if( uart_tx_handler != NULL )
    UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, TX );
  
  UART_CTS_CLEAR_INTERRUPT( );
  
  return;
}
#endif


/******************************************************************************
 * IRQs for MSP430+CCxxxx using Code Composer
 */

#elif  defined __TI_COMPILER_VERSION__ 

BSP_ISR_FUNCTION( uart_tx_enter_irq, USCIAB0TX_VECTOR )
{
#if( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE )
  /* if remote device can accept more data */
  if( UART_CTS_GET_STATE( ) == UART_CTS_DEASSERTED )
    uart_tx_irq( ); /* handle the transmit request */ 
  
  else /* if remote device is holding us off then disable interrupts,
        * they will get re-enabled when the CTS interrupt is triggered.
        * this way we don't get stuck in an infinite loop because we are not
        * handling the transmit interrupt, we will get back here once the
        * CTS interrupt is generated because it will enable interrupts again. */
    UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX );
#else
  uart_tx_irq( ); /* handle the transmit request */
#endif
  return;
}

BSP_ISR_FUNCTION( uart_rx_enter_irq, USCIAB0RX_VECTOR )
{
  uart_rx_irq( );
  return;
}

#if( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE )
BSP_ISR_FUNCTION( uart_cts_irq, INFIX( PORT, UART_CTS_PORT_NUM, _VECTOR ) )
{
  /* if a handler exists then a message must be currently in the buffer so
   * it is safe to re-enable interrupts */
  if( uart_tx_handler != NULL )
    UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, TX );
  
  UART_CTS_CLEAR_INTERRUPT( );
  
  return;
}
#endif




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
       * will be immediately entered upon re-enabling it.*/
      UART_IRQ_FLAG_CLR( UART_NUMBER, UART_LOCATION, TX ); /* eset the interrupt */
      
      BSP_EXIT_CRITICAL_SECTION( intState );
      }
    else
      {
      bspIState_t intState;
      BSP_ENTER_CRITICAL_SECTION( intState );
      
      /* we're done sending data.  since we left the interrupt pending,
       * disable it so we don't re-enter the isr.  the interrupt will be
       * re-enabled when there is another message to send. */
      UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX );
      
      /* no more data to send, reset the handler to flag not busy */
      uart_tx_handler = NULL;
      
      BSP_EXIT_CRITICAL_SECTION( intState );
      }

    UART_SEND( UART_NUMBER, UART_LOCATION, c ); /* send the byte */
    }
  else /* if no handler exists?!?!?!? */
    /* something went wrong, disable interrupts so we don't get stuck here */
    UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX );

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
  unsigned char c = UART_RECEIVE( UART_NUMBER, UART_LOCATION );
  
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
  volatile unsigned int i;

  /* make sure the handler functions are cleared in case we are re-initialized */
  uart_tx_handler = NULL;
  uart_rx_handler = NULL;

  /* initialize the uart interface for operations */
  UART_INIT( UART_NUMBER,
             UART_LOCATION,
             UART_FLOW_CONTROL,    /* enable/disable flow control */
             UART_PARITY_MODE,     /* enable/disable parity */
             UART_STOP_BITS,       /* number of stop bits */
             UART_BAUD_RATE );     /* baud rate to use */
   
  i = UART_BAUD_RATE >> 5; /* delay approximately 1 bit time */
  while( --i != 0 ) /* give the uart some time to initialize */
      ; /* null statement */

  /* set the interrupt flag so that a transmit interrupt will be pending
   * that way when a message is sent and the irq is enabled, the interrupt
   * will happen immediately to start the transmission */
  UART_IRQ_FLAG_SET( UART_NUMBER, UART_LOCATION, TX ); /* set the interrupt */
  
  /* enable receive interrupts, they are always welcome. */
  UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, RX ); 

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
     * this will cause the message to begin transmission */
    UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, TX ); 

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
