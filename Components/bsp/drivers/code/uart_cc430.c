#include "uart_cc430.h"

// local variables
static uart_get_tx_data_type uart_tx_handler = NULL;
static uart_put_rx_data_type uart_rx_handler = NULL;
static bool uart_tx_suspend = false;

// function prototypes
static void uart_tx_irq( void );
static void uart_rx_irq( void );

#pragma vector=USCI_A0_VECTOR
__interrupt void uart_rx_tx_enter_irq( void );
#pragma vector=USCI_A0_VECTOR
__interrupt void uart_rx_tx_enter_irq( void )
{
  unsigned int v;

  // while there are interrupts to service
  for( v = UCA0IV; v != USCI_NONE; v = UCA0IV )
    switch( v )
    {
    case USCI_UCRXIFG: // if a receive interrupt
      uart_rx_irq( ); // handle the receive request
      break;
    
    case USCI_UCTXIFG: // if a transmit interrupt
      uart_tx_irq( ); // handle the transmit request
      break;
      
    default: // something not quite right
      asm( " NOP" ); // a place for a break point
      break;
    }

  return;
}

// the isr's...
static void uart_tx_irq( void )
  {    
  unsigned char c;
    {
    uart_get_tx_data_type handler;
    BSP_CRITICAL_STATEMENT( handler = uart_tx_handler );

    // if not currently in suspend mode and a handler exists
    if( uart_tx_suspend == false && handler != NULL )
      {
      if( (*handler)( &c ) != false ) // if data available, reset the interrupt
        {
        UART_SEND( UART_NUMBER, UART_LOCATION, c ); // send the byte
        }
      else // if no data suspend transmission
        {
        uart_tx_message_suspend( handler );
        UART_IRQ_FLAG_SET( UART_NUMBER, UART_LOCATION, TX );
        }
      }
    else
      {
      bspIState_t istate;
      BSP_ENTER_CRITICAL_SECTION( istate );
      // if we are in suspended mode or we just sent an xon or xoff character
      // while transmits were disabled or the message has been completely sent,
      // then simply disable irq's so we don't get stuck in an infinite loop
      UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX );
      UART_IRQ_FLAG_SET( UART_NUMBER, UART_LOCATION, TX );
      BSP_EXIT_CRITICAL_SECTION( istate );
      }
    }
  return;
}

static void uart_rx_irq( void )
  {
  uart_put_rx_data_type handler;

  // read in the received data, this will clear the interrupt also
  unsigned char c = UART_RECEIVE( UART_NUMBER, UART_LOCATION );

  BSP_CRITICAL_STATEMENT( handler = uart_rx_handler );
  if( handler != NULL ) // if a handler exists to receive data
    if( ( *handler)( c ) == false ) // if the user is suspending
      uart_rx_message_suspend( handler );
    
  return;
  }

// user interface functions
void uart_init( void )
  {
  volatile unsigned int i;
  bspIState_t istate;
  BSP_ENTER_CRITICAL_SECTION( istate );

  // disable the receive and transmit interrupts for the moment
  UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, TX ); 
  UART_IRQ_DISABLE( UART_NUMBER, UART_LOCATION, RX ); 

  BSP_EXIT_CRITICAL_SECTION( istate );

  // make sure the handler functions are cleared in case we are re-initialized
  uart_tx_handler = NULL;
  uart_rx_handler = NULL;

  // clear transmit suspend semaphore
  uart_tx_suspend = false;
  
  // initialize the uart interface for operations
  UART_INIT( UART_NUMBER,
             UART_LOCATION,
             UART_FLOW_CONTROL,    // enable/disable flow control
             UART_PARITY_MODE,     // enable/disable parity
             UART_STOP_BITS,       // number of stop bits
             UART_BAUD_RATE );     // baud rate to use

  i = UART_BAUD_RATE >> 5; // delay approximately 1 bit time
  while( --i != 0 ) // give the uart some time to initialize
      ; // null statement

  // enable receive interrupts, they are always welcome.
  UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, RX ); 

  return;
  }

bool uart_tx_message( uart_get_tx_data_type handler )
  {
  bspIState_t  intState;
  bool status = false; // assume failure initially

  // updates required, store interrupt state and disable interrupts
  BSP_ENTER_CRITICAL_SECTION(intState);

  // if no message is currently being sent and handler looks valid
  if( uart_tx_handler == NULL && handler != NULL )
    {
    uart_tx_handler = handler; // install the handler

    // once the handler has been setup, enable the interrupt.
    // this will cause the message to begin transmission
    UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, TX ); 

    status = true; // indicate success    
    }

  BSP_EXIT_CRITICAL_SECTION(intState); // restore interrupt state
    
  return status; // indicate status
  }

bool uart_tx_message_end( uart_get_tx_data_type handler )
  {
  bspIState_t intState;
  bool status = false; // assume failure initially

  BSP_ENTER_CRITICAL_SECTION( intState );

  if( uart_tx_handler == handler )
    {
    // no more data to send, reset the handler to flag not busy
    uart_tx_handler = NULL;
    uart_tx_suspend = false;
    status = true; // indicate success
    }
    
  BSP_EXIT_CRITICAL_SECTION( intState );
  
  return status; // indicate status
  }

bool uart_tx_message_suspend( uart_get_tx_data_type handler )
  {
  bool status = false; // assume failure initially
  bspIState_t intState;
  BSP_ENTER_CRITICAL_SECTION( intState );
  
  if( uart_tx_handler == handler )
    {
    uart_tx_suspend = true; // indicate we are in suspended status
    status = true; // indicate success
    }
  
  BSP_EXIT_CRITICAL_SECTION( intState );
  return status;
  }

bool uart_tx_message_resume( uart_get_tx_data_type handler )
  {
  bool status = false; // assume failure initially
  bspIState_t intState;
  BSP_ENTER_CRITICAL_SECTION( intState );
  
  if( uart_tx_handler == handler )
    {
    uart_tx_suspend = false; // indicate we are no longer suspended
    UART_IRQ_ENABLE( UART_NUMBER, UART_LOCATION, TX ); // enable interrupt
    status = true; // indicate success
    }
  
  BSP_EXIT_CRITICAL_SECTION( intState );
  return status;
  }

bool uart_rx_message( uart_put_rx_data_type handler )
  {
  bspIState_t intState;
  bool status = false;  // assume failure initially
  
  // updates required, store interrupt state and disable interrupts
  BSP_ENTER_CRITICAL_SECTION(intState);

  // if no message is being received and the handler looks valid
  if( uart_rx_handler == NULL && handler != NULL )
    {
    uart_rx_handler = handler; // install the handler

    status = true; // indicate success
    }
  
  BSP_EXIT_CRITICAL_SECTION(intState); // restore interrupt state
    
  return status; // indicate status
  }

bool uart_rx_message_end( uart_put_rx_data_type handler )
  {
  bspIState_t intState;
  bool status = false; // assume failure initially
    
  BSP_ENTER_CRITICAL_SECTION(intState);
    
  // if it appears that the current receiver client is terminating the message
  if( uart_rx_handler == handler )
    {
    // clear the handler to indicate the driver is free for acquisition
    uart_rx_handler = NULL;

    status = true;
    }
   
  BSP_EXIT_CRITICAL_SECTION(intState);

  return status; // return result
  }

bool uart_rx_message_suspend( uart_put_rx_data_type handler )
  {
  return false; // cannot suspend yet
  }

bool uart_rx_message_resume( uart_put_rx_data_type handler )
  {
  return true; // always receiving for now
  }
