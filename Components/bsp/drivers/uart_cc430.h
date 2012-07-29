#ifndef uart_cc430_h
#define uart_cc430_h

#include <stdbool.h> // supports bool, true, and false
#include <stddef.h>  // supports NULL macro

#include "bsp.h"

////////////////////////////////////////////////////////////////////////////////
// clock definition
// this allows for the user to modify the clock rate used in subsequent
// calculations by defining a different value at the project level.
#ifndef UART_CLOCK_MHZ
  #define UART_CLOCK_MHZ BSP_CLOCK_MHZ
#endif

// constants of operation
#define UART_IRQ_ENABLED   1
#define UART_IRQ_DISABLED  0
#define UART_IRQ_PENDING   1
#define UART_IRQ_IDLE      0

// uart hardware flow control options
#define UART_FLOW_CONTROL_OFF      0  // no flow control
#define UART_FLOW_CONTROL_HARDWARE 1  // hardware flow control
#define UART_FLOW_CONTROL_XON_XOFF 2  // software flow control

// uart parity enabled status
#define UART_PARITY_NONE  0
#define UART_PARITY_EVEN  1
#define UART_PARITY_ODD   2
#define UART_PARITY_MARK  3 // not supported for cc430
#define UART_PARITY_SPACE 4 // not supported for cc430

// uart number of stops bits to use
#define UART_1_STOP_BIT   0
#define UART_2_STOP_BITS  1

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// common target definitions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// set default values for any not defined by the user
#define UART_NUMBER          UART_LETTER	
#define UART_LOCATION        UART_MODULE

#define UART_LETTER          A
#define UART_MODULE          0
#define UART_NUMBER          UART_LETTER	
#define UART_LOCATION        UART_MODULE

#ifndef UART_BAUD_RATE
  #define UART_BAUD_RATE 9600
#endif
#ifndef UART_FLOW_CONTROL
  #define UART_FLOW_CONTROL  UART_FLOW_CONTROL_XON_XOFF
#endif
#ifndef UART_PARITY_MODE
  #define UART_PARITY_MODE   UART_PARITY_NONE
#endif
#ifndef UART_STOP_BITS
  #define UART_STOP_BITS     UART_1_STOP_BIT
#endif
#ifndef UART_XON_CHARACTER
  #define UART_XON_CHARACTER 17   // ascii definition of xon
#endif
#ifndef UART_XOFF_CHARACTER
  #define UART_XOFF_CHARACTER 19  // ascii definition of xoff
#endif




#include "pp_utils.h"
#include "msp430.h"

#if UART_FLOW_CONTROL == UART_FLOW_CONTROL_HARDWARE
  #define UART_HARDWARE_HANDSHAKE_IN_SOFTWARE
#endif

////////////////////////////////////////////////////////////////////////////////
// port pin support definitions
#define IO_PORT_REGISTER_BIT_SET( port, bit, reg, state, hi, lo )              \
                              ( ( state == lo )                                \
                                  ? ( INFIX( P, port, reg ) &= ~( 1 << bit ) ) \
                                  : ( INFIX( P, port, reg ) |= ( 1 << bit ) ) )
#define IO_PORT_REGISTER_BIT_GET( port, bit, reg, hi, lo )                     \
                 ( ( ( INFIX( P, port, reg ) & ( 1 << bit ) ) == 0 ) ? lo : hi )
  
#define IO_PORT_INPUT_LO 0
#define IO_PORT_INPUT_HI 1
#define IO_PORT_GET_INPUT( port, bit )                                         \
   IO_PORT_REGISTER_BIT_GET( port, bit, IN, IO_PORT_INPUT_HI, IO_PORT_INPUT_LO )

#define IO_PORT_OUTPUT_LO 0
#define IO_PORT_OUTPUT_HI 1
// provide redirection since OUT is defined as a macro elswhere
#define P0_IO_OUT P0OUT
#define P1_IO_OUT P1OUT
#define P2_IO_OUT P2OUT
#define P3_IO_OUT P3OUT
#define P4_IO_OUT P4OUT
#define P5_IO_OUT P5OUT
#define P6_IO_OUT P6OUT
#define P7_IO_OUT P7OUT
#define P8_IO_OUT P8OUT
#define IO_PORT_SET_OUTPUT( port, bit, state )                                 \
                          IO_PORT_REGISTER_BIT_SET( port, bit, _IO_OUT, state, \
                                          IO_PORT_OUTPUT_HI, IO_PORT_OUTPUT_LO )
#define IO_PORT_GET_OUTPUT( port, bit )                                        \
                                 IO_PORT_REGISTER_BIT_GET( port, bit, _IO_OUT, \
                                          IO_PORT_OUTPUT_HI, IO_PORT_OUTPUT_LO )

#define IO_PORT_DIRECTION_INPUT  0
#define IO_PORT_DIRECTION_OUTPUT 1
#define IO_PORT_SET_DIRECTION( port, bit, dir )                                \
                           IO_PORT_REGISTER_BIT_SET( port, bit, DIR, dir,      \
                             IO_PORT_DIRECTION_OUTPUT, IO_PORT_DIRECTION_INPUT )
#define IO_PORT_GET_DIRECTION( port, bit )                                     \
                           IO_PORT_REGISTER_BIT_GET( port, bit, DIR,           \
                             IO_PORT_DIRECTION_OUTPUT, IO_PORT_DIRECTION_INPUT )

#define IO_PORT_RESISTOR_DISABLED 0
#define IO_PORT_RESISTOR_ENABLED  1
#define IO_PORT_SET_RESISTOR( port, bit, mode )                                \
                         IO_PORT_REGISTER_BIT_SET( port, bit, REN, mode,       \
                           IO_PORT_RESISTOR_ENABLED, IO_PORT_RESISTOR_DISABLED )
#define IO_PORT_GET_RESISTOR( port, bit )                                      \
                         IO_PORT_REGISTER_BIT_GET( port, bit, REN,             \
                           IO_PORT_RESISTOR_ENABLED, IO_PORT_RESISTOR_DISABLED )

#define IO_PORT_SELECT_IO         0
#define IO_PORT_SELECT_PERIPHERAL 1
#define IO_PORT_SET_SELECT( port, bit, mode )                                  \
                               IO_PORT_REGISTER_BIT_SET( port, bit, SEL, mode, \
                                  IO_PORT_SELECT_PERIPHERAL, IO_PORT_SELECT_IO )
#define IO_PORT_GET_SELECT( port, bit )                                        \
                               IO_PORT_REGISTER_BIT_GET( port, bit, SEL,       \
                                  IO_PORT_SELECT_PERIPHERAL, IO_PORT_SELECT_IO )

#define IO_PORT_INTERRUPT_DISABLED 0
#define IO_PORT_INTERRUPT_ENABLED  1
#define IO_PORT_SET_INTERRUPT_ENABLED_STATE( port, bit, mode )                 \
                       IO_PORT_REGISTER_BIT_SET( port, bit, IE, mode,          \
                         IO_PORT_INTERRUPT_ENABLED, IO_PORT_INTERRUPT_DISABLED )
#define IO_PORT_GET_INTERRUPT_ENABLED_STATE( port, bit )                       \
                       IO_PORT_REGISTER_BIT_GET( port, bit, IE,                \
                         IO_PORT_INTERRUPT_ENABLED, IO_PORT_INTERRUPT_DISABLED )

#define IO_PORT_INTERRUPT_RISING_EDGE  0
#define IO_PORT_INTERRUPT_FALLING_EDGE 1
#define IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode )                           \
                IO_PORT_REGISTER_BIT_SET( port, bit, IES, mode,                 \
                  IO_PORT_INTERRUPT_FALLING_EDGE, IO_PORT_INTERRUPT_RISING_EDGE )
#define IO_PORT_GET_INTERRUPT_EDGE( port, bit )                                 \
                IO_PORT_REGISTER_BIT_GET( port, bit, IES,                       \
                  IO_PORT_INTERRUPT_FALLING_EDGE, IO_PORT_INTERRUPT_RISING_EDGE )
      
#define IO_PORT_INTERRUPT_IDLE    0
#define IO_PORT_INTERRUPT_PENDING 1
#define IO_PORT_SET_INTERRUPT_STATE( port, bit, mode )                         \
                           IO_PORT_REGISTER_BIT_SET( port, bit, IFG, mode,     \
                             IO_PORT_INTERRUPT_PENDING, IO_PORT_INTERRUPT_IDLE )
#define IO_PORT_GET_INTERRUPT_STATE( port, bit )                               \
                           IO_PORT_REGISTER_BIT_GET( port, bit, IFG,           \
                             IO_PORT_INTERRUPT_PENDING, IO_PORT_INTERRUPT_IDLE )

// the following macro enables an interrupt only after making sure the interrupt
// flag is cleared first
#define IO_PORT_SAFE_ENABLE_INTERRUPT( port, bit )                             \
 ( IO_PORT_SET_INTERRUPT_STATE( port, bit, IO_PORT_INTERRUPT_IDLE ),           \
   IO_PORT_SET_INTERRUPT_ENABLED_STATE( port, bit, IO_PORT_INTERRUPT_ENABLED ) )

// the following macro changes the sense of the edge that causes an interrupt
// in a manner that guarantees no interrupt is generated.
#define IO_PORT_SAFE_SET_INTERRUPT_EDGE( port, bit, mode )                     \
                  ( ( IO_PORT_GET_INTERRUPT_STATE( port, bit )                 \
                        == IO_PORT_INTERRUPT_DISABLED )                        \
                           ? IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode )     \
                           : ( IO_PORT_SET_INTERRUPT_STATE( port, bit,         \
                                                 IO_PORT_INTERRUPT_DISABLED ), \
                               IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode ),  \
                               IO_PORT_SAFE_ENABLE_INTERRUPT( port, bit ) ) )
      
#ifndef UART_TX_PORT_NUM
  #define UART_TX_PORT_NUM 1
#endif
#ifndef UART_TX_BIT_NUM
  #define UART_TX_BIT_NUM 6
#endif
#ifndef UART_RX_PORT_NUM
  #define UART_RX_PORT_NUM 1
#endif
#ifndef UART_RX_BIT_NUM
  #define UART_RX_BIT_NUM 5
#endif

#define UART_RTS_ASSERTED   IO_PORT_OUTPUT_HI
#define UART_RTS_DEASSERTED IO_PORT_OUTPUT_LO
#define UART_ASSERT_RTS( state ) IO_PORT_SET_OUTPUT( UART_RTS_PORT_NUM, UART_RTS_BIT_NUM, state )
      
#define UART_CTS_DEASSERTED IO_PORT_OUTPUT_LO
#define UART_CTS_CLEAR_INTERRUPT( )                                                        \
  IO_PORT_SET_INTERRUPT_STATE( UART_CTS_PORT_NUM, UART_CTS_BIT_NUM, IO_PORT_INTERRUPT_IDLE ) 
#define UART_CTS_GET_STATE( ) IO_PORT_GET_INPUT( UART_CTS_PORT_NUM, UART_CTS_BIT_NUM )
#define UART_CTS_ASSERTION_EDGE ( ( UART_CTS_DEASSERTED == IO_PORT_OUTPUT_LO ) \
                                              ? IO_PORT_INTERRUPT_RISING_EDGE  \
                                              : IO_PORT_INTERRUPT_FALLING_EDGE )

#define UART_CTS_RTS_INIT( rts_port, rts_bit, cts_port, cts_bit ) \
                                             ( (void)0 ) // do nothing
      
////////////////////////////////////////////////////////////////////////////////
// uart identification
#define UART_ID( num, loc ) INFIX( UART_ID_, num, loc )
#define UART_ID_A0    0x4130 // 'A' in hex followed by '0' in hex
#define UART_ID_A1    0x4131 // 'A' in hex followed by '1' in hex
//#define UART_ID_B0    0x4230 // 'B' in hex followed by '0' in hex // not allowed on cc430
//#define UART_ID_B1    0x4231 // 'B' in hex followed by '1' in hex // not allowed on cc430

////////////////////////////////////////////////////////////////////////////////
// uart register resolution
#define UCxxCTL0( num, loc ) CONCATENATE( INFIX( UC, num, loc ), CTL0 )
#define UCxxCTL1( num, loc ) CONCATENATE( INFIX( UC, num, loc ), CTL1 )
#define UCxxBR0( num, loc )  CONCATENATE( INFIX( UC, num, loc ), BR0 )
#define UCxxBR1( num, loc )  CONCATENATE( INFIX( UC, num, loc ), BR1 )
#define UCxxMCTL( num, loc ) CONCATENATE( INFIX( UC, num, loc ), MCTL )

////////////////////////////////////////////////////////////////////////////////
// parity operation
#define UART_PARITY_ENABLE( parity ) ( ( ( parity ) == UART_PARITY_NONE ) ? 0 : ( UCPEN ) )
#define UART_PARITY_FORM( parity )   ( ( ( parity ) == UART_PARITY_EVEN ) ? ( UCPAR ) : 0 )
#define UART_PARITY_SET( num, loc, parity )   ( UCxxCTL0( num, loc )            \
                                             = ( UCxxCTL0( num, loc )           \
                                                 & ~( ( UCPEN ) | ( UCPAR ) ) ) \
                                               | UART_PARITY_ENABLE( parity )   \
                                               | UART_PARITY_FORM( parity ) )
      
////////////////////////////////////////////////////////////////////////////////
// data byte ordering
#define UART_LSB_FIRST 0
#define UART_MSB_FIRST 1
#define UART_BYTE_ORDER_SET( num, loc, mode )                               \
                                 ( ( ( mode ) == UART_LSB_FIRST )           \
                                   ? ( UCxxCTL0( num, loc ) &= ~( UCMSB ) ) \
                                   : ( UCxxCTL0( num, loc ) |= ( UCMSB ) ) )

////////////////////////////////////////////////////////////////////////////////
// data length
#define UART_DATA_LENGTH_7_BITS 1
#define UART_DATA_LENGTH_8_BITS 0
#define UART_DATA_LENGTH_SET( num, loc, mode )                                \
                                  ( ( ( mode ) == UART_DATA_LENGTH_7_BITS )   \
                                    ? ( UCxxCTL0( num, loc ) |= ( UC7BIT ) )  \
                                    : ( UCxxCTL0( num, loc ) &= ~( UC7BIT ) ) )

////////////////////////////////////////////////////////////////////////////////
// stop bits
#define UART_STOP_BITS_SET( num, loc, bits )                         \
                          ( ( ( bits ) == UART_1_STOP_BIT ) ?        \
                              ( UCxxCTL0( num, loc ) &= ~( UCSPB ) ) \
                            : ( UCxxCTL0( num, loc ) |= ( UCSPB ) ) )

////////////////////////////////////////////////////////////////////////////////
// uart mode of operation
#define UART_MODE_UART      0
#define UART_MODE_IDLE_LINE 1
#define UART_MODE_ADDRESSED 2
#define UART_MODE_AUTO_BAUD 3
#define UART_MODE_SET( num, loc, mode ) ( UCxxCTL0( num, loc )                 \
       = ( ( UCxxCTL0( num, loc ) & ~( UCMODE_3 ) )                            \
         | ( ( ( ( mode ) & UART_MODE_ADDRESSED ) == 0 ) ? 0 : ( UCMODE1 ) )   \
         | ( ( ( ( mode ) & UART_MODE_IDLE_LINE ) == 0 ) ? 0 : ( UCMODE0 ) ) ) )

////////////////////////////////////////////////////////////////////////////////
// synchronous/asynchronous operation
#define UART_SYNC_ASYNC 0
#define UART_SYNC_SYNC  1
#define UART_SYNC_SET( num, loc, mode )                                 \
                            ( ( ( mode ) == UART_SYNC_ASYNC )           \
                              ? ( UCxxCTL0( num, loc ) &= ~( UCSYNC ) ) \
                              : ( UCxxCTL0( num, loc ) |= ( UCSYNC ) ) )

////////////////////////////////////////////////////////////////////////////////
// uart clock source select
#define UART_CLOCK_UCLK  0
#define UART_CLOCK_ACLK  1
#define UART_CLOCK_SMCLK 2
#define UART_CLOCK_SET( num, loc, src ) ( UCxxCTL1( num, loc )                 \
           = ( ( UCxxCTL1( num, loc ) & ~( UCSSEL1 | UCSSEL0 ) )               \
             | ( ( ( ( src ) & UART_CLOCK_SMCLK ) == 0 ) ? 0 : ( UCSSEL1 ) )   \
             | ( ( ( ( src ) & UART_CLOCK_ACLK  ) == 0 ) ? 0 : ( UCSSEL0 ) ) ) )

////////////////////////////////////////////////////////////////////////////////
// enable/reset uart
#define UART_RESET( num, loc )  ( UCxxCTL1( num, loc ) |= UCSWRST )
#define UART_ENABLE( num, loc ) ( UCxxCTL1( num, loc ) &= ~UCSWRST )

////////////////////////////////////////////////////////////////////////////////
// baud rate calculation
#define UART_FLOOR( n, b )           ( ( n ) >> ( b ) )
#define UART_ROUND( n, b )           ( ( ( ( n ) >> ( ( b ) - 1 ) ) + 1 ) >> 1 )
// using q27.5 to represent the N value in the documentation calculations
#define UART_N_Q27_5( baud )         ( ( ( UART_CLOCK_MHZ ) * 32 * 1000000ul ) / ( baud ) )
#define UART_UCOS16( baud )          ( ( UART_FLOOR( UART_N_Q27_5( baud ), 5 ) >= 16 ) ? 1 : 0 )
#define UART_LF_UCBR( baud )         UART_FLOOR( UART_ROUND( UART_N_Q27_5( baud ), 2 ), 3 )
#define UART_LF_UCBRS( baud )        ( UART_ROUND( UART_N_Q27_5( baud ), 2 ) % 8 )
#define UART_LF_UCBRF( baud )        0
#define UART_OS_UCBR( baud )         UART_FLOOR( UART_ROUND( UART_N_Q27_5( baud ) / 16, 1 ), 4 )
#define UART_OS_UCBRF( baud )        ( UART_ROUND( UART_N_Q27_5( baud ) / 16, 1 ) % 16 )
#define UART_OS_UCBRS( baud )        0
#define UART_UCBR( num, loc, baud )  ( ( UART_UCOS16( baud ) == 0 ) ? UART_LF_UCBR( baud )  : UART_OS_UCBR( baud ) )
#define UART_UCBRS( num, loc, baud ) ( ( UART_UCOS16( baud ) == 0 ) ? UART_LF_UCBRS( baud ) : UART_OS_UCBRS( baud ) )
#define UART_UCBRF( num, loc, baud ) ( ( UART_UCOS16( baud ) == 0 ) ? UART_LF_UCBRF( baud ) : UART_OS_UCBRF( baud ) )
      

// this setup macro assumes 8 bit data lsb first to follow standard RS232 interface protocols      
#define UART_INIT_MSP430( num, loc, flow, parity, stop, baud )                                      \
  ( UART_RESET( num, loc ),  /* reset the uart */                                                  \
    IO_PORT_SET_SELECT( UART_TX_PORT_NUM, UART_TX_BIT_NUM, IO_PORT_SELECT_PERIPHERAL ),            \
    IO_PORT_SET_SELECT( UART_RX_PORT_NUM, UART_RX_BIT_NUM, IO_PORT_SELECT_PERIPHERAL ),            \
    UART_CTS_RTS_INIT( UART_RTS_PORT_NUM, UART_RTS_BIT_NUM, UART_CTS_PORT_NUM, UART_CTS_BIT_NUM ), \
    UART_PARITY_SET( num, loc, parity ),                                                           \
    UART_BYTE_ORDER_SET( num, loc, UART_LSB_FIRST ),                                               \
    UART_DATA_LENGTH_SET( num, loc, UART_DATA_LENGTH_8_BITS ),                                     \
    UART_STOP_BITS_SET( num, loc, stop ),                                                          \
    UART_MODE_SET( num, loc, UART_MODE_UART ),                                                     \
    UART_SYNC_SET( num, loc, UART_SYNC_ASYNC ),                                                    \
    UART_CLOCK_SET( num, loc, UART_CLOCK_SMCLK ),                                                  \
    UCxxBR0( num, loc ) = ( UART_UCBR( num, loc, baud ) & 0xFF ), /* set baud rate */              \
    UCxxBR1( num, loc ) = ( ( UART_UCBR( num, loc, baud ) >> 8 ) & 0xFF ), /* set baud rate */     \
    UCxxMCTL( num, loc ) = ( UCxxMCTL( num, loc ) & ~( UCBRS2 | UCBRS1 | UCBRS0 ) )                \
                           | ( ( UART_UCBRS( num, loc, baud ) & 4 ) != 0 ? ( UCBRS2 ) : 0 )        \
                           | ( ( UART_UCBRS( num, loc, baud ) & 2 ) != 0 ? ( UCBRS1 ) : 0 )        \
                           | ( ( UART_UCBRS( num, loc, baud ) & 1 ) != 0 ? ( UCBRS0 ) : 0 ),       \
    UCxxMCTL( num, loc ) = ( UCxxMCTL( num, loc ) & ~( UCBRF3 | UCBRF2 | UCBRF1 | UCBRF0 ) )       \
                           | ( ( UART_UCBRF( num, loc, baud ) & 8 ) != 0 ? ( UCBRF3 ) : 0 )        \
                           | ( ( UART_UCBRF( num, loc, baud ) & 4 ) != 0 ? ( UCBRF2 ) : 0 )        \
                           | ( ( UART_UCBRF( num, loc, baud ) & 2 ) != 0 ? ( UCBRF1 ) : 0 )        \
                           | ( ( UART_UCBRF( num, loc, baud ) & 1 ) != 0 ? ( UCBRF0 ) : 0 ),       \
    UCxxMCTL( num, loc ) |= UART_UCOS16( baud ),                                                   \
    UART_ENABLE( num, loc ) ) /* start up the uart */

#define UART_IRQ_FLAG_SET( num, loc, func )     ( CONCATENATE( INFIX( UC, num, loc ), IFG ) |= INFIX( UC, func, IFG ) )
#define UART_IRQ_ENABLE( num, loc, func )       ( CONCATENATE( INFIX( UC, num, loc ), IE ) |= INFIX( UC, func, IE ) )
#define UART_IRQ_DISABLE( num, loc, func )      ( CONCATENATE( INFIX( UC, num, loc ), IE ) &= ~INFIX( UC, func, IE ) )
      
#define UART_SEND( num, loc, c ) ( CONCATENATE( UC, INFIX( num, loc, TXBUF ) ) = c )
#define UART_RECEIVE( num, loc ) CONCATENATE( UC, INFIX( num, loc, RXBUF ) )
     
#define UART_INIT( num, loc, flow, parity, stop, baud )   \
      UART_INIT_MSP430( num, loc, flow, parity, stop, baud )
      

      
      
      
      
      
      
      
////////////////////////////////////////////////////////////////////////////////
// typedefs

// this type represents a function to call for each data character being
// transmitted across the uart.  it will be called at the character data rate.
// the user should return true if there are still more characters to send and
// false if this is the last character to send.  the character to send should
// be placed at the position pointed to by the passed parameter
typedef bool ( *uart_get_tx_data_type )( unsigned char* );

// this type represents a function to call for each data character that is
// received from the uart.  it will be called at the character data rate.
// the character recieved is passed as the parameter to the function.
// the user should return true if it is willing to accept more data, a return
// value of false indicates the user is closing this message and no longer
// wants to accept data from the uart.
typedef bool ( *uart_put_rx_data_type )( unsigned char );


////////////////////////////////////////////////////////////////////////////////
// function prototypes

// initializes the uart for operation.  this function should be called before
// any other uart functions are called.
void uart_init( void );

// attempts to begin a uart transmission.  if <handler> is NULL or there is
// currently another message being sent, false is returned.  otherwise, true
// is returned indicating the message will be sent immediately.
// the handler passed must be able to respond to character by character
// requests from the isr.  see the description of uart_get_tx_data_type above
// for more details.
bool uart_tx_message( uart_get_tx_data_type handler );

// attempts to begin a uart reception.  if <handler> is NULL or there is
// currently another message being received, false is returned.  otherwise,
// true is returned indicating the handler has been accepted for any new
// data received.  the handler passed must be able to respond to character by
// character requests from the isr.  see the description of
// uart_put_rx_data_type above for more details.
bool uart_rx_message( uart_put_rx_data_type handler );

// attempts to halt further received messages.  the handler must be the same
// handler as that passed in the uart_rx_message call that initiated a receive
// message otherwise the receiver is not halted.  returns true on successfully
// ending the current receiver.  returns false if the handler passed does not
// match the currently installed handler.
bool uart_rx_message_end( uart_put_rx_data_type handler );
bool uart_rx_message_suspend( uart_put_rx_data_type handler );
bool uart_rx_message_resume( uart_put_rx_data_type handler );

// attempts to halt further transmitted messages.  the handler must be the same
// handler as that passed in the uart_tx_message call that initiated a transmit
// message otherwise the transmission is not halted.  returns true on successfully
// ending the current transmission.  returns false if the handler passed does not
// match the currently installed handler.
bool uart_tx_message_end( uart_get_tx_data_type handler );
bool uart_tx_message_suspend( uart_get_tx_data_type handler );
bool uart_tx_message_resume( uart_get_tx_data_type handler );


#endif // uart_h
