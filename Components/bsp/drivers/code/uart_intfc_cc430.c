#include <stdlib.h>
#include "uart_intfc_cc430.h"
#ifdef FREQUENCY_HOPPING
#include "nwk_pll.h"
#endif

// this value must be at least 2.
#ifndef RX_TX_BUFFER_SIZE
  #define RX_TX_BUFFER_SIZE 500
#endif

#if RX_TX_BUFFER_SIZE < 2
  #error "ERROR:  The value of the macro RX_TX_BUFFER_SIZE must be at least 2."
#endif

#define UART_MINIMUM_BUFFER_MARGIN ( ( RX_TX_BUFFER_SIZE > 32 ) \
                                     ? 8 : ( ( RX_TX_BUFFER_SIZE > 16 ) \
                                           ? ( RX_TX_BUFFER_SIZE + 1 ) / 2 \
                                           : RX_TX_BUFFER_SIZE - 1 ) )

static unsigned char rx_buff[RX_TX_BUFFER_SIZE];
static unsigned char tx_buff[RX_TX_BUFFER_SIZE];
static unsigned char* rx_head = rx_buff;
static unsigned char* rx_tail = rx_buff;
static unsigned char* tx_head = tx_buff;
static unsigned char* tx_tail = tx_buff;

// uart data handler function prototypes
bool tx_handler( unsigned char* c );
bool rx_handler( unsigned char c );

static int buffer_free_space( unsigned char* head, unsigned char* tail );
static int buffer_used_space( unsigned char* head, unsigned char* tail );
static bool push_buffer( unsigned char** head, unsigned char* tail,
                         unsigned char* buff, unsigned char* data, int len );
static int pop_buffer( unsigned char* head, unsigned char** tail,
                       unsigned char* buff, unsigned char* data, int max_len );

void uart_intfc_init( void )
  {
  bspIState_t istate;

  uart_init( ); // initialize the uart for operations
  
  BSP_ENTER_CRITICAL_SECTION( istate );

  // initialize the buffer pointers in case we are re-initialized
  rx_head = rx_buff;
  rx_tail = rx_buff;
  tx_head = tx_buff;
  tx_tail = tx_buff;

  BSP_EXIT_CRITICAL_SECTION( istate );

  uart_rx_message( rx_handler ); // enable us to receive uart data
  uart_tx_message( tx_handler ); // enable us to transmit uart data

  return;
  }

int buffer_free_space( unsigned char* head, unsigned char* tail )
  {
  // the free count is the buffer size minus the used count minus one
  // because we don't want the pointers to ever get back on top of each other
  // because that would indicate an empty buffer.
  return RX_TX_BUFFER_SIZE - buffer_used_space( head, tail ) - 1;
  }

int buffer_used_space( unsigned char* head, unsigned char* tail )
  {
  ptrdiff_t used;
  
  used = head - tail; // get used count
  if( used < 0 ) // if the pointers were wrapped
    used += RX_TX_BUFFER_SIZE; // correct the count
  
  return used; // return used count
  }

// assumes on entry that data, buff, tail, and head are all valid pointers
bool push_buffer( unsigned char** head, unsigned char* tail,
                  unsigned char* buff, unsigned char* data, int len )
  {
  unsigned char* local_head;

  BSP_CRITICAL_STATEMENT( local_head = *head );
  
  // if no room in the buffer
  if( buffer_free_space( local_head, tail ) < len )
    return false; // indicate failure to enqueue message
  
  // there is room for the data, put in the buffer
  
  do // put the data in the buffer
    {
    if( local_head == buff + RX_TX_BUFFER_SIZE ) // if wrapping around
      local_head = buff; // reset pointer
    
    *local_head++ = *data++; // copy over this byte of data
    } while( --len > 0 ); // copy all the data to the buffer
  
  BSP_CRITICAL_STATEMENT( *head = local_head ); // update reference value
  
  return true;
  }

// assumes on entry that data, buff, tail, and head are all valid pointers
int pop_buffer( unsigned char* head, unsigned char** tail,
                unsigned char* buff, unsigned char* data, int max_len )
  {
  unsigned char* local_tail;
  int cnt = 0;

  BSP_CRITICAL_STATEMENT( local_tail = *tail );
  
  // if the buffer is empty or no data requested
  if( local_tail == head || max_len <= 0 )
    return 0; // indicate so
  
  do // retrieve the data from the buffer
    {
    if( local_tail == buff + RX_TX_BUFFER_SIZE ) // if wrapping around
      local_tail = buff; // reset pointer
    
    *data++ = *local_tail++; // copy data from buffer
      
    // while the user needs more data and there is data left in the fifo
    } while( ++cnt < max_len && local_tail != head );
    
  BSP_CRITICAL_STATEMENT( *tail = local_tail ); // update reference value
  
  return cnt; // return number of characters retrieved from the buffer
  }

bool tx_send_wait( const void* data, size_t len )
  {
  if( len > 0 && data != NULL ) // if the information looks viable
    {
    while( len > 0 ) // while there is data left to transfer
      {
      int sz = tx_peek( ); // get free space in the fifo

      if( sz > 0 ) // if there is room for at least some data
        {
        if( sz > len ) // if more room than required
          sz = len; // limit size to just the data to send

        tx_send( data, sz ); // send this portion of the information

        data = (unsigned char*)data + sz; // move the pointer

        len -= sz; // adjust the count of remaining data to send
        }
      
      #ifdef FREQUENCY_HOPPING
      // run the pll charge pump if frequency hopping active
      //   only send pump requests if there are still characters still to send
      nwk_pllBackgrounder( len == 0 );
      #endif
      }
    
    return true; // indicate success
    }
  
  return false; // otherwise indicate failure
  }

bool tx_send( const void* data, size_t len )
  {
  bool status;
  unsigned char* tail;
  
  // get current state of tail pointer
  BSP_CRITICAL_STATEMENT( tail = tx_tail );

  // put data into transmit buffer
  status = push_buffer( &tx_head, tail, tx_buff, (unsigned char*)data, len );

  if( status != false ) // if data was put in the buffer properly
    uart_tx_message_resume( tx_handler ); // notify the irq that data is ready to send
  
  return status; // return status
  }

int tx_peek( void )
  {
  unsigned char* head;
  unsigned char* tail;
  
  BSP_CRITICAL_STATEMENT( head = tx_head; tail = tx_tail );
  
  return buffer_free_space( head, tail );
  }

int rx_peek( void )
  {
  unsigned char* head;
  unsigned char* tail;
  
  BSP_CRITICAL_STATEMENT( head = rx_head; tail = rx_tail );
  
  return buffer_used_space( head, tail );
  }

int rx_receive( void* data, int max_len )
  {
  int cnt;
  unsigned char* head;
  unsigned char* tail;
  
  // get current state of head pointer
  BSP_CRITICAL_STATEMENT( head = rx_head );
  
  // retrieve data from buffer
  cnt = pop_buffer( head, &rx_tail, rx_buff, data, max_len );

  // get current state of tail pointer
  BSP_CRITICAL_STATEMENT( tail = rx_tail );

  if( buffer_free_space( head, tail ) >= UART_MINIMUM_BUFFER_MARGIN )
    uart_rx_message_resume( rx_handler ); // turn reception back on if buffer has room

  return cnt; // indicate the number of bytes retrieved from the buffer
  }

bool uart_busy( void )
  {
  int cnt;
  unsigned char* head;
  unsigned char* tail;
  
  // get receive buffer count
  BSP_CRITICAL_STATEMENT( head = rx_head; tail = rx_tail );
  cnt = buffer_used_space( head, tail );
  
  // get transmit buffer count
  BSP_CRITICAL_STATEMENT( head = tx_head; tail = tx_tail );
  cnt += buffer_used_space( head, tail );
  
  return ( ( cnt == 0 ) ? false : true ); // return status of uart
  }

bool tx_handler( unsigned char* c )
  {
  int cnt;
  unsigned char* head;

  // get current state of head pointer
  BSP_CRITICAL_STATEMENT( head = tx_head );

  // get the next character from the buffer
  cnt = pop_buffer( head, &tx_tail, tx_buff, c, 1 );
  
  return ( cnt != 0 ) ? true : false; // return status
  }

bool rx_handler( unsigned char c )
  {
  unsigned char* tail;
  unsigned char* head;

  // get current state of tail pointer
  BSP_CRITICAL_STATEMENT( tail = rx_tail );

  // put data onto the receive buffer
  push_buffer( &rx_head, tail, rx_buff, &c, 1 );
  
  // get current state of head pointer
  BSP_CRITICAL_STATEMENT( head = rx_head );

  if( buffer_free_space( head, tail ) < UART_MINIMUM_BUFFER_MARGIN )
    uart_rx_message_suspend( rx_handler ); // halt reception if buffer is full

  return true; // always accept data received from the uart
  }
