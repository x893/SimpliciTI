#ifndef uart_intfc_cc430_h
#define uart_intfc_cc430_h

#include <stddef.h>
#include <stdbool.h>
#include "uart_cc430.h"

// call this function before using any other functions in this interface
void uart_intfc_init( void );

// enqueue's the message pointed to by <buff> which is of length <len>
// and initiates its transfer across the uart.  true is returned if there
// was space in the fifo to send the data, false if the fifo didn't have
// enough free space to enqueue the data.
bool tx_send( const void* buff, size_t len );

// enqueue's the message bointed to by <buff> which is of length <len>
// and initiates its transfer across the uart.  this is a blocking function
// in that if the transmit fifo doesn't have enough room to enqueue the data
// in its entirety it will push the data out a piece at a time as the room in
// the fifo becomes available.  the function returns true upon completion of
// moving all the data into the fifo and false if either a null pointer or a
// length of zero was passed.
bool tx_send_wait( const void* buff, size_t len );

// returns the nubmer of bytes of free space in the output fifo.
int tx_peek( void );

// returns the number of bytes currently available in the receive queue.
int rx_peek( void );

// fills in the buffer <buff> with data from the receive queue until either
// <max_len> bytes have been transferred into <buff> or the receive queue
// is emptied.  the actual number of bytes put into <buff> is returned
int rx_receive( void* buff, int max_len );

// returns true if there are characters in the receive buffer or transmit buffer
// false if both buffers are empty.
bool uart_busy( void );

#endif
