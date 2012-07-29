/**************************************************************************************************
  Filename:       uart_intfc.h
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This file supports the SimpliciTI-compatible UART API functions.

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

#ifndef uart_intfc_h
#define uart_intfc_h

#include <stddef.h>
#include "uart.h"

/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
/* call this function before using any other functions in this interface */
void uart_intfc_init( void );

/* enqueue's the message pointed to by <data> which is of length <len>
 * and initiates its transfer across the uart.  true is returned if there
 * was space in the fifo to send the data, false if the fifo didn't have
 * enough free space to enqueue the data. */
bool tx_send( const void* data, size_t len );

/* enqueue's the message bointed to by <data> which is of length <len>
 * and initiates its transfer across the uart.  this is a blocking function
 * in that if the transmit fifo doesn't have enough room to enqueue the data
 * in its entirety it will push the data out a piece at a time as the room in
 * the fifo becomes available.  the function returns true upon completion of
 * moving all the data into the fifo and false if either a null pointer or a
 * length of zero was passed. */
bool tx_send_wait( const void* data, size_t len );

/* returns the nubmer of bytes of free space in the output fifo. */
int tx_peek( void );

/* returns the number of bytes currently available in the receive queue. */
int rx_peek( void );

/* fills in the buffer <data> with data from the receive queue until either
 * <max_len> bytes have been transferred into <buff> or the receive queue
 * is emptied.  the actual number of bytes put into <buff> is returned */
int rx_receive( void* data, int max_len );

/* returns true if there are characters in the receive buffer or transmit buffer
 * false if both buffers are empty. */
bool uart_busy( void );

#endif
