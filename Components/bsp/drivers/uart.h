/**************************************************************************************************
  Filename:       uart.h
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This header file supports the SimpliciTI-compatible UART driver.

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

#ifndef uart_h
#define uart_h

/******************************************************************************
 * INCLUDES
 */
#include <stdbool.h> /* supports bool, true, and false */ 


#include <stddef.h>  /* supports NULL macro */

#include "bsp.h"

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/* clock definition
 * this allows for the user to modify the clock rate used in subsequent
 * calculations by defining a different value at the project level. */
#ifndef UART_CLOCK_MHZ
  #define UART_CLOCK_MHZ BSP_CLOCK_MHZ
#endif

/* constants of operation */
#define UART_IRQ_ENABLED   1
#define UART_IRQ_DISABLED  0
#define UART_IRQ_PENDING   1
#define UART_IRQ_IDLE      0

/* uart hardware flow control options */
#define UART_FLOW_CONTROL_ON  1
#define UART_FLOW_CONTROL_OFF 0

/* uart parity enabled status */
#define UART_PARITY_NONE 0
#define UART_PARITY_EVEN 1
#define UART_PARITY_ODD  2

/* uart number of stops bits to use */
#define UART_1_STOP_BIT   0
#define UART_2_STOP_BITS  1

/* common target definitions and set default values for any not defined 
 * by the user */ 
#ifndef UART_BAUD_RATE
  #define UART_BAUD_RATE 115200
#endif
#ifndef UART_FLOW_CONTROL
  #define UART_FLOW_CONTROL  UART_FLOW_CONTROL_ON
#endif
#ifndef UART_PARITY_MODE
  #define UART_PARITY_MODE   UART_PARITY_NONE
#endif
#ifndef UART_STOP_BITS
  #define UART_STOP_BITS     UART_1_STOP_BIT
#endif


/* ------------------------------------------------------------------------------------------------
 *                          MACROS AND DEFINES FOR ALL 8051 VARIANTS
 * ------------------------------------------------------------------------------------------------
 */

#if ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC8051__ )

#include MCU_H
#include "pp_utils.h"

/*  Map bit-fields in CPU registers that macros will access. This mapping has
 * been tested for CC1110/1, CC2510/1, CC2430/1, CC2530. If using an 8051-based
 * SoC apart from those, check the below mapping with the part datasheet or
 * guide */

#define SFRUNION( name, addr, ... ) __sfr __no_init volatile \
          DECL_BIT_FIELD_UNION( , , unsigned char, name, , __VA_ARGS__ ) @ addr

SFRUNION( _IEN2_,    0x9A, DECL_BIT_FIELDS_7( unsigned char, , 2, WDTIE, 1, P1IE, 1, UTX1IE, 1, UTX0IE, 1, P2IE, 1, RFIE, 1 ) );
SFRUNION( _PERCFG_,  0xF1, DECL_BIT_FIELDS_7( unsigned char, , 1, T1CFG, 1, T3CFG, 1, T4CFG, 1, , 2, U1CFG, 1, U0CFG, 1 ) );
SFRBIT(  _ADCCFG_,   0xF2, ADCCFG7, ADCCFG6, ADCCFG5, ADCCFG4, ADCCFG3, ADCCFG2, ADCCFG1, ADCCFG0 );

SFRBIT(  _P0SEL_,    0xF3, SELP0_7, SELP0_6, SELP0_5, SELP0_4, SELP0_3, SELP0_2, SELP0_1, SELP0_0 );
SFRBIT(  _P1SEL_,    0xF4, SELP1_7, SELP1_6, SELP1_5, SELP1_4, SELP1_3, SELP1_2, SELP1_1, SELP1_0 );
SFRBIT(  _P2SEL_,    0xF5, , PRI3P1, PRI2P1, PRI1P1, PRI0P1, SELP2_4, SELP2_3, SELP2_0 );
SFRBIT( _P0DIR_,     0xFD, DIRP0_7, DIRP0_6, DIRP0_5, DIRP0_4, DIRP0_3, DIRP0_2, DIRP0_1, DIRP0_0 );
SFRBIT( _P1DIR_,     0xFE, DIRP1_7, DIRP1_6, DIRP1_5, DIRP1_4, DIRP1_3, DIRP1_2, DIRP1_1, DIRP1_0 );
SFRUNION( _P2DIR_,   0xFF, DECL_BIT_FIELDS_7( unsigned char, PRIP0, 2, , 1, DIRP2_4, 1, DIRP2_3, 1, DIRP2_2, 1, DIRP2_1, 1, DIRP2_0, 1 ) );

SFRBIT( _U0CSR_   ,  0x86, U0MODE, U0RE, U0SLAVE, U0FE, U0ERR, U0RX_BYTE, U0TX_BYTE, U0ACTIVE );
SFRBIT( _U0UCR_   ,  0xC4 , U0FLUSH, U0FLOW, U0D9, U0BIT9, U0PARITY, U0SPB, U0STOP, U0START );
SFRUNION( _U0GCR_,   0xC5, DECL_BIT_FIELDS_4( unsigned char, U0CPOL, 1, U0CPHA, 1, U0ORDER, 1, U0BAUD_E, 5 ) );

SFRBIT( _U1UCR_   ,  0xFB , U1FLUSH, U1FLOW, U1D9, U1BIT9, U1PARITY, U1SPB, U1STOP, U1START );
SFRUNION( _U1GCR_,   0xFC, DECL_BIT_FIELDS_4( unsigned char, U1CPOL, 1, U1CPHA, 1, U1ORDER, 1, U1BAUD_E, 5 ) );



/*                IRQ VECTOR INFORMATION AND INTERRUPT MANAGEMENT             */ 

#define UART_IRQ_VECTOR( uart, func ) INFIX( CONCATENATE( U, func ), uart, _VECTOR )

/* irq enable/disable macros */
#define UART_IRQ_STATE( uart, loc, func )    INFIX( CONCATENATE( U, func ), uart, IE )
#define UART_IRQ_ENABLE( uart, loc, func )   ( UART_IRQ_STATE( uart, loc, func ) = UART_IRQ_ENABLED )
#define UART_IRQ_DISABLE( uart, loc, func )  ( UART_IRQ_STATE( uart, loc, func ) = UART_IRQ_DISABLED )

/* irq flag macros */
#define UART_IRQ_FLAG( uart, func )       INFIX( CONCATENATE( U, func ), uart, IF )
#define UART_IRQ_IS_PENDING( uart, func ) ( UART_IRQ_FLAG( uart, func ) == UART_IRQ_PENDING )



/*                        UART IDENTIFIER PREAMBLE BUILDERS                   */
#define UART_NUMBER_0    0
#define UART_NUMBER_1    1
#define UART_LOCATION_1  1
#define UART_LOCATION_2  2
#define UART_UART( uart )          INFIX( UART, _, uart )
#define UART_LOC( loc )            INFIX( LOC, _, loc )
#define UART_PREAMBLE( uart, loc ) INFIX( UART_UART( uart ), _, UART_LOC( loc ) )



/*                           UART PORT USAGE MACROS                           */

/* uart ==> 0 or 1, loc ==> 1 or 2, returns UART_<uart>_LOC_<loc>_PORT */
#define UART_PORT( uart, loc ) INFIX( UART_PREAMBLE( uart, loc ), _, PORT )

#define UART_0_LOC_1_PORT 0
#define UART_0_LOC_2_PORT 1
#define UART_1_LOC_1_PORT 0
#define UART_1_LOC_2_PORT 1

/*                          UART PORT PIN MACROS                              */

/* uart ==> 0 or 1, loc ==> 1 or 2, func ==> RX or TX or RT or CT */
/* returns UART_<uart>_LOC_<loc>_<func>_BIT */
#define UART_PIN( uart, loc, func ) INFIX( UART_PREAMBLE( uart, loc ), _, UART_PIN_FUNC( func ) )

/* func ==> RX or TX or RT or CT, returns <func>_BIT */
#define UART_PIN_FUNC( func ) INFIX( func, _, BIT )

#define UART_0_LOC_1_RX_BIT 2
#define UART_0_LOC_1_TX_BIT 3
#define UART_0_LOC_1_CT_BIT 4
#define UART_0_LOC_1_RT_BIT 5

#define UART_0_LOC_2_CT_BIT 2
#define UART_0_LOC_2_RT_BIT 3
#define UART_0_LOC_2_RX_BIT 4
#define UART_0_LOC_2_TX_BIT 5

#define UART_1_LOC_1_CT_BIT 2
#define UART_1_LOC_1_RT_BIT 3
#define UART_1_LOC_1_TX_BIT 4
#define UART_1_LOC_1_RX_BIT 5

#define UART_1_LOC_2_CT_BIT 4
#define UART_1_LOC_2_RT_BIT 5
#define UART_1_LOC_2_TX_BIT 6
#define UART_1_LOC_2_RX_BIT 7


/*                 UART REGISTER AND BIT FIELD ACCESS MACROS                  */

#define UART_UxCSR( uart )   INFIX( U, uart, CSR )     /* control and status register */
#define UART_MODE( uart )    INFIX( U, uart, MODE )    /* SPI or UART mode select */
#define UART_RE( uart )      INFIX( U, uart, RE )      /* receiver enable */
#define UART_FE( uart )      INFIX( U, uart, FE )      /* framing error */
#define UART_ERR( uart )     INFIX( U, uart, ERR )     /* parity error */
#define UART_RX_BYTE( uart ) INFIX( U, uart, RX_BYTE ) /* received status */
#define UART_TX_BYTE( uart ) INFIX( U, uart, TX_BYTE ) /* transmit status */
#define UART_ACTIVE( uart )  INFIX( U, uart, ACTIVE )  /* UART busy status */

#define UART_UxUCR( uart )   INFIX( U, uart, UCR )     /* contorl register */
#define UART_FLUSH( uart )   INFIX( U, uart, FLUSH )   /* flush UART state */
#define UART_FLOW( uart )    INFIX( U, uart, FLOW )    /* flow control enable */
#define UART_D9( uart )      INFIX( U, uart, D9 )      /* data bit 9 or parity */
#define UART_BIT9( uart )    INFIX( U, uart, BIT9 )    /* 9 bit data enable */
#define UART_PARITY( uart )  INFIX( U, uart, PARITY )  /* parity enable */
#define UART_SPB( uart )     INFIX( U, uart, SPB )     /* number of stop bits */
#define UART_STOP( uart )    INFIX( U, uart, STOP )    /* stop bit polarity */
#define UART_START( uart )   INFIX( U, uart, START )   /* start bit polarity */

#define UART_UxGCR( uart )   INFIX( U, uart, GCR )     /* generic control register */
#define UART_ORDER( uart )   INFIX( U, uart, ORDER )   /* lsb or msb first */
#define UART_BAUD_E( uart )  INFIX( U, uart, BAUD_E )  /* baud rate exponent */

#define UART_UxDBUF( uart )  INFIX( U, uart, DBUF )    /* tranciever data buffer */
#define UART_UxBAUD( uart )  INFIX( U, uart, BAUD )    /* buad rate mantissa */

/* These macros provide access to resource allocation functions in the io registers */
#define UART_UxCFG( uart )   INFIX( U, uart, CFG )



/*            UART REGISTER AND BIT FIELD OPERATIONS MACROS                   */

/* uart mode settings */
#define UART_MODE_UART 1
#define UART_MODE_SPI  0
#define UART_MODE_SET_UART( uart ) ( UART_MODE( uart ) = UART_MODE_UART )

/* uart receiver enabled states */
#define UART_RE_ENABLED  1
#define UART_RE_DISABLED 0
#define UART_RE_ENABLE( uart )  ( UART_RE( uart ) = UART_RE_ENABLED )
#define UART_RE_DISABLE( uart ) ( UART_RE( uart ) = UART_RE_DISABLED )

/* uart frame error status */
#define UART_FE_NOERR 0
#define UART_FE_ERROR 1

/* uart parity error status */
#define UART_ERR_NOERR 0
#define UART_ERR_ERROR 1

/* uart received byte status */
#define UART_RX_BYTE_RX_EMPTY 0
#define UART_RX_BYTE_RX_FULL  1

/* uart transmit byte status */
#define UART_TX_BYTE_TX_FULL  0
#define UART_TX_BYTE_TX_EMPTY 1

/* uart busy status */
#define UART_ACTIVE_IDLE 0
#define UART_ACTIVE_BUSY 1

/* uart parity states */
#if ( defined MRFI_CC2530 ) || ( defined MRFI_CC2531 ) || ( defined MRFI_CC2533 )
  #define UART_PARITY_ENABLED  1
  #define UART_PARITY_DISABLED 0
#else
  #define UART_PARITY_ENABLED  0
  #define UART_PARITY_DISABLED 1
#endif

/* uart flush the uart */
#define UART_FLUSH_FLUSH_NOW 1
#define UART_FLUSH_NOW( uart ) ( UART_FLUSH( uart ) = UART_FLUSH_FLUSH_NOW )

/* uart hardware flow control options */
#define UART_FLOW_CONTROL_ENABLE( uart )  ( UART_FLOW( uart ) = UART_FLOW_CONTROL_ON )
#define UART_FLOW_CONTROL_DISABLE( uart ) ( UART_FLOW( uart ) = UART_FLOW_CONTROL_OFF )

/* uart bit 9 or parity states */
#define UART_D9_HI          1
#define UART_D9_LO          0
#if ( defined MRFI_CC2530 ) || ( defined MRFI_CC2531 ) || ( defined MRFI_CC2533 )
  #define UART_D9_EVEN_PARITY 1
  #define UART_D9_ODD_PARITY  0
#else
  #define UART_D9_EVEN_PARITY 0
  #define UART_D9_ODD_PARITY  1
#endif
#define UART_SET_PARITY_EVEN( uart )       ( UART_D9( uart ) = UART_D9_EVEN_PARITY )
#define UART_SET_PARITY_ODD( uart )        ( UART_D9( uart ) = UART_D9_ODD_PARITY )
#define UART_SET_PARITY_MODE( uart, mode ) ( ( mode == UART_PARITY_EVEN ) \
                  ? UART_SET_PARITY_EVEN( uart ) : UART_SET_PARITY_ODD( uart ) )

/* uart 8/9 bit transfer size selection */
#define UART_BIT9_8_BIT_MODE  0
#define UART_BIT9_9_BIT_MODE  1
#define UART_BIT9_USE_PARITY  1
#define UART_BIT9_NO_PARITY   0
#define UART_BIT9_SET_9_BIT_MODE( uart ) ( UART_BIT9( uart ) = UART_BIT9_9_BIT_MODE )
#define UART_BIT9_SET_8_BIT_MODE( uart ) ( UART_BIT9( uart ) = UART_BIT9_8_BIT_MODE )

/* uart parity enabled status */
#define UART_PARITY_ENABLE( uart, mode )  ( UART_PARITY( uart ) = UART_PARITY_ENABLED, \
                                                    UART_BIT9_SET_9_BIT_MODE( uart ),  \
                                                    UART_SET_PARITY_MODE( uart, mode ) )
#define UART_PARITY_DISABLE( uart, mode ) ( UART_PARITY( uart ) = UART_PARITY_DISABLED )

/* uart number of stops bits to use */
#define UART_SPB_1_STOP_BIT  UART_1_STOP_BIT
#define UART_SPB_2_STOP_BITS UART_2_STOP_BITS
#define UART_SPB_USE_1_STOP_BIT( uart )  ( UART_SPB( uart ) = UART_SPB_1_STOP_BIT )
#define UART_SPB_USE_2_STOP_BITS( uart ) ( UART_SPB( uart ) = UART_SPB_2_STOP_BITS )

/* uart stop bit polarity */
#define UART_STOP_LO 0
#define UART_STOP_HI 1
#define UART_STOP_ACTIVE_LO( uart ) ( UART_STOP( uart ) = UART_STOP_LO )
#define UART_STOP_ACTIVE_HI( uart ) ( UART_STOP( uart ) = UART_STOP_HI )

/* uart start bit polarity */
#define UART_START_LO 0
#define UART_START_HI 1
#define UART_START_ACTIVE_LO( uart ) ( UART_START( uart ) = UART_START_LO )
#define UART_START_ACTIVE_HI( uart ) ( UART_START( uart ) = UART_START_HI )

/* uart transfer order settings */
#define UART_ORDER_LSB_FIRST 0
#define UART_ORDER_MSB_FIRST 1
#define UART_ORDER_SEND_LSB_FIRST( uart ) ( UART_ORDER( uart ) = UART_ORDER_LSB_FIRST )
#define UART_ORDER_SEND_MSB_FIRST( uart ) ( UART_ORDER( uart ) = UART_ORDER_MSB_FIRST )



/*        RESOURCE ALLOCATION AND INITIALIZATION MACROS                       */

/* set uart operations for alternate location 1 or 2 */
#define UART_INIT_UxCFG( uart, loc )  ( UART_UxCFG( uart ) = loc - 1 )

/* disable any adc pins which will be used by the uart */
#define UART_ADCCFGx( uart, loc, func ) CONCATENATE( ADCCFG, UART_PIN( uart, loc, func ) )
#define UART_INIT_ADCCFG( uart, loc, flow ) \
                  ( ( loc != 1 ) ? ( (void) 0 ) : \
                      ( (void) ( ( flow == UART_FLOW_CONTROL_OFF ) ? \
                                       ( UART_ADCCFGx( uart, loc, TX ) = 0, \
                                         UART_ADCCFGx( uart, loc, RX ) = 0 ) : \
                                       ( UART_ADCCFGx( uart, loc, TX ) = 0, \
                                         UART_ADCCFGx( uart, loc, RX ) = 0, \
                                         UART_ADCCFGx( uart, loc, CT ) = 0, \
                                         UART_ADCCFGx( uart, loc, RT ) = 0 ) ) ) )

/* set appropriate io pins to peripheral function for use by the uart */
#define UART_PxSEL_x( uart, loc, func ) INFIX( CONCATENATE( SELP, UART_PORT( uart, loc ) ), _, UART_PIN( uart, loc, func ) )
#define UART_INIT_PxSEL( uart, loc, flow ) \
                  ( ( flow == UART_FLOW_CONTROL_OFF ) ? \
                                    ( UART_PxSEL_x( uart, loc, TX ) = 1, \
                                      UART_PxSEL_x( uart, loc, RX ) = 1 ) : \
                                    ( UART_PxSEL_x( uart, loc, TX ) = 1, \
                                      UART_PxSEL_x( uart, loc, RX ) = 1, \
                                      UART_PxSEL_x( uart, loc, CT ) = 1, \
                                      UART_PxSEL_x( uart, loc, RT ) = 1 ) )

/* set priority of uart over other peripherals, assumes current uart being
 * configured has priority over the other uart whether or not it is configured */
#define UART_INIT_PRIxPx( uart, loc ) ( ( loc != 1 ) \
                    ? PRIP0 = uart : ( PRI3P1 = uart, PRI2P1 = 0, PRI0P1 = 0 ) )

/* set flow control */
#define UART_INIT_FLOW_CONTROL( uart, flow ) \
    ( ( flow == UART_FLOW_CONTROL_OFF ) \
        ? UART_FLOW_CONTROL_DISABLE( uart ) : UART_FLOW_CONTROL_ENABLE( uart ) )

/* set parity */
#define UART_INIT_PARITY( uart, parity ) ( ( parity == UART_PARITY_NONE ) \
        ? UART_PARITY_DISABLE( uart, parity ) : UART_PARITY_ENABLE( uart, parity ) )
//( ( parity == UART_PARITY_ENABLED ) \
                    ? UART_PARITY_ENABLE( uart ) : UART_PARITY_DISABLE( uart ) )

/* set number of stop bits */
#define UART_INIT_SPB( uart, spb ) ( ( spb == UART_SPB_1_STOP_BIT ) \
          ? UART_SPB_USE_1_STOP_BIT( uart ) : UART_SPB_USE_2_STOP_BITS( uart ) )

/* set stop bit(s) polarity */
#define UART_INIT_STOP( uart, stop ) ( ( stop == UART_STOP_LO ) \
                   ? UART_STOP_ACTIVE_LO( uart ) : UART_STOP_ACTIVE_HI( uart ) )

/* set start bit polarity */
#define UART_INIT_START( uart, start ) ( ( start == UART_START_LO ) \
                 ? UART_START_ACTIVE_LO( uart ) : UART_START_ACTIVE_HI( uart ) )

/* set transfer bit order */
#define UART_INIT_ORDER( uart, order ) ( ( order == UART_ORDER_LSB_FIRST ) \
       ? UART_ORDER_SEND_LSB_FIRST( uart ) : UART_ORDER_SEND_MSB_FIRST( uart ) )

/* set baud rate... */
/*
The equation from the data sheets for the CC2430, CC2510, and CC1110 is

                      baud_e
   (256 + baud_m) * 2
  --------------------------- * fref == baud_rate
                28
              2

Knowing that the maximum value for the baud_m register is 255, we can
create a table using that value and the reference clock rate to determine
where the baud_e cutoff buad rates are at.  For example, if we plug 32MHz in
for fref, 255 in for baud_m, and the values 0 through 15 in for baud_e then
we get the following table of values

        | maximum
baud_e  | achievable
setting | baud rate
--------+------------
    0   |       60
    1   |      121
    2   |      243
    3   |      487
    4   |      974
    5   |     1949
    6   |     3898
    7   |     7797
    8   |    15594
    9   |    31188
   10   |    62377
   11   |   124755
   12   |   249511
   13   |   499023
   14   |   998046
   15   |  1996093

Thus we can use the spec sheet formula to calculate baud_e values based on the
baud rate requested and the system clock frequency.

Note: there is a little bit of fixed point optimization in the following
      formula to manage overflow issues.
*/
#define UART_CALC_BAUD_E_CUTOFF( n ) \
         ( ( ( 511ul * UART_CLOCK_MHZ * 15625ul ) >> ( 22 - n ) ) + 1 )
#define UART_BAUD_CAST( baud ) ( (unsigned long) baud )
#define UART_CALC_BAUD_E( baud )      \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  0 ) ) ?  0 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  1 ) ) ?  1 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  2 ) ) ?  2 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  3 ) ) ?  3 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  4 ) ) ?  4 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  5 ) ) ?  5 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  6 ) ) ?  6 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  7 ) ) ?  7 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  8 ) ) ?  8 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF(  9 ) ) ?  9 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 10 ) ) ? 10 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 11 ) ) ? 11 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 12 ) ) ? 12 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 13 ) ) ? 13 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 14 ) ) ? 14 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 15 ) ) ? 15 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 16 ) ) ? 16 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 17 ) ) ? 17 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 18 ) ) ? 18 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 19 ) ) ? 19 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 20 ) ) ? 20 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 21 ) ) ? 21 : \
         ( ( baud < UART_CALC_BAUD_E_CUTOFF( 22 ) ) ? 22 : \
             23 ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) ) )
/*
Now that we know what the value we need for the baud_e setting, we can use
that knowledge and solve the original formula for the baud_m setting resulting
in the following

                    28     baud_e
                  2    * 2        * baud
baud_m == -256 + ------------------------
                           fref

Again, a little fixed point optimization and rounding results in
*/
#define UART_CALC_BAUD_M( baud ) \
                           ( ( ( ( baud << ( 23 - UART_CALC_BAUD_E( baud ) ) ) \
                                 / 15625ul / UART_CLOCK_MHZ + 1 ) >> 1 ) - 256 )
#define UART_INIT_BAUD( uart, baud ) \
           ( UART_BAUD_E( uart ) = UART_CALC_BAUD_E( UART_BAUD_CAST( baud ) ), \
             UART_UxBAUD( uart ) = UART_CALC_BAUD_M( UART_BAUD_CAST( baud ) ) )

/* initialize all aspects of a uart according to the parameters passed
 * assumes data is always 8 bits, no 9 bit support is supplied in this macro
 * also the receiver is enabled, no support for transmit only in this macro */
#define UART_INIT_8051( uart, loc, flow, parity, spb, stop, start, order, baud ) \
  ( UART_MODE_SET_UART( uart ),           UART_RE_ENABLE( uart ),                \
    UART_INIT_UxCFG( uart, loc ),         UART_INIT_ADCCFG( uart, loc, flow ),   \
    UART_INIT_PxSEL( uart, loc, flow ),   UART_INIT_PRIxPx( uart, loc ),         \
    UART_INIT_FLOW_CONTROL( uart, flow ), UART_INIT_PARITY( uart, parity ),      \
    UART_INIT_SPB( uart, spb ),           UART_INIT_STOP( uart, stop ),          \
    UART_INIT_START( uart, start ),       UART_INIT_ORDER( uart, order ),        \
    UART_INIT_BAUD( uart, baud ),         UART_FLUSH_NOW( uart ) )

/* sanity check */
#if !( defined UART_NUMBER )
    || !( defined UART_LOCATION )
#error "The macros UART_NUMBER and UART_LOCATION must be defined for 8051 processors."
#endif
      
/* generalized uart interface */
#define UART_INIT( uart, loc, flow, parity, stop, baud )                                  \
               UART_INIT_8051( uart, loc, flow, parity, stop, \
                       UART_STOP_HI, UART_START_LO, UART_ORDER_LSB_FIRST, baud )

#define UART_IRQ_FLAG_CLR( uart, loc, func )  ( UART_IRQ_FLAG( uart, func ) = UART_IRQ_IDLE )
#define UART_IRQ_FLAG_SET( uart, loc, func )  ( UART_IRQ_FLAG( uart, func ) = UART_IRQ_PENDING )

#define UART_SEND( uart, loc, c ) ( UART_UxDBUF( uart ) = c )
#define UART_RECEIVE( uart, loc ) UART_UxDBUF( uart )


/* ------------------------------------------------------------------------------------------------
 *                                Macros and Defines for MSP430 variants
 * ------------------------------------------------------------------------------------------------
 */
#elif ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC430__ ) || (  defined __TI_COMPILER_VERSION__)

#include "pp_utils.h"
#include "msp430.h"

#if UART_FLOW_CONTROL == UART_FLOW_CONTROL_ON
  #define UART_HARDWARE_HANDSHAKE_IN_SOFTWARE
#elif UART_FLOW_CONTROL == UART_FLOW_CONTROL_OFF
  #define NO_UART_HARDWARE_HANDSHAKE_IN_SOFTWARE
#endif

/*                  PORT PIN SUPPORT DEFINITIONS                              */
      
#define IO_PORT_REGISTER_BIT_SET( port, bit, reg, state, hi, lo ) \
                              ( ( state == lo ) \
                                  ? ( INFIX( P, port, reg ) &= ~( 1 << bit ) ) \
                                  : ( INFIX( P, port, reg ) |= ( 1 << bit ) ) )
#define IO_PORT_REGISTER_BIT_GET( port, bit, reg, hi, lo ) \
                 ( ( ( INFIX( P, port, reg ) & ( 1 << bit ) ) == 0 ) ? lo : hi )
  
#define IO_PORT_INPUT_LO 0
#define IO_PORT_INPUT_HI 1
#define IO_PORT_GET_INPUT( port, bit ) \
   IO_PORT_REGISTER_BIT_GET( port, bit, IN, IO_PORT_INPUT_HI, IO_PORT_INPUT_LO )

#define IO_PORT_OUTPUT_LO 0
#define IO_PORT_OUTPUT_HI 1
/* provide redirection since OUT is defined as a macro elswhere */
#define P0_IO_OUT P0OUT
#define P1_IO_OUT P1OUT
#define P2_IO_OUT P2OUT
#define P3_IO_OUT P3OUT
#define P4_IO_OUT P4OUT
#define P5_IO_OUT P5OUT
#define P6_IO_OUT P6OUT
#define P7_IO_OUT P7OUT
#define P8_IO_OUT P8OUT
#define IO_PORT_SET_OUTPUT( port, bit, state ) \
                          IO_PORT_REGISTER_BIT_SET( port, bit, _IO_OUT, state, \
                                          IO_PORT_OUTPUT_HI, IO_PORT_OUTPUT_LO )
#define IO_PORT_GET_OUTPUT( port, bit ) \
                                 IO_PORT_REGISTER_BIT_GET( port, bit, _IO_OUT, \
                                          IO_PORT_OUTPUT_HI, IO_PORT_OUTPUT_LO )

#define IO_PORT_DIRECTION_INPUT  0
#define IO_PORT_DIRECTION_OUTPUT 1
#define IO_PORT_SET_DIRECTION( port, bit, dir ) \
                           IO_PORT_REGISTER_BIT_SET( port, bit, DIR, dir, \
                             IO_PORT_DIRECTION_OUTPUT, IO_PORT_DIRECTION_INPUT )
#define IO_PORT_GET_DIRECTION( port, bit ) \
                           IO_PORT_REGISTER_BIT_GET( port, bit, DIR, \
                             IO_PORT_DIRECTION_OUTPUT, IO_PORT_DIRECTION_INPUT )

#define IO_PORT_RESISTOR_DISABLED 0
#define IO_PORT_RESISTOR_ENABLED  1
      
#if (defined P0REN) || (defined P1REN) || (defined P2REN)
#define IO_PORT_SET_RESISTOR( port, bit, mode ) \
                         IO_PORT_REGISTER_BIT_SET( port, bit, REN, mode, \
                           IO_PORT_RESISTOR_ENABLED, IO_PORT_RESISTOR_DISABLED )
#define IO_PORT_GET_RESISTOR( port, bit ) \
                         IO_PORT_REGISTER_BIT_GET( port, bit, REN, \
                           IO_PORT_RESISTOR_ENABLED, IO_PORT_RESISTOR_DISABLED )
#else
#define IO_PORT_SET_RESISTOR( port, bit, mode ) 1 
#define IO_PORT_GET_RESISTOR( port, bit ) 1
#endif
      
#define IO_PORT_SELECT_IO         0
#define IO_PORT_SELECT_PERIPHERAL 1
#define IO_PORT_SET_SELECT( port, bit, mode ) \
                               IO_PORT_REGISTER_BIT_SET( port, bit, SEL, mode, \
                                  IO_PORT_SELECT_PERIPHERAL, IO_PORT_SELECT_IO )
#define IO_PORT_GET_SELECT( port, bit ) \
                               IO_PORT_REGISTER_BIT_GET( port, bit, SEL, \
                                  IO_PORT_SELECT_PERIPHERAL, IO_PORT_SELECT_IO )

#define IO_PORT_INTERRUPT_DISABLED 0
#define IO_PORT_INTERRUPT_ENABLED  1
#define IO_PORT_SET_INTERRUPT_ENABLED_STATE( port, bit, mode ) \
                       IO_PORT_REGISTER_BIT_SET( port, bit, IE, mode, \
                         IO_PORT_INTERRUPT_ENABLED, IO_PORT_INTERRUPT_DISABLED )
#define IO_PORT_GET_INTERRUPT_ENABLED_STATE( port, bit ) \
                       IO_PORT_REGISTER_BIT_GET( port, bit, IE, \
                         IO_PORT_INTERRUPT_ENABLED, IO_PORT_INTERRUPT_DISABLED )

#define IO_PORT_INTERRUPT_RISING_EDGE  0
#define IO_PORT_INTERRUPT_FALLING_EDGE 1
#define IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode ) \
                IO_PORT_REGISTER_BIT_SET( port, bit, IES, mode, \
                  IO_PORT_INTERRUPT_FALLING_EDGE, IO_PORT_INTERRUPT_RISING_EDGE )
#define IO_PORT_GET_INTERRUPT_EDGE( port, bit ) \
                IO_PORT_REGISTER_BIT_GET( port, bit, IES, \
                  IO_PORT_INTERRUPT_FALLING_EDGE, IO_PORT_INTERRUPT_RISING_EDGE )
      
#define IO_PORT_INTERRUPT_IDLE    0
#define IO_PORT_INTERRUPT_PENDING 1
#define IO_PORT_SET_INTERRUPT_STATE( port, bit, mode ) \
                           IO_PORT_REGISTER_BIT_SET( port, bit, IFG, mode, \
                             IO_PORT_INTERRUPT_PENDING, IO_PORT_INTERRUPT_IDLE )
#define IO_PORT_GET_INTERRUPT_STATE( port, bit ) \
                           IO_PORT_REGISTER_BIT_GET( port, bit, IFG, \
                             IO_PORT_INTERRUPT_PENDING, IO_PORT_INTERRUPT_IDLE )

/* the following macro enables an interrupt only after making sure the interrupt
 * flag is cleared first */
#define IO_PORT_SAFE_ENABLE_INTERRUPT( port, bit ) \
 ( IO_PORT_SET_INTERRUPT_STATE( port, bit, IO_PORT_INTERRUPT_IDLE ), \
   IO_PORT_SET_INTERRUPT_ENABLED_STATE( port, bit, IO_PORT_INTERRUPT_ENABLED ) )

/* the following macro changes the sense of the edge that causes an interrupt
 * in a manner that guarantees no interrupt is generated. */
#define IO_PORT_SAFE_SET_INTERRUPT_EDGE( port, bit, mode ) \
                  ( ( IO_PORT_GET_INTERRUPT_STATE( port, bit ) \
                        == IO_PORT_INTERRUPT_DISABLED ) \
                           ? IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode ) \
                           : ( IO_PORT_SET_INTERRUPT_STATE( port, bit, \
                                                 IO_PORT_INTERRUPT_DISABLED ), \
                               IO_PORT_SET_INTERRUPT_EDGE( port, bit, mode ), \
                               IO_PORT_SAFE_ENABLE_INTERRUPT( port, bit ) ) )
      
#ifndef UART_TX_PORT_NUM
  #define UART_TX_PORT_NUM 3
#endif
#ifndef UART_TX_BIT_NUM
  #define UART_TX_BIT_NUM 4
#endif
#ifndef UART_RX_PORT_NUM
  #define UART_RX_PORT_NUM 3
#endif
#ifndef UART_RX_BIT_NUM
  #define UART_RX_BIT_NUM 5
#endif
#ifndef UART_CTS_PORT_NUM
  #define UART_CTS_PORT_NUM 2
#endif
#ifndef UART_CTS_BIT_NUM
  #define UART_CTS_BIT_NUM 7
#endif
#ifndef UART_RTS_PORT_NUM
  #define UART_RTS_PORT_NUM 2
#endif
#ifndef UART_RTS_BIT_NUM
  #define UART_RTS_BIT_NUM 6
#endif

#define UART_RTS_ASSERTED   IO_PORT_OUTPUT_HI
#define UART_RTS_DEASSERTED IO_PORT_OUTPUT_LO
#define UART_ASSERT_RTS( state ) IO_PORT_SET_OUTPUT( UART_RTS_PORT_NUM, UART_RTS_BIT_NUM, state )
      
#define UART_CTS_DEASSERTED IO_PORT_OUTPUT_LO
#define UART_CTS_CLEAR_INTERRUPT( ) \
  IO_PORT_SET_INTERRUPT_STATE( UART_CTS_PORT_NUM, UART_CTS_BIT_NUM, IO_PORT_INTERRUPT_IDLE ) 
#define UART_CTS_GET_STATE( ) IO_PORT_GET_INPUT( UART_CTS_PORT_NUM, UART_CTS_BIT_NUM )
#define UART_CTS_ASSERTION_EDGE ( ( UART_CTS_DEASSERTED == IO_PORT_OUTPUT_LO ) \
                                              ? IO_PORT_INTERRUPT_RISING_EDGE  \
                                              : IO_PORT_INTERRUPT_FALLING_EDGE )

#ifndef UART_CTS_RTS_INIT
  #if( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE )
      
    #define UART_CTS_RTS_INIT( rts_port, rts_bit, cts_port, cts_bit ) \
      ( IO_PORT_SET_INTERRUPT_STATE( cts_port, cts_bit, IO_PORT_INTERRUPT_DISABLED ), \
        UART_ASSERT_RTS( UART_RTS_ASSERTED ), \
        IO_PORT_SET_DIRECTION( rts_port, rts_bit, IO_PORT_DIRECTION_OUTPUT ), \
        IO_PORT_SET_RESISTOR( rts_port, rts_bit, IO_PORT_RESISTOR_DISABLED ), \
        IO_PORT_SET_SELECT( rts_port, rts_bit, IO_PORT_SELECT_IO ), \
        IO_PORT_SET_SELECT( cts_port, cts_bit, IO_PORT_SELECT_IO ), \
        IO_PORT_SET_DIRECTION( cts_port, cts_bit, IO_PORT_DIRECTION_INPUT ), \
        IO_PORT_SET_OUTPUT( cts_port, cts_bit, UART_CTS_DEASSERTED ), \
        IO_PORT_SET_RESISTOR( cts_port, cts_bit, IO_PORT_RESISTOR_ENABLED ), \
        IO_PORT_SAFE_SET_INTERRUPT_EDGE( cts_port, cts_bit, UART_CTS_ASSERTION_EDGE ), \
        IO_PORT_SAFE_ENABLE_INTERRUPT( cts_port, cts_bit ) )

  #else /* !( defined UART_HARDWARE_HANDSHAKE_IN_SOFTWARE ) */ 
    #define UART_CTS_RTS_INIT( rts_port, rts_bit, cts_port, cts_bit ) \
                                             ( (void)0 ) /* do nothing */
  #endif
#endif /* !( defined UART_CTS_RTS_INIT ) */
      
/*                       UART IDENTIFICATION                                  */
      
#define UART_ID( num, loc ) INFIX( UART_ID_, num, loc )
#define UART_ID_A0    0x4130 /* 'A' in hex followed by '0' in hex */
#define UART_ID_A1    0x4131 /* 'A' in hex followed by '1' in hex */
#define UART_ID_B0    0x4230 /* 'B' in hex followed by '0' in hex */
#define UART_ID_B1    0x4231 /* 'B' in hex followed by '1' in hex */

/*                   UART REGISTER RESOLUTION                                 */
      
#define UCxxCTL0( num, loc ) CONCATENATE( INFIX( UC, num, loc ), CTL0 )
#define UCxxCTL1( num, loc ) CONCATENATE( INFIX( UC, num, loc ), CTL1 )
#define UCxxBR0( num, loc )  CONCATENATE( INFIX( UC, num, loc ), BR0 )
#define UCxxBR1( num, loc )  CONCATENATE( INFIX( UC, num, loc ), BR1 )
#define UCxxMCTL( num, loc ) CONCATENATE( INFIX( UC, num, loc ), MCTL )

/*                   PARITY OPERATION                                         */
      
#define UART_PARITY_ENABLE( parity ) ( ( ( parity ) == UART_PARITY_NONE ) ? 0 : ( UCPEN ) )
#define UART_PARITY_FORM( parity )   ( ( ( parity ) == UART_PARITY_EVEN ) ? ( UCPAR ) : 0 )
#define UART_PARITY_SET( num, loc, parity )   ( UCxxCTL0( num, loc )           \
                                             = ( UCxxCTL0( num, loc )          \
                                                 & ~( ( UCPEN ) | ( UCPAR ) ) )\
                                               | UART_PARITY_ENABLE( parity )  \
                                               | UART_PARITY_FORM( parity ) )
      
/*                    DATA BYTE ORDERING                                      */
      
#define UART_LSB_FIRST 0
#define UART_MSB_FIRST 1
#define UART_BYTE_ORDER_SET( num, loc, mode )                               \
                                 ( ( ( mode ) == UART_LSB_FIRST )           \
                                   ? ( UCxxCTL0( num, loc ) &= ~( UCMSB ) ) \
                                   : ( UCxxCTL0( num, loc ) |= ( UCMSB ) ) )

/*                      DATA LENGTH                                           */
      
#define UART_DATA_LENGTH_7_BITS 1
#define UART_DATA_LENGTH_8_BITS 0
#define UART_DATA_LENGTH_SET( num, loc, mode )                               \
                                  ( ( ( mode ) == UART_DATA_LENGTH_7_BITS )  \
                                    ? ( UCxxCTL0( num, loc ) |= ( UC7BIT ) ) \
                                    : ( UCxxCTL0( num, loc ) &= ~( UC7BIT ) ) )

/*                        STOP BITS                                           */
      
#define UART_STOP_BITS_SET( num, loc, bits )                         \
                          ( ( ( bits ) == UART_1_STOP_BIT ) ?        \
                              ( UCxxCTL0( num, loc ) &= ~( UCSPB ) ) \
                            : ( UCxxCTL0( num, loc ) |= ( UCSPB ) ) )

/*                       UART MODE OF OPERATION                               */
      
#define UART_MODE_UART      0
#define UART_MODE_IDLE_LINE 1
#define UART_MODE_ADDRESSED 2
#define UART_MODE_AUTO_BAUD 3
#define UART_MODE_SET( num, loc, mode ) ( UCxxCTL0( num, loc )                 \
       = ( ( UCxxCTL0( num, loc ) & ~( UCMODE_3 ) )                            \
         | ( ( ( ( mode ) & UART_MODE_ADDRESSED ) == 0 ) ? 0 : ( UCMODE1 ) )   \
         | ( ( ( ( mode ) & UART_MODE_IDLE_LINE ) == 0 ) ? 0 : ( UCMODE0 ) ) ) )

/*                    SYNCHRONOUS/ASYNCHRONOUS OPERATION                      */
#define UART_SYNC_ASYNC 0
#define UART_SYNC_SYNC  1
#define UART_SYNC_SET( num, loc, mode )                                 \
                            ( ( ( mode ) == UART_SYNC_ASYNC )           \
                              ? ( UCxxCTL0( num, loc ) &= ~( UCSYNC ) ) \
                              : ( UCxxCTL0( num, loc ) |= ( UCSYNC ) ) )

/*                     UART CLOCK SOURCE SELECT                               */
#define UART_CLOCK_UCLK  0
#define UART_CLOCK_ACLK  1
#define UART_CLOCK_SMCLK 2
#define UART_CLOCK_SET( num, loc, src ) ( UCxxCTL1( num, loc )                 \
           = ( ( UCxxCTL1( num, loc ) & ~( UCSSEL1 | UCSSEL0 ) )               \
             | ( ( ( ( src ) & UART_CLOCK_SMCLK ) == 0 ) ? 0 : ( UCSSEL1 ) )   \
             | ( ( ( ( src ) & UART_CLOCK_ACLK  ) == 0 ) ? 0 : ( UCSSEL0 ) ) ) )

/*                         ENABLE/RESET UART                                  */
      
#define UART_RESET( num, loc )  ( UCxxCTL1( num, loc ) |= UCSWRST )
#define UART_ENABLE( num, loc ) ( UCxxCTL1( num, loc ) &= ~UCSWRST )

/*                       BAUD RATE CALCULATION                                */
      
#define UART_FLOOR( n, b )           ( ( (unsigned long)n ) >> ( b ) )
#define UART_ROUND( n, b )           ( ( ( ( (unsigned long)n ) >> ( ( b ) - 1 ) ) + 1 ) >> 1 )
/* using q27.5 to represent the N value in the documentation calculations */
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
      

/* this setup macro assumes 8 bit data lsb first to follow standard RS232 interface protocols     */  
#define UART_INIT_F2618( num, loc, flow, parity, stop, baud )                                      \
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

#define UART_IRQ_FLAG_CLR( num, loc, func )     /* do nothing */
#define UART_IRQ_FLAG_SET( num, loc, func )     /* do nothing */
#define UART_IRQ_ENABLE( num, loc, func )       ( IE2 |=  INFIX( UC, num, INFIX( loc, func, IE ) ) )
#define UART_IRQ_DISABLE( num, loc, func )      ( IE2 &= ~INFIX( UC, num, INFIX( loc, func, IE ) ) )
      
#define UART_SEND( num, loc, c ) ( CONCATENATE( UC, INFIX( num, loc, TXBUF ) ) = c )
#define UART_RECEIVE( num, loc ) CONCATENATE( UC, INFIX( num, loc, RXBUF ) )
     
#define UART_INIT( num, loc, flow, parity, stop, baud ) \
      UART_INIT_F2618( num, loc, flow, parity, stop, baud )
      
#endif /* defined ( __IAR_SYSTEMS_ICC__ && __ICC430__ && BSP_BOARD_SRF05EB ) */

      
      
      
      
      
      
      
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
