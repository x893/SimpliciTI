/**************************************************************************************************
  Filename:       nwk_pll.h
  Revised:        $Date: 2009-02-09 16:48:33 -0700 (Tue, 06 May 2008) $
  Revision:       $Revision: 17025 $
  Author:         $Author: jnoxon $

  Description:    This header file supports the SimpliciTI PLL network application.

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

#ifndef NWK_PLL_H
#define NWK_PLL_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "mrfi.h"
#include "nwk_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Typdefs
 * ------------------------------------------------------------------------------------------------
 */

#define MAX_PLL_APP_FRAME 34

#ifdef NWK_PLL
enum
{
  pll_cmd_LocateReference = 0x01,
  pll_cmd_LocateResponse  = 0x02,
  pll_cmd_PumpRequest     = 0x04,
  pll_cmd_PumpResponse    = 0x08,
  pll_ent_reference       = 0x10,
  pll_ent_alternate       = 0x20,
  pll_ent_slave           = 0x40,
  pll_locked              = 0x80
};

typedef struct
{
  uint8_t     Cmd; // must be first so data logging works correctly
  uint8_t     LinkQuality;
  uint16_t    misc;
  int32_t     ChannelDelay;
  addr_t      OriginatorAddr;
  mrfi_Time_t Time;
} pll_Packet_t;

#define PLL_PKT_MEMBER_SZ( m )                 sizeof(((pll_Packet_t*)0)->m)
#define PLL_PKT_MEMBER_STRUCT_OFS( m )         MRFI_OFFSET_OF( pll_Packet_t, m )
#define PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE( m, n ) \
             ( ( PLL_PKT_MEMBER_STRUCT_OFS(n) < PLL_PKT_MEMBER_STRUCT_OFS(m) ) \
                                                    ? PLL_PKT_MEMBER_SZ(n) : 0 )
#define PLL_PKT_MEMBER_BUFFER_OFS( m ) \
        ( ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, Cmd) )            \
        + ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, LinkQuality) )    \
        + ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, misc) )           \
        + ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, ChannelDelay) )   \
        + ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, OriginatorAddr) ) \
        + ( PLL_PKT_MEMBER_BUFFER_SZ_INCLUDE(m, Time) )           )
#define PLL_PKT_BUFFER_SZ    ( PLL_PKT_MEMBER_BUFFER_OFS(Time) \
                               + MRFI_TIME_PACKET_SZ )
#define PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d, m ) \
            MRFI_MOVE_UNALIGNED( (uint8_t*)(d) + PLL_PKT_MEMBER_STRUCT_OFS(m), \
                                 (uint8_t*)(d) + PLL_PKT_MEMBER_BUFFER_OFS(m), \
                                 (d)->m, 1 /* copy high to low */ )
#define PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d, m ) \
            MRFI_MOVE_UNALIGNED( (uint8_t*)(d) + PLL_PKT_MEMBER_BUFFER_OFS(m), \
                                 (uint8_t*)(d) + PLL_PKT_MEMBER_STRUCT_OFS(m), \
                                 (d)->m, 0 /* copy low to high */ )
#define PLL_PKT_EXPAND_FROM_BUFFER( d ) \
                   ( MRFI_TIME_EXPAND_FROM_BUFFER( &( (d)->Time) ), \
                     PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d, OriginatorAddr), \
                     PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d,   ChannelDelay), \
                     PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d,           misc), \
                     PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d,    LinkQuality), \
                     PLL_PKT_EXPAND_MEMBER_FROM_BUFFER( d,            Cmd)  )
#define PLL_PKT_COMPRESS_TO_BUFFER( d ) \
                   ( PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d,            Cmd), \
                     PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d,    LinkQuality), \
                     PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d,           misc), \
                     PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d,   ChannelDelay), \
                     PLL_PKT_COMPRESS_MEMBER_TO_BUFFER( d, OriginatorAddr), \
                     MRFI_TIME_COMPRESS_TO_BUFFER( &( (d)->Time) ) )
#endif


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void         nwk_PLLInit(void);
fhStatus_t   nwk_processPLL( mrfiPacket_t * );
smplStatus_t nwk_pllControl( ioctlAction_t action, void *val );
bool     nwk_pllBackgrounder( bool no_pump );
bool         nwk_pllIsLocked( uint8_t pll_num );

#endif
