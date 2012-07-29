
/**************************************************************************************************
  Filename:       nwk_pll.c
  Revised:        $Date: 2008-07-02 10:50:58 -0700 (Wed, 02 Jul 2008) $
  Revision:       $Revision: 17448 $
  Author:         $Author: jnoxon $

  Description:    This file supports the SimpliciTI PLL network application.

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
#include <string.h>
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk.h"
#include "nwk_frame.h"
#include "nwk_globals.h"
#include "nwk_api.h"
#include "nwk_pll.h"

#ifdef NWK_PLL_SHOW_LOCATION_INDICATORS
#include "bsp_leds.h"
#endif

#ifdef NWK_PLL

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

// get pointer to pll_packet_structure, hand it a mrfi_packet_t type
#define NWK_PLL_PACKET_PTR( f ) ( (pll_Packet_t*)( MRFI_P_PAYLOAD( f ) + F_APP_PAYLOAD_OS ) )

#ifndef NWK_PLL_RX_TIME_STAMP_FIFO_SZ
  #define NWK_PLL_RX_TIME_STAMP_FIFO_SZ 4
#endif

#ifndef NWK_PLL_SAMPLE_PERIOD_DEFAULT
  #define NWK_PLL_SAMPLE_PERIOD_DEFAULT 50
#endif
#if NWK_PLL_SAMPLE_PERIOD_DEFAULT < 20
  #error "ERROR: NWK_PLL_SAMPLE_PERIOD_DEFAULT must be at least 20ms."
#endif

#ifndef NWK_PLL_SAMPLE_PERIOD_MAX
  #define NWK_PLL_SAMPLE_PERIOD_MAX ( 0x3FFFFFFFL )  // effectively infinity
#endif

#ifndef NWK_PLL_LOOP_GAIN_Kp
  #define NWK_PLL_LOOP_GAIN_Kp 800//8192
#endif
#define NWK_PLL_LOOP_GAIN_Kp_x ( ( 1L * NWK_PLL_LOOP_GAIN_Kp * ( BSP_TIMER_CLK_KHZ ) ) / 1024 )

#ifndef NWK_PLL_LOOP_GAIN_Ki
  #define NWK_PLL_LOOP_GAIN_Ki 100//128
#endif
#define NWK_PLL_LOOP_GAIN_Ki_x ( ( 1L * NWK_PLL_LOOP_GAIN_Ki * ( BSP_TIMER_CLK_KHZ ) ) / 1024 )

#ifndef NWK_PLL_LOOP_GAIN_Kd
  #define NWK_PLL_LOOP_GAIN_Kd 0
#endif
#define NWK_PLL_LOOP_GAIN_Kd_x ( ( 1L * NWK_PLL_LOOP_GAIN_Kd * ( BSP_TIMER_CLK_KHZ ) ) / 1024 )

#ifndef NWK_PLL_Kp_HOLD_OFF
  #define NWK_PLL_Kp_HOLD_OFF 0
#endif

#ifndef NWK_PLL_HI_GAIN_THRESHOLD
  #define NWK_PLL_HI_GAIN_THRESHOLD (NWK_PLL_SAMPLE_PERIOD_DEFAULT * 5L)
#endif

#ifndef NWK_PLL_LO_GAIN_THRESHOLD
  #define NWK_PLL_LO_GAIN_THRESHOLD (NWK_PLL_SAMPLE_PERIOD_DEFAULT * 50L)
#endif

#ifndef NWK_PLL_HI_GAIN_WINDOW_SZ
  #define NWK_PLL_HI_GAIN_WINDOW_SZ 1000 // 1ms
#endif

#ifndef NWK_PLL_LO_GAIN_WINDOW_SZ
  #define NWK_PLL_LO_GAIN_WINDOW_SZ 100 // 100us
#endif

#ifndef NWK_PLL_FAILURE_LIMIT
  #define NWK_PLL_FAILURE_LIMIT 10
#endif

// make sure we are not dividing by zero
#if NWK_PLL_HI_GAIN_THRESHOLD == NWK_PLL_LO_GAIN_THRESHOLD

  #define NWK_PLL_GAIN_INTERP_M 0
  #define NWK_PLL_GAIN_INTERP_B \
       ( ( ( NWK_PLL_HI_GAIN_WINDOW_SZ + NWK_PLL_LO_GAIN_WINDOW_SZ ) + 1 ) / 2 )

#else // NWK_PLL_HI_GAIN_THRESHOLD != NWK_PLL_LO_GAIN_THRESHOLD
  #define NWK_PLL_DY ( NWK_PLL_HI_GAIN_WINDOW_SZ - NWK_PLL_LO_GAIN_WINDOW_SZ )
  #define NWK_PLL_DX ( NWK_PLL_HI_GAIN_THRESHOLD - NWK_PLL_LO_GAIN_THRESHOLD )
  #define NWK_PLL_GAIN_INTERP_M  ( ( NWK_PLL_DY * 256L ) / NWK_PLL_DX )

  #define NWK_PLL_NUM_B \
              ( ( 1L * NWK_PLL_HI_GAIN_THRESHOLD ) * NWK_PLL_LO_GAIN_WINDOW_SZ \
              - ( 1L * NWK_PLL_LO_GAIN_THRESHOLD ) * NWK_PLL_HI_GAIN_WINDOW_SZ )
  #define NWK_PLL_B ( ( NWK_PLL_NUM_B * 2L ) / NWK_PLL_DX )
  #if NWK_PLL_B < 0
    #define NWK_PLL_GAIN_INTERP_B ( ( NWK_PLL_B - 1L ) / 2 )
  #else
    #define NWK_PLL_GAIN_INTERP_B ( ( NWK_PLL_B + 1L ) / 2 )
  #endif

#endif // NWK_PLL_HI_GAIN_THRESHOLD != NWK_PLL_LO_GAIN_THRESHOLD

// normalize values for comparisons and calculations to measured error
#define NWK_PLL_HI_GAIN_WINDOW_CNT \
                           ( ( NWK_PLL_HI_GAIN_WINDOW_SZ * 16384L + 65 ) / 125 )
#define NWK_PLL_LO_GAIN_WINDOW_CNT \
                           ( ( NWK_PLL_LO_GAIN_WINDOW_SZ * 16384L + 65 ) / 125 )
#define NWK_PLL_GAIN_INTERP_M_CNT \
                               ( ( NWK_PLL_GAIN_INTERP_M * 16384L + 65 ) / 125 )
#define NWK_PLL_GAIN_INTERP_B_CNT \
                               ( ( NWK_PLL_GAIN_INTERP_B * 16384L + 65 ) / 125 )

// identify what type of entity we are for communications
#ifdef NWK_PLL_REFERENCE_CLOCK
  #define NWK_PLL_ENTITY pll_ent_reference
#elif defined ACCESS_POINT || defined RANGE_EXTENDER
  #define NWK_PLL_ENTITY pll_ent_alternate
#else
  #define NWK_PLL_ENTITY pll_ent_slave
#endif

#define NWK_PLL_SAMPLE_ERROR ((int32_t)(0x80000001UL))
#define NWK_PLL_NO_RXTS_AVAILABLE ((int32_t)(0x80000000UL))

#define NWK_PLL_NUM_OF_PLLS 1

#define NWK_PLL_LOCKED_VALUE ( ( 5 * NWK_PLL_SAMPLE_PERIOD_DEFAULT <= NWK_PLL_SAMPLE_PERIOD_MAX ) \
                               ? ( 5 * NWK_PLL_SAMPLE_PERIOD_DEFAULT ) : NWK_PLL_SAMPLE_PERIOD_MAX )

/******************************************************************************
 * TYPEDEFS
 */
typedef enum
{
  NWK_PLL_STATE_OFF    = 0x01,
  NWK_PLL_STATE_ON     = 0x02,
  NWK_PLL_STATE_LOCATE = 0x04
} pll_states_t;

typedef struct
{
  uint8_t      packetSem;
  uint8_t      MasterLQI;
  addr_t       MasterAddr;
  int32_t      Integrator;
  int16_t      HoldOff;
  int32_t      LastPreFilter;
  int32_t      ChargePumpGain;
  int32_t      ChargePumpCounter;
  uint8_t      PumpRequestSem;
  int32_t      LastSampleTime;
  pll_states_t State;
  uint8_t      Failures;
  uint16_t     dead_band;
  int8_t       locked;
  int32_t      old_err;
  int32_t      average_err;
} pll_type;

/******************************************************************************
 * LOCAL VARIABLES
 */
// the time stamps of the most recent receive events
static mrfi_Time_t sRxTimeStamps[NWK_PLL_RX_TIME_STAMP_FIFO_SZ];
static pll_type splls[NWK_PLL_NUM_OF_PLLS];
static MRFI_ms_event_t spllExistingEvent;
static volatile uint8_t spllmscnt;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
#if ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC8051__ )
static __idata_reentrant void nwk_pll_1ms_event( void );
#else
static void nwk_pll_1ms_event( void );
#endif
static void nwk_pllGetChannelDelay( pll_Packet_t* ts );
static mrfi_Time_t* nwk_pllAcquireRxTimeStamp( void );
static void nwk_pllChargePump( pll_type* pll );
static void nwk_pllHandleRequestCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr );
static void nwk_pllLocateResponseCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr );
static void nwk_pllPumpResponseCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr );
static void nwk_pllRunChargePump( pll_type* pll, pll_Packet_t* data, int32_t req_dly );
static void nwk_pllRunControlLoop( pll_type* pll, pll_Packet_t* data, int32_t err_sample );
static void nwk_pllCalcFailureRate( pll_type* pll, int32_t err_sample );
static void nwk_pllCalcNewSamplePeriod( pll_type* pll, int32_t err_sample );
static uint32_t nwk_pllCheckErrorThreshold( pll_type* pll, int32_t err_sample );
static void nwk_pllResetPLL( void );
static void nwk_pllStartPLL( void );
static void nwk_pllJumpPLL( mrfi_Time_t* t );

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_PLLInit
 *
 * @brief       Initialize NWK Phase Locked Loop application.
 *
 * @return   none.
 */
void nwk_PLLInit(void)
{
  BSP_STATIC_ASSERT( sizeof( pll_Packet_t ) <= MAX_PLL_APP_FRAME );
  uint8_t i;

  // initialize the receive time stamp fifo elements
  // since the receive time stamps will never have their rxts pointer filled
  // in, we use this space as a flag for usage within the fifo.
  for( i = 0; i < NWK_PLL_RX_TIME_STAMP_FIFO_SZ; i++ ) // for each element
    sRxTimeStamps[i].rxts[0] = 0; // indicate it is not used
  
  // initialize logging of receive time stamps
  nwk_pllAcquireRxTimeStamp( );

  nwk_pllResetPLL( ); // reset the pll

  // install our 1 millisecond event manager
  BSP_CRITICAL_STATEMENT( spllExistingEvent = MRFI_Set_ms_Event( nwk_pll_1ms_event ) );

  return;
}

/******************************************************************************
 * @fn          nwk_pllIsLocked
 *
 * @brief       Call to determine if PLL referenced is locked.
 *
 * @return      true if PLL referenced consideres itself locked.
 */
bool nwk_pllIsLocked( uint8_t pll_num )
{
  return splls[pll_num].locked;
}

/******************************************************************************
 * @fn          nwk_pll_1ms_event
 *
 * @brief       This routine is called from the timer interrupt.
 *
 * @return   none.
 */
#if ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC8051__ )
__idata_reentrant void nwk_pll_1ms_event( void )
#else
void nwk_pll_1ms_event( void )
#endif
{
  if( spllExistingEvent != NULL ) // if an existing event to call
    spllExistingEvent( ); // call it
  
  spllmscnt++; // increment the millisecond counter
  
  return;
}

/******************************************************************************
 * @fn          nwk_pllGetChannelDelay
 *
 * @brief       Calculates the difference in the two time stamps provided
 *              and fills in the ChannelDelay member of the packet structure
 *              in the frame passed and the received time passed and puts the
 *              result into the ChannelDelay member of the frame passed.  If
 *              the ChannelDelay member is not large enough to hold the
 *              calculated value, the ChannelDelay member is then set to all
 *              F's to indicate overflow in the calculation.
 *              Finally, the rxTime reference is released so it is available
 *              for subsequent receive events to use it.
 *
 * @param       ts -- pointer to a pll packet structure
 *
 * @return   none.
 */
void nwk_pllGetChannelDelay( pll_Packet_t* ts )
{
  mrfi_Time_t* tx = &(ts->Time);
  mrfi_Time_t* rx = *((mrfi_Time_t**)&(tx->rxts));
  
  ts->misc <<= 1;
  if( tx->state.lsb.order_test != 0 )
    ts->misc |= tx->state.lsb.overflow;
  else
    ts->misc |= tx->state.msb.overflow;
  ts->misc <<= 1;
  if( rx->state.lsb.order_test != 0 )
    ts->misc |= rx->state.lsb.overflow;
  else
    ts->misc |= rx->state.msb.overflow;
  
  if( rx == NULL ) // if no receive time stamp to use
  {
    ts->ChannelDelay = NWK_PLL_NO_RXTS_AVAILABLE; // set error value
    return; // bail early
  }
  
  // calculate the channel delay time and put it back in rxts
  MRFI_GetTimeDelta( rx, tx, rx );

  // if channel delay value is within usable range
  if( (int32_t)(rx->milliseconds) < 0x7FFF
      && (int32_t)(rx->milliseconds) > -0x7FFF )
    ts->ChannelDelay = (int32_t)( ( rx->milliseconds << 16 ) | rx->timer );
  else // if channel delay is outside of usable range
    ts->ChannelDelay = NWK_PLL_SAMPLE_ERROR; // set invalid value
  
  // free up the receive time stamp for future use
  rx->rxts[0] = 0;

  return;
}

/******************************************************************************
 * @fn          nwk_pllAcquireRxTimeStamp
 *
 * @brief       Returns the pointer to the most recent time stamp received and
 *              updates the time stamp destination for the next receive
 *              operation.
 *
 * @return      none.
 */
mrfi_Time_t* nwk_pllAcquireRxTimeStamp( void )
{
  int i;
  mrfi_Time_t* tm = NULL; // assume failure initially
  bspIState_t s;
  
  // loop through the fifo elements
  for( i = 0; i < NWK_PLL_RX_TIME_STAMP_FIFO_SZ; i++ )
  {
    BSP_ENTER_CRITICAL_SECTION( s );
    if( sRxTimeStamps[i].rxts[0] == 0 ) // if a free one found
    {
      tm = sRxTimeStamps + i;
      tm->rxts[0] = 1; // indicate this one is in use
      BSP_EXIT_CRITICAL_SECTION( s );
      break;
    }
    BSP_EXIT_CRITICAL_SECTION( s );
  }

  // swap free time stamp with the one that was just filled in
  tm = MRFI_SetRxTimeStampAddr( tm );

  return tm; // return the time stamp most recently filled in.
}

/******************************************************************************
 * @fn          nwk_pllJumpPLL
 *
 * @brief       Jumps the PLL to a new state and resetst the control loop
 *              variables.  Used when the PLL initially locks to a new reference
 *              or has gotten lost and needs to quickly get back on top of
 *              things.
 *
 * @return   nothing
 */
void nwk_pllJumpPLL( mrfi_Time_t* t )
{
  bspIState_t s;
  pll_type* pll = splls;

  BSP_ENTER_CRITICAL_SECTION( s );
          
  // jam timer value to start, sets hop time and logical channel too
  MRFI_SetTime( t );
  pll->Integrator = 0; // clear the integrator
  pll->LastPreFilter  = 0; // clear the sample history

  sHopRate = MRFI_HOP_TIME_ms - 1; // set the hop rate to normal
          
  // reset the sample time
  pll->LastSampleTime = 0;

  // reset charge pump gains
  pll->ChargePumpCounter = pll->ChargePumpGain = NWK_PLL_SAMPLE_PERIOD_DEFAULT;

  BSP_EXIT_CRITICAL_SECTION( s );
  return;
}

/******************************************************************************
 * @fn          nwk_pllCheckSem
 *
 * @brief       Updates status of semaphores of the passed pll
 *
 * input parameters
 * @param       pll - pointer to a pll structure to operate on
 *
 * @return      Nothing
 */
static void nwk_pllCheckSem( pll_type* pll )
{
  /* count any millisecond operations since last time through the loop */
  /* no need to wrap the test in critical statement code as it is atomic */
  if( spllmscnt != 0 )
  {
    uint8_t ms_cnt;

    /* copy over elapsed milliseconds and clear counter */
    BSP_CRITICAL_STATEMENT( ms_cnt = spllmscnt; spllmscnt = 0 );

    pll->LastSampleTime += ms_cnt;
    if( pll->LastSampleTime < ms_cnt ) /* check for overflow */
      pll->LastSampleTime = 0x7FFFFFFFUL; /* saturate if needed */

    /* if the charge pump has timed out */
    pll->ChargePumpCounter -= ms_cnt;
    if( pll->ChargePumpCounter <= 0 )
    {
      /* reset the counter */
      pll->ChargePumpCounter += NWK_PLL_SAMPLE_PERIOD_DEFAULT;

      /* if the pll is on set the semaphore indicating a pump request is needed */
      if( pll->State != NWK_PLL_STATE_OFF )
        pll->PumpRequestSem = 1;
    }
  }
  return;
}

/******************************************************************************
 * @fn          nwk_pllChargePump
 *
 * @brief       Initiates a charge pump request.  Works in either locate
 *              or normal running operations
 *
 * input parameters
 * @param       pll - pointer to a pll structure to operate on
 *
 * @return      nothing
 */
void nwk_pllChargePump( pll_type* pll )
  {
  ioctlRawSend_t pump; // a pump request context
  pll_Packet_t packet; // the payload to send
  addr_t master_addr;
    
  // clear this pump request
  pll->PumpRequestSem = 0;
  master_addr = pll->MasterAddr;
    
  // fill in the transmit context structure
  pump.addr = &master_addr; // send request to the master reference (may be broadcast)
  pump.msg  = (uint8_t*)&packet; // send the modified packet
  pump.len  = sizeof( pll_Packet_t ); // only send the packet data
  pump.port = SMPL_PORT_PLL;
      
  // if the system is attempting to locate a reference
  if( pll->State == NWK_PLL_STATE_LOCATE )
  {
    // assign the locate reference command
    packet.Cmd = pll_cmd_LocateReference;

    // indicate total failure rate since we locating our reference
    pll->dead_band = 65535u;
  }
  else // if a normal pump request
  {
    packet.Cmd = pll_cmd_PumpRequest; // assign the pump request command
    pll->Failures++; // increment the failure counter, cleared when a response is seen
  }

  // if we are locked up to our reference clock
  if( pll->ChargePumpGain > NWK_PLL_LOCKED_VALUE )
    packet.Cmd |= pll_locked; // indicate so

  packet.Cmd |= NWK_PLL_ENTITY; // tell the requestee what we are

  packet.OriginatorAddr = *nwk_getMyAddress( ); // indicate where it is coming from
  
  // delay a random amount of a millisecond, add in offset to make sure
  // calculation time offset is not favored
  MRFI_DelayUsec( ( (int16_t)MRFI_RandomByte( ) << 2 )
                  + ( MRFI_RandomByte( ) & 3 ) + 250 );

  // align data elements in packet for packet transmission
  PLL_PKT_COMPRESS_TO_BUFFER( &packet );
      
  // send the request, don't worry if it fails, the pll application
  // expects lost requests on occaison.
  SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &pump );
  
  return;
}

/******************************************************************************
 * @fn          nwk_pllHandleRequestCommand
 *
 * @brief       Sends a response to a locate or pump request packet received
 *              over the radio link.
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       data - a pll pump packet structure
 * @param       addr - reference to originating radio
 *
 * @return      nothing
 */
void nwk_pllHandleRequestCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr )
{
  // if we are a viable reference clock but we have not yet locked to
  // another reference clock, don't allow ourselves to be used as a
  // reference clock.  By holding off this way, we guarantee that no
  // reference loops of alternate reference clocks can occur.
  // if we are not a viable reference clock
  if( ( ( NWK_PLL_ENTITY & pll_ent_reference ) == 0
      && ( ( NWK_PLL_ENTITY & pll_ent_alternate ) == 0
             || pll->ChargePumpGain <= NWK_PLL_LOCKED_VALUE ) ) )
  {
    // release the rx time stamp since we'r not responding
    (*((mrfi_Time_t**)&(data->Time.rxts)))->rxts[0] = 0;
  }
  else // we are a viable reference clock and the packet is for us
  {
    ioctlRawSend_t resp; // a response context
      
    data->misc = 0; // clear miscelaneous status data
          
    // calculate channel delay and release the rx time stamp
    nwk_pllGetChannelDelay( data );
    
    // if there was a receive time stamp then we can work on it
    if( data->ChannelDelay != NWK_PLL_NO_RXTS_AVAILABLE )
    {
      // determine how to address the response.  If the requestor is not locked
      // then use a broadcast address so others will see it allowing passive
      // operations to work and keep the network from being flooded
      if( ( data->Cmd & pll_locked ) == 0 ) // if requestor not locked
        resp.addr = (addr_t*) nwk_getBCastAddress( ); // use broadcast address
      else // if requestor locked
        resp.addr = addr; // send it back to the originating address

      // fill in remaining response context structure for transmitting
      resp.msg  = (uint8_t*)data; // resend the modified packet
      resp.len  = sizeof( pll_Packet_t ); // only send the packet data
      resp.port = SMPL_PORT_PLL;

      // assign the command response type
      if( ( data->Cmd & pll_cmd_LocateReference ) != 0 )
      {
        data->Cmd &= ~pll_cmd_LocateReference;
        data->Cmd |= pll_cmd_LocateResponse;
      }
      else
      {
        data->Cmd &= ~pll_cmd_PumpRequest;
        data->Cmd |= pll_cmd_PumpResponse;
      }
    
      data->Cmd &= ~( pll_ent_reference | pll_ent_alternate | pll_ent_slave );
      data->Cmd |= NWK_PLL_ENTITY; // tell the respondee what we are
    
      // Note that we have preserved the locked status of the requestor

      // delay a random amount of a millisecond, add in offset to make sure
      // calculation time offset is not favored
      MRFI_DelayUsec( ( (int16_t)MRFI_RandomByte( ) << 2 )
                     + ( MRFI_RandomByte( ) & 3 ) + 250 );

      // align data elements in packet for packet transmission
      PLL_PKT_COMPRESS_TO_BUFFER( data );
      
      // send the response, don't worry if it fails, the pll application
      // expects lost responses on occaison.
      SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &resp );
    }
  }
  return;
}

/******************************************************************************
 * @fn          nwk_pllLocateResponseCommand
 *
 * @brief       Handles responses to locate commands.
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       data - a pll pump packet structure
 * @param       addr - reference to originating radio
 *
 * @return      nothing
 */
void nwk_pllLocateResponseCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr )
{
  // attempt to find the best reference clock, if the network reference
  // clock responds then use it otherwise, use the next best thing which
  // will be either a range extender or an access point but choose the
  // one with the best LQI value so we are more likely to get responses
  // from pump request commands.
    
  // if a better reference appears to have been found
  if( pll->MasterLQI < data->LinkQuality)
  {
    pll->MasterAddr = *addr; // save its address
    pll->MasterLQI = data->LinkQuality; // save link quality
  }
    
  // if we are still attempting to locate a reference
  if( pll->State == NWK_PLL_STATE_LOCATE )
  {
    // get address priority status for later comparisons
    int8_t my_addr = memcmp( &data->OriginatorAddr, nwk_getMyAddress( ), MRFI_ADDR_SIZE );
      
    // if the received locate response originated from an address with higher
    // priority than ours
    if( my_addr < 0 )
    {
      // reset the charge pump counter in a manner which should see multiple
      // pump requests before it times out.  That way, we will continue to
      // operate in passive mode and not flood the network with our own requests
      pll->ChargePumpCounter = 2 * NWK_PLL_LOCKED_VALUE;

      // jump to the current time stamp to get us close
      nwk_pllJumpPLL( &(data->Time) );
      pll->dead_band = 2560; // assume a 10ms dead band window
    }
    else if( my_addr == 0 ) // if this is our locate request
    {
      sHopRate = MRFI_HOP_TIME_ms - 1; // set the hop rate to normal
      
      // indicate the pll is now running normally
      pll->State = NWK_PLL_STATE_ON;
    }
  }

  // release the rx time stamp, we don't use it on locate responses
  (*((mrfi_Time_t**)&(data->Time.rxts)))->rxts[0] = 0;
  
  return;
}

/******************************************************************************
 * @fn          nwk_pllCalcFailureRate
 *
 * @brief       Updates the failure rate measurment
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       err_sample - the error input for this sample
 *
 * @return      nothing
 */
void nwk_pllCalcFailureRate( pll_type* pll, int32_t err_sample )
{
  // update failure rate using a first order low pass filter
  // y[n] = y[n-1]*(1-k) + x[n]*k, 0 <= k <= 1
  // sample is in q16.16 format at this point but is twice as big
  // as needed because of differential sampling so divide k by 2,
  // result will be in q8.24 format which is adjusted to q24.8 by
  // the final shift and the cast results in a q8.8 result which
  // is what we started with.
  #define k 640 // 2.5%
  if( err_sample < 0 ) // get absolute value
    err_sample = -err_sample;
  err_sample >>= 8; // change to q25.7 format
  err_sample *= k / 2; // multiply by half of gain due to diff sampling, q8.24 format
  err_sample += (int32_t)(pll->dead_band) * ( 65535L - k ); // finish low pass filtering, q24.8 format
  err_sample >>= 16; // restore to q24.8 format
  pll->dead_band = (int16_t)err_sample; // truncate to q8.8 format
  #undef k
}

/******************************************************************************
 * @fn          nwk_pllCheckErrorThreshold
 *
 * @brief       Determines if the threshold window on the pll has been violated
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       err_sample - the error input for this sample
 *
 * @return      returns true if threshold is violated
 */
uint32_t nwk_pllCheckErrorThreshold( pll_type* pll, int32_t err_sample )
{
  int32_t result;
  
  if( pll->ChargePumpGain < NWK_PLL_HI_GAIN_THRESHOLD / 2 )
    result = NWK_PLL_HI_GAIN_WINDOW_CNT;
  else if ( pll->ChargePumpGain > NWK_PLL_LO_GAIN_THRESHOLD * 2 )
    result = NWK_PLL_LO_GAIN_WINDOW_CNT;
  else
  {
    result = pll->ChargePumpGain;
    result *= NWK_PLL_GAIN_INTERP_M_CNT;
    result >>= 8;
    result += NWK_PLL_GAIN_INTERP_B_CNT;
    if( result < NWK_PLL_LO_GAIN_WINDOW_CNT )
      result = NWK_PLL_LO_GAIN_WINDOW_CNT;
    else if( result > NWK_PLL_HI_GAIN_WINDOW_CNT )
      result = NWK_PLL_HI_GAIN_WINDOW_CNT;
  }

  return result;
}

/******************************************************************************
 * @fn          nwk_pllCalcNewSamplePeriod
 *
 * @brief       Determines the new sample period to use for the upcoming sample
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       err_sample - the error input for this sample
 *
 * @return      nothing
 */
void nwk_pllCalcNewSamplePeriod( pll_type* pll, int32_t err_sample )
{
  uint32_t gain = (uint32_t)pll->ChargePumpGain;
  uint32_t error_thresh;
  
  error_thresh = nwk_pllCheckErrorThreshold( pll, err_sample );

  pll->old_err = err_sample;

  // by adjusting the sample period we can minimize overhead when the PLL is
  // working well.  The algorithm below increases the sample period by 1.6% each
  // time the most recent sample falls within a specified threshold.  However, if
  // the sample falls outside the threshold then the sample period is reduced by
  // 6.4%, about four times faster than it increases.  This provides a quicker
  // response to threshold failures but will still bounce around the noise floor
  // of the system including any jitter caused by application latencies.  Thus,
  // the better the application layer manages keeping the interrupts available and
  // not locked out the less overhead the system on whole will incur due to PLL
  // packet traffic.
            
  // if the sample difference is greater than a defined threshold, decrease the
  // sample period so we can increase system gain.  The value below represents
  // 250us in q16.16 format but again that is twice the actual error so this
  // adjusts the sample period at +/- 125us boundaries

  // get magnitude value of error sample
  if( err_sample < 0 )
    err_sample = -err_sample;
  
  // for the special case of no error
  if( err_sample == 0 )
  {
    gain += gain >> 4; // increase sample time by 6.25%
    if( gain > NWK_PLL_SAMPLE_PERIOD_MAX ) // if we passed maximum period
      gain = NWK_PLL_SAMPLE_PERIOD_MAX; // set it to maximum
  }
  else
  {
    
    // if gain will be decreasing and it can be decreased
    if( error_thresh < err_sample && gain > NWK_PLL_SAMPLE_PERIOD_DEFAULT )
    {
      // calculate reciprocal (at least a close approximation)
      uint32_t r = 0xFFFFFFFFUL / (uint32_t)err_sample;
      // multiply to get avg_err/(2*error)
      // guaranteed to be less than one so only calculate the fractional part
      uint32_t p00 = ( r & 0xFFFFUL ) * ( error_thresh & 0xFFFFUL );
      uint32_t p10 = ( r & 0xFFFFUL ) * ( error_thresh >> 16 );
      uint32_t p01 = ( r >> 16 ) * ( error_thresh & 0xFFFFUL );
      uint32_t p11;

      r = p00 + ( ( p10 + p01 ) << 16 );
      
      // flatten the 1/x result somewhat
      r >>= 4;
      r |= 0xF0000000UL; // it should be close to 1
    
      // calculate new gain value
      p00 = ( r & 0xFFFFUL ) * ( gain & 0xFFFFUL );
      p10 = ( r & 0xFFFFUL ) * ( gain >> 16 );
      p01 = ( r >> 16 ) * ( gain & 0xFFFFUL );
      p11 = ( r >> 16 ) * ( gain >> 16 );
      // complete multiplication
      p00 = ( ( p00 >> 8 ) + 0x80 ) >> 8; // round
      p00 += ( p10 & 0xFFFFUL ) + ( p01 & 0xFFFFUL );
      p00 >>= 8;
      p10 >>= 8;
      p01 >>= 8;
      p11 += ( p00 + p10 + p01 ) >> 8;
      
      gain -= gain >> 3; // decrease existing gain by 12.5%
      if( gain < p11 ) // if actual decrease would be less than 12.5%
        gain = p11; // use calculated decrease rate, else limit to 12.5%
    
      // calculate new gain value
      if( gain < NWK_PLL_SAMPLE_PERIOD_DEFAULT ) // if sample period got too low
        gain = NWK_PLL_SAMPLE_PERIOD_DEFAULT; // reset it to minimum sample period rate
    }
  // if sample period is not at maximum then we can increase it so system overhead
  // is minimized and gain goes down making the pll more stable by being less
  // affected by noise.
    else if( error_thresh > err_sample && gain < NWK_PLL_SAMPLE_PERIOD_MAX )
    {
      // calculate reciprocal (at least a close approximation)
      uint32_t r = 0xFFFFFFFFUL / (uint32_t)err_sample;
      // multiply to get avg_err/(2*error)
      uint32_t p00 = ( r & 0xFFFFUL ) * ( error_thresh & 0xFFFFUL );
      uint32_t p10 = ( r & 0xFFFFUL ) * ( error_thresh >> 16 );
      uint32_t p01 = ( r >> 16 ) * ( error_thresh & 0xFFFFUL );
      uint32_t p11 = ( r >> 16 ) * ( error_thresh >> 16 );
      // finish with result in q8.24 format
      r = ( p00 + 0x80 ) >> 8; // round
      r += ( ( p10 & 0xFFFFUL ) << 8 ) + ( ( p01 & 0xFFFFUL ) << 8 );
      p11 += ( p10 >> 16 ) + ( p01 >> 16 ) + ( r >> 24 );
      r &= 0xFFFFFFUL;
      r |= p11 << 24;
      p11 >>= 8;
      
      // flatten 1/x function somewhat
      pll -= 0x01000000UL; // subtract off offset
      p11 >>= 4;           // scale the remaining value
      p11 += 0x01000000UL; // restore the offset
      
      p00 = ( r & 0xFFFFUL ) * ( gain & 0xFFFFUL );
      p10 = ( r & 0xFFFFUL ) * ( gain >> 16 );
      p01 = ( r >> 16 ) * ( gain & 0xFFFFUL );
      p11 = ( r >> 16 ) * ( gain >> 16 );
      // complete the calculation
      r = ( ( p00 >> 8 ) + 0x80 ) >> 8; // round
      r += ( p10 & 0xFFUL ) + ( p01 & 0xFFUL );
      p11 = ( p11 << 8 ) + ( p10 >> 8 ) + ( p01 >> 8 ) + ( r >> 8 );
      
      gain += gain >> 4; // calculate 6% increase
      if( gain > p11 ) // if actual increase is less than 6%
        gain = p11; // use calculated gain

      if( gain > NWK_PLL_SAMPLE_PERIOD_MAX ) // if we passed maximum period
        gain = NWK_PLL_SAMPLE_PERIOD_MAX; // set it to maximum
    }
    BSP_CRITICAL_STATEMENT( pll->ChargePumpGain = gain ); // update loop gain 
  }

  pll->ChargePumpCounter += pll->ChargePumpGain - NWK_PLL_SAMPLE_PERIOD_DEFAULT;

  return;
}

/******************************************************************************
 * @fn          nwk_pllRunControlLoop
 *
 * @brief       Runs the control loop on the pll
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       data - a pll pump packet structure
 * @param       err_sample - the error input for this sample
 *
 * @return      nothing
 */
void nwk_pllRunControlLoop( pll_type* pll, pll_Packet_t* data, int32_t err_sample )
{
  int32_t derivative;
  int32_t proportional;
  int32_t integral;
  int32_t control_effort;
  int32_t last_sample_time;

  nwk_pllCalcFailureRate( pll, err_sample );

  nwk_pllCalcNewSamplePeriod( pll, err_sample );
  
  // copy over the last sample time so it doesn't change during calculations
  // and then reset it
  BSP_CRITICAL_STATEMENT( last_sample_time = pll->LastSampleTime;
                          pll->LastSampleTime = 0 );


  // calculate control effort
  //y[n] = I + kp*To*x[n] + kd*(x[n] - x[n-1])   where I = I + ki*To^2*x[n]
              
  // calculate derivitive effort, q8.24 result
  derivative = err_sample - pll->LastPreFilter;
#if NWK_PLL_LOOP_GAIN_Kd_x != 0
  if( err_sample > 0 ) // if current sample is positive
  {
    if( pll->LastPreFilter < 0 ) // and previous sample is negative
      if( derivative < 0 ) // then check for overflow
        derivative = 0x7FFFFFFFL; // saturate on overflow detected
  }
  else // if current sample is negative
  {
    if( pll->LastPreFilter > 0 ) // and previous sample is positive
      if( derivative > 0 ) // then check for underflow
        derivative = -0x7FFFFFFFL; // saturate on underflow detected
  }

  // check for overflow from calculation
  if( derivative <= -0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Kd_x * ( ( NWK_PLL_LOOP_GAIN_Kd_x < 0 ) ? -1 : 1 )
      || derivative >= 0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Kd_x * ( ( NWK_PLL_LOOP_GAIN_Kd_x < 0 ) ? -1 : 1 ) )
    // saturate on underflow detected
    derivative = ( ( derivative < 0 ) ? -0x7FFFFFFFL : 0x7FFFFFFFL );
  else // if no overflow/underflow will occur
    derivative *= NWK_PLL_LOOP_GAIN_Kd_x; // use normal calculation
#else
  derivative = 0; // set to zero if gain is zero
#endif

  // calculate proportional effort, q8.24 result
  // check for overflow from calculation
#if NWK_PLL_LOOP_GAIN_Kp_x != 0
  if( err_sample <= -0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Kp_x * ( ( NWK_PLL_LOOP_GAIN_Kp_x < 0 ) ? -1 : 1 )
      || err_sample >= 0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Kp_x * ( ( NWK_PLL_LOOP_GAIN_Kp_x < 0 ) ? -1 : 1 ) )
    // saturate on overflow/underflow detected
    proportional = ( ( err_sample < 0 ) ? -0x7FFFFFFFL : 0x7FFFFFFFL );
  else // if no overflow/underflow will occur
    proportional = err_sample * NWK_PLL_LOOP_GAIN_Kp_x; // use normal calculation
#else
  proportional = 0; // set to zero if gain is zero
#endif
              
  if( pll->HoldOff != 0 )
  {
    pll->HoldOff--;
    pll->Integrator = integral = 0; // set to zero
  }
  else
  {
    // calculate integral effort, q8.24 result
    // check for overflow from calculation
#if NWK_PLL_LOOP_GAIN_Ki_x != 0
    if( err_sample <= -0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Ki_x * ( ( NWK_PLL_LOOP_GAIN_Ki_x < 0 ) ? -1 : 1 )
        || err_sample >= 0x7FFFFFFFL / NWK_PLL_LOOP_GAIN_Ki_x * ( ( NWK_PLL_LOOP_GAIN_Ki_x < 0 ) ? -1 : 1 ) )
      // saturate on underflow detected
      integral = ( ( err_sample < 0 ) ? -0x7FFFFFFFL : 0x7FFFFFFFL );
    else // if no overflow/underflow will occur
      integral = err_sample * NWK_PLL_LOOP_GAIN_Ki_x; // use normal calculation
    
    // scale the integrator term
    integral /= ( last_sample_time + 128 ) / 256 + 1;

    if( pll->Integrator > 0 ) // if integrator is positive
    {
      pll->Integrator += integral; // update integration value
      if( integral > 0 ) // and the additional value is positive then an overflow can occur
        if( pll->Integrator < 0 ) // if an overflow occurred
          pll->Integrator = 0x7FFFFFFFL; // saturate on overflow detected
    }
    else // if integrator is negative
    {
      pll->Integrator += integral; // update integration value
      if( integral < 0 ) // and the additional value is negative then an underflow can occur
        if( pll->Integrator > 0 ) // if an underflow occurred
          pll->Integrator = -0x7FFFFFFFL; // saturate on underflow detected
    }
              
    // saturate integrator if needed
    #if BSP_MAX_MODULATION_MAGNITUDE < 127
      if( pll->Integrator > ( (int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 ) )
        pll->Integrator = ((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 );
      else if( pll->Integrator < -((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 ) )
        pll->Integrator = -( (int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 );
    #endif
            
#else
    pll->Integrator = 0; // set to zero if gain is zero
#endif
  }              

  // update error history
  pll->LastPreFilter = err_sample;

  // calculate overall effort
  control_effort = proportional + derivative;
  if( proportional > 0 )
  {
    if( derivative > 0 && control_effort < 0 ) // if overflow occurred
      control_effort = 0x7FFFFFFFL; // saturate
  }
  else // proportional <= 0
  {
    if( derivative < 0 && control_effort >= 0 ) // if underflow occurred
      control_effort = -0x7FFFFFFFL; // saturate
  }

  // scale the proportional and derivative terms
  control_effort /= ( last_sample_time + 128 ) / 256 + 1;

  if( control_effort > 0 )
  {
    control_effort += pll->Integrator;
    if( pll->Integrator > 0 && control_effort < 0 ) // if overflow occurred
      control_effort = 0x7FFFFFFFL; // saturate
  }
  else // control_effort <= 0
  {
    control_effort += pll->Integrator;
    if( pll->Integrator < 0 && control_effort >= 0 ) // if underflow occurred
      control_effort = -0x7FFFFFFFL; // saturate
  }


  // saturate control effort if needed
  #if BSP_MAX_MODULATION_MAGNITUDE < 127
    if( control_effort > ((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 ) )
      control_effort = ((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 );
    else if( control_effort < -((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 ) )
      control_effort = -((int32_t)(BSP_MAX_MODULATION_MAGNITUDE) << 24 );
  #endif

  // effect the control effort
  MRFI_AdjustTimerModulationRate( control_effort );

  if( pll->ChargePumpGain > NWK_PLL_LOCKED_VALUE )
  {
    if( pll->locked == false )
      pll->locked = true;
  }
  else
  {
    if( pll->locked != false )
      pll->locked = false;
  }

  return;
}

/******************************************************************************
 * @fn          nwk_pllRunChargePump
 *
 * @brief       Runs the PLL charge pump and updates the control loop
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       data - a pll pump packet structure
 * @param       req_dly - the delay time of the request packet
 *
 * @return      nothing
 */
void nwk_pllRunChargePump( pll_type* pll, pll_Packet_t* data, int32_t req_dly )
{
  int32_t err_sample;

  pll->Failures = 0; // clear the failure counter
            
  // calculate the delta between request and response channel delays
  err_sample = data->ChannelDelay - req_dly;
  
  // update the average error distance value
  if( err_sample < 0 ) // use absolute value of error
    err_sample = -err_sample;
  if( pll->average_err > err_sample ) // add emphasis to decreasing average error
    pll->average_err += ( err_sample - pll->average_err ) >> 3;
  else // reduce emphasis to increasing average error
    pll->average_err += ( err_sample - pll->average_err ) >> 6;
  if( data->ChannelDelay < req_dly ) // correct sign change if needed
    err_sample = -err_sample;

  // if we are really out in the woods, basically if we are off by more
  // than +/- 32 milliseconds (because measured error is twice as large
  // as actual error due to differential measurement)
  if( err_sample > 0x3FFFFFL || err_sample < -0x3FFFFFL )
  {
    // then just jump to the current time stamp to get us close
    nwk_pllJumpPLL( &(data->Time) );

    // reset the sample time
    pll->LastSampleTime = 0;

    // reset charge pump gains
    pll->ChargePumpCounter = pll->ChargePumpGain = NWK_PLL_SAMPLE_PERIOD_DEFAULT;

    // reset the modulation value
    MRFI_AdjustTimerModulationRate( 0 );

    pll->dead_band = 2560; // assume a 10ms dead band window
  }
  else // we are within range to run the regular control loop
  {
    nwk_pllRunControlLoop( pll, data, err_sample );
  }
  return;
}

/******************************************************************************
 * @fn          nwk_pllPumpResponseCommand
 *
 * @brief       Handles responses to pump request commands.
 *
 * input parameters
 * @param       pll  - pointer to a pll structure to operate on
 * @param       data - a pll pump packet structure
 * @param       addr - the address of the responding reference
 *
 * @return      nothing
 */
void nwk_pllPumpResponseCommand( pll_type* pll, pll_Packet_t* data, addr_t* addr )
{
  // get priority of entity originating pump request
  int8_t my_addr = memcmp( &data->OriginatorAddr, nwk_getMyAddress( ), MRFI_ADDR_SIZE );

  // if we're not locked and another system has priority over us
  if( pll->ChargePumpGain <= NWK_PLL_LOCKED_VALUE && my_addr < 0 )
  {
    // then just jump to the current time stamp to get us close
    nwk_pllJumpPLL( &(data->Time) );

    pll->dead_band = 2560; // assume a 10ms deadband window
          
    // reset the charge pump counter in a manner which should see multiple
    // pump requests before it times out.  That way, we will continue to
    // operate in passive mode and not flood the network with our own requests
    pll->ChargePumpCounter = 2 * NWK_PLL_LOCKED_VALUE;
    
    // release the rx time stamp, we're done
    (*((mrfi_Time_t**)&(data->Time.rxts)))->rxts[0] = 0;
  }
  else if( my_addr == 0 ) // if this response is for me
  {
    // if the channel delay from the pump request is invalid
    if( data->ChannelDelay == NWK_PLL_SAMPLE_ERROR )
    {
      // then just jump to the current time stamp to get us close
      nwk_pllJumpPLL( &(data->Time) );

      pll->dead_band = 2560; // assume a 10ms deadband window
          
      // release the rx time stamp, we're done
      (*((mrfi_Time_t**)&(data->Time.rxts)))->rxts[0] = 0;
    }
    // if the channel delay from the pump request is valid
    else
    {
      // copy request channel delay so we don't loose it
      int32_t req_dly = data->ChannelDelay;

      // calculate channel delay for response packet, releases rx time stamp
      nwk_pllGetChannelDelay( data );
      
      // if the delay from the pump response is valid too and its from our
      // selected reference
      if( data->ChannelDelay != NWK_PLL_SAMPLE_ERROR
                && data->ChannelDelay != NWK_PLL_NO_RXTS_AVAILABLE )
        nwk_pllRunChargePump( pll, data, req_dly );
    }
  }
  else // not anything we need to work on, release the rxts
    (*((mrfi_Time_t**)&(data->Time.rxts)))->rxts[0] = 0;

  return;
}
/******************************************************************************
 * @fn          nwk_pllBackgrounder
 *
 * @brief       User must call this routine regularly from thier main loop and
 *              also from any functions not working with the radio which can
 *              take a long time to execute.  The more often the better.
 *              NOTE: this routine runs off of semaphores that are set by the
 *                    1 millisecond timer interrupt and also by radio interrupts
 *                    but it should not be run in the ISR's because it needs to
 *                    manage packets over the radio and that is inappropriate
 *                    to do in an ISR thread.
 *
 * input parameters
 * @param       no_pump - true if user is wanting to hold off pump requests
 *
 * @return   non-zero if the PLL considers itself locked to a reference.
 */
#ifdef NWK_PLL_SHOW_LOCATION_INDICATORS
static uint8_t nwk_pll_indicator_state = 0; // initialize indicator state
#endif
bool nwk_pllBackgrounder( bool no_pump )
{
  static bool recurse = false;
  pll_type* pll = splls;
  
  if( recurse == false )
  {
    recurse = true; // assert recursion flag
    
    nwk_pllCheckSem( pll ); // update semaphore status
  
    FHSS_ACTIVE( MRFI_FreqHoppingBckgndr( ) ); // manage changing of frequencies

    if( no_pump == false )
    {
      recurse = true; // assert recursion flag
    
      // if time to run the charge pump
      if( pll->PumpRequestSem != 0 )
      {
        nwk_pllChargePump( pll );
#ifdef NWK_PLL_SHOW_LOCATION_INDICATORS
        if( splls->State == NWK_PLL_STATE_LOCATE )
        {
          if( nwk_pll_indicator_state == 0 )
          {
            nwk_pll_indicator_state = 4;
            nwk_pll_indicator_state |= ( ( BSP_LED1_IS_ON( ) ) ? 1 : 0 );
            nwk_pll_indicator_state |= ( ( BSP_LED2_IS_ON( ) ) ? 2 : 0 );
          }

          if( BSP_LED1_IS_ON( ) )
            if( BSP_LED2_IS_ON( ) )
              BSP_TURN_OFF_LED1( );
            else
              BSP_TURN_ON_LED2( );
          else
            if( BSP_LED2_IS_ON( ) )
              BSP_TURN_OFF_LED2( );
            else
              BSP_TURN_ON_LED1( );
        }
        else if( nwk_pll_indicator_state != 0 )
        {
          if( ( nwk_pll_indicator_state & 1 ) == 0 )
            BSP_TURN_ON_LED1( );
          else
            BSP_TURN_OFF_LED1( );
          if( ( nwk_pll_indicator_state & 2 ) == 0 )
            BSP_TURN_ON_LED2( );
          else
            BSP_TURN_OFF_LED2( );
          nwk_pll_indicator_state = 0;
        }
#endif
      }

      // if there are one or more packets to work on
      if( pll->packetSem != 0 )
      {
        pll_Packet_t pkt;
        addr_t addr;
        ioctlRawReceive_t rcv;
    
        // indicate we are consuming a packet
        BSP_CRITICAL_STATEMENT( pll->packetSem-- );
    
        // fill in the receive context
        rcv.addr = &addr;
        rcv.msg  = (uint8_t*)(&pkt);
        rcv.port = SMPL_PORT_PLL;
  
        // go get the frame and if it was found
        if( SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &rcv ) == SMPL_SUCCESS )
        {
          // if this is a locate reference or pump request command
          if( ( pkt.Cmd & ( pll_cmd_LocateReference | pll_cmd_PumpRequest ) ) != 0 )
            nwk_pllHandleRequestCommand( pll, &pkt, &addr );
      
          // if a locate response command
          else if( ( pkt.Cmd & pll_cmd_LocateResponse ) != 0 )
            nwk_pllLocateResponseCommand( pll, &pkt, &addr );

          // if a pump response command and the PLL is running
          else if( ( pkt.Cmd & pll_cmd_PumpResponse ) != 0
                   && pll->State == NWK_PLL_STATE_ON )
            nwk_pllPumpResponseCommand( pll, &pkt, &addr );

          else // an unknown command, release the receive data packet, we don't need it
            (*((mrfi_Time_t**)&(pkt.Time.rxts)))->rxts[0] = 0;
        }
      }

      // finally, if we're supposed to be running, check to see if we have
      // lost the reference clock completely and if so, restart the system
      if( pll->State == NWK_PLL_STATE_ON && pll->Failures > NWK_PLL_FAILURE_LIMIT )
      {
        nwk_pllResetPLL( ); // reset the system
        nwk_pllStartPLL( ); // now start it back up
      }
    }
    
    recurse = false; // reset recursion flag
  }
  
  return sTxValid;
}

/****************************************************************************************
 * @fn          nwk_pllStartPLL
 *
 * @brief       Starts up the pll if it is not already running
 *
 * input parameters
 *              none
 *
 * @return      nothing
 */
void nwk_pllStartPLL( void )
{
  pll_type* pll = splls;

  if( pll->State != NWK_PLL_STATE_ON ) // if we really need to start the pll
  {
    if( pll->State != NWK_PLL_STATE_OFF ) // if the system is not already off
      nwk_pllResetPLL( ); // put the pll into a known state

    pll->State = NWK_PLL_STATE_LOCATE; // move to locating reference clock state
    
    // set hop rate to be quick so we cycle through frequencies faster than
    // the normal hop rate so we can catch up with the reference clock.
    sHopRate = 3 * NWK_PLL_SAMPLE_PERIOD_DEFAULT;
  }

  return;
}

/****************************************************************************************
 * @fn          nwk_pllResetPLL
 *
 * @brief       Completes necessary dynamic reset operations.
 *
 * input parameters
 *              none
 *
 * @return      nothing
 */
void nwk_pllResetPLL( void )
{
  pll_type* pll = splls;
  
  pll->State = NWK_PLL_STATE_OFF;
    
#ifdef NWK_PLL_REFERENCE_CLOCK
  // initialize the master reference to our own address
  pll->MasterAddr = *nwk_getMyAddress( );
#else
  // initialize the master reference to the broadcast address
  pll->MasterAddr = *nwk_getBCastAddress( );
#endif
  
  pll->MasterLQI = 0; // set LQI to minimum to start
  
  // reset the sample time
  pll->LastSampleTime = 0;

  // reset charge pump gains
  pll->ChargePumpCounter = pll->ChargePumpGain = NWK_PLL_SAMPLE_PERIOD_DEFAULT;
  // randomly backoff on first access
  pll->ChargePumpCounter += MRFI_RandomByte( );

#ifdef NWK_PLL_REFERENCE_CLOCK
  pll->ChargePumpCounter = 1; // short the pump count to minimum for the reference
#endif

  // clear the integrator
  pll->Integrator = 0;

  // initialize integrator holdoff
  pll->HoldOff = NWK_PLL_Kp_HOLD_OFF;

  // clear the sample history
  pll->LastPreFilter = 0;

  // initialize the semaphores    
  pll->packetSem = 0;     // no packets yet
  pll->PumpRequestSem = 0; // no charge pump requests yet

  pll->Failures = 0;
  
#ifdef NWK_PLL_REFERENCE_CLOCK
  pll->dead_band = 0; // reference clocks are always in sync
#else
  pll->dead_band = 65535u; // initially we always fail
#endif

  pll->locked = 0;
  pll->old_err = 0;

  return;
}

/****************************************************************************************
 * @fn          nwk_pllControl
 *
 * @brief       handles IOCTL requests
 *
 * input parameters
 * @param   action  - requested action
                        IOCTL_ACT_ON & IOCTL_ACT_OFF are only supported actions
 * @param   val     - ignored, pass NULL
 *
 * output parameters
 * @param   val     - not modified
 *
 * @return   status of operation.
 */
smplStatus_t nwk_pllControl(ioctlAction_t action, void *val)
{
  smplStatus_t status = SMPL_SUCCESS; // assume success initially

  switch( action )
  {
  case IOCTL_ACT_ON:
    // start the charge pump by issuing a pump request
#ifndef NWK_PLL_REFERENCE_CLOCK
    nwk_pllStartPLL( );
#endif
    break;
  case IOCTL_ACT_OFF:
    nwk_pllResetPLL( );
    break;
  default:
    status = SMPL_BAD_PARAM;
    break;
  }

  return status;
}

/****************************************************************************************
 * @fn          nwk_processPLL
 *
 * @brief       Process a PLL application frame.
 *
 * input parameters
 * @param   frame     - pointer to frame
 *
 * @return   Disposition for frame: release (FHS_RELEASE) or keep (FHS_KEEP).
 */
fhStatus_t nwk_processPLL( mrfiPacket_t *frame )
{
  pll_type* pll = splls;
  mrfi_Time_t* rxts = nwk_pllAcquireRxTimeStamp( );

  // if this packet is for us (in case address filtering is off)
  if( memcmp( MRFI_P_DST_ADDR( frame ), nwk_getBCastAddress( ), sizeof( addr_t ) ) == 0
      || memcmp( MRFI_P_DST_ADDR( frame ), nwk_getMyAddress( ), sizeof( addr_t ) ) == 0 )
  {
    if( rxts == NULL ) // if no receive time stamp to go with this packet
      return FHS_RELEASE; // ignore the packet

    // we have a receive time stamp and the packet is for us

    // expand raw packet into aligned packet
    PLL_PKT_EXPAND_FROM_BUFFER( NWK_PLL_PACKET_PTR(frame) );
         
    // put reference to last receive event time stamp into received data
    memcpy( NWK_PLL_PACKET_PTR( frame )->Time.rxts, &rxts, sizeof(mrfi_Time_t*));
  
    // copy the link quality value into the packet
    memcpy( &(NWK_PLL_PACKET_PTR( frame )->LinkQuality),
            &(frame->rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS]),
            PLL_PKT_MEMBER_SZ(LinkQuality) );
  
    // indicate we have a packet to manage
    BSP_CRITICAL_STATEMENT( pll->packetSem++ );
    
    return FHS_KEEP; // keep it so we can work on it in the backgrounder
  }

  // else  
  // release the receive data packet as it's not for us
  if( rxts != NULL ) // if a receive time stamp to go with this packet
    rxts->rxts[0] = 0; // release it

  return FHS_RELEASE; // ignore the packet
}

#else  /* NWK_PLL */

/****************************************************************************************
 * @fn          nwk_processPLL
 *
 * @brief       Process a NWK PLL application frame. Stub when NWK_PLL not defined.
 *              This stub simply releases all frames for this port as there is
 *              nothing for us to do.
 *
 * input parameters
 * @param   frame     - pointer to frame
 *
 * @return   Disposition for frame: release (FHS_RELEASE).
 */
fhStatus_t nwk_processPLL(mrfiPacket_t *frame)
{
  return FHS_RELEASE;
}

#endif  /* NWK_PLL */
