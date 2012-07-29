/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

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

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Include file for all MRFI services.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_H
#define MRFI_H


/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stddef.h>
#include <stdbool.h>
#include "bsp.h"
#include "mrfi_defs.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_NUM_LOGICAL_CHANS           __mrfi_NUM_LOGICAL_CHANS__

#define MRFI_NUM_POWER_SETTINGS          __mrfi_NUM_POWER_SETTINGS__

/* return values for MRFI_Transmit */
#define MRFI_TX_RESULT_SUCCESS        0
#define MRFI_TX_RESULT_FAILED         1

/* transmit type parameter for MRFI_Transmit */
#define MRFI_TX_TYPE_FORCED           0
#define MRFI_TX_TYPE_CCA              1

#define MRFI_PAYLOAD_OFFSET                 __mrfi_PAYLOAD_OFS__
#define MRFI_LENGTH_FIELD_OFS               __mrfi_LENGTH_FIELD_OFS__
#define MRFI_LENGTH_FIELD_SIZE              __mrfi_LENGTH_FIELD_SIZE__
#define MRFI_HEADER_SIZE                    __mrfi_HEADER_SIZE__
#define MRFI_FRAME_BODY_OFS                 __mrfi_DST_ADDR_OFS__

#define MRFI_BACKOFF_PERIOD_USECS           __mrfi_BACKOFF_PERIOD_USECS__

/* rx metrics definitions, known as appended "packet status bytes" in datasheet parlance */
#define MRFI_RX_METRICS_CRC_OK_MASK         __mrfi_RX_METRICS_CRC_OK_MASK__
#define MRFI_RX_METRICS_LQI_MASK            __mrfi_RX_METRICS_LQI_MASK__

/* Max time we can be in a critical section within the delay function.
 * This could be fine-tuned by observing the overhead is calling the bsp delay
 * function. The overhead should be very small compared to this value.
 * Note that the max value for this must be less than 19 usec with the
 * default CLKCON.TICKSPD and CLKCON.CLOCKSPD settings and external 26 MHz
 * crystal as a clock source (which we use).
 *
 * Be careful of direct calls to Mrfi_DelayUsec().
 */
#define MRFI_MAX_DELAY_US 16 /* usec */


/* Network header size definition */
/* *********************************  NOTE  ****************************************
 * There is a dependency here that really shouldn't exist. A reimplementation
 * is necessary to avoid it.
 *
 * MRFI allocates the frame buffer which means it needs to know at compile time
 * how big the buffer is. This means in must know the NWK header size, the
 * maximum NWK and User application payload sizes and whether Security is enabled.
 * ********************************************************************************
 */
#ifndef SMPL_SECURE
#define  NWK_HDR_SIZE   3
#define  NWK_PAYLOAD    MAX_NWK_PAYLOAD
#else
#define  NWK_HDR_SIZE   6
#define  NWK_PAYLOAD    (MAX_NWK_PAYLOAD+4)
#endif

/* if external code has defined a maximum payload, use that instead of default */
#ifdef MAX_APP_PAYLOAD
#ifndef MAX_NWK_PAYLOAD
#error ERROR: MAX_NWK_PAYLOAD not defined
#endif
#if MAX_APP_PAYLOAD < NWK_PAYLOAD
#define MAX_PAYLOAD  NWK_PAYLOAD
#else
#define MAX_PAYLOAD  MAX_APP_PAYLOAD
#endif
#define MRFI_MAX_PAYLOAD_SIZE  (MAX_PAYLOAD+NWK_HDR_SIZE) /* SimpliciTI payload size plus six byte overhead */
#endif


/* frame definitions */
#define MRFI_ADDR_SIZE              __mrfi_ADDR_SIZE__
#ifndef MRFI_MAX_PAYLOAD_SIZE
#define MRFI_MAX_PAYLOAD_SIZE       __mrfi_MAX_PAYLOAD_SIZE__
#endif
#define MRFI_MAX_FRAME_SIZE         (MRFI_MAX_PAYLOAD_SIZE + __mrfi_FRAME_OVERHEAD_SIZE__)
#define MRFI_RX_METRICS_SIZE        __mrfi_RX_METRICS_SIZE__
#define MRFI_RX_METRICS_RSSI_OFS    __mrfi_RX_METRICS_RSSI_OFS__
#define MRFI_RX_METRICS_CRC_LQI_OFS __mrfi_RX_METRICS_CRC_LQI_OFS__

/* Radio States */
#define MRFI_RADIO_STATE_UNKNOWN  0
#define MRFI_RADIO_STATE_OFF      1
#define MRFI_RADIO_STATE_IDLE     2
#define MRFI_RADIO_STATE_RX       3

/* Platform constant used to calculate worst-case for an application
 * acknowledgment delay. Used in the NWK_REPLY_DELAY() macro.
 *

                                      processing time on peer
                                      |   round trip
                                      |   |      max number of replays
                                      |   |      |             number of backoff opportunities
                                      |   |      |             |         average number of backoffs
                                      |   |      |             |         |                                    */
#define   PLATFORM_FACTOR_CONSTANT   (2 + 2*(MAX_HOPS*(MRFI_CCA_RETRIES*(8*MRFI_BACKOFF_PERIOD_USECS)/1000)))

/* Frequency hopping requires the timer always be running for the network PLL
 * to operate properly.
 */
#if defined FREQUENCY_HOPPING && !defined MRFI_TIMER_ALWAYS_ACTIVE
  #define MRFI_TIMER_ALWAYS_ACTIVE
#endif

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  /* The number of bytes used to represent time in the time array.  This value controls
   * the maximum time the system can represent.  A minimum size of 5 is required so that
   * calls to MRFI_DelayMs are not incorrectly timed.
   * The following table indicates limits for a given MRFI_TIME_SIZE value
   * MRFI_TIME_SIZE   Maximum time representation
   *      5 bytes   =   4 hours 39 minutes 37.216 seconds
   *      6 bytes   =   49 days 17 hours 2 minutes 47.296 seconds
   *      7 bytes   =   34 years 307 days 7 hours 53 minutes 47.776 seconds
   *      8 bytes   =   8919 years 147 days 11 hours 31 minutes 50.656 seconds
   *      where 1 year = 365.25 days and century leap years are ignored.
   */
  #ifndef MRFI_TIME_SIZE
    #define MRFI_TIME_SIZE 6
  #endif
  #if MRFI_TIME_SIZE < 5
    #error "ERROR: MRFI_TIME_SIZE must be at least a value of 5."
  #endif

  // the number of bytes in the hop count array in the mrfi_Time_t structure
  #define MRFI_HOP_COUNT_SIZE 2

/* the number of bytes the physical timer represents in the time array.
 * NOTE: the timer size is 2 for both 8 and 16 bit timers but may need to be
 *       different for larger timers if we ever encounter them.
 */
  #if BSP_TIMER_SIZE == 8 || BSP_TIMER_SIZE == 16
    #define MRFI_TIMER_SZ 2
  #endif
  #define MRFI_ROLLOVER_LIMIT  ( BSP_ROLLOVER_LIMIT - 1 )
  #define MRFI_ROLLOVER_EXTRAS BSP_ROLLOVER_EXTRAS
  #define MRFI_ROLLOVER_ROLLOVERS  BSP_ROLLOVER_ROLLOVERS
#endif
#ifdef FREQUENCY_HOPPING
  #ifndef MRFI_HOP_TIME_ms
    #define MRFI_HOP_TIME_ms 400
  #endif
  #if MRFI_HOP_TIME_ms < 20
    #error "ERROR: MRFI_HOP_TIME_ms must be greater than or equal to 20."
  #endif
  #define FHSS_HOP_MARGIN 4 /* 4ms of margin */
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_GET_PAYLOAD_LEN(p)         __mrfi_GET_PAYLOAD_LEN__(p)
#define MRFI_SET_PAYLOAD_LEN(p,x)       __mrfi_SET_PAYLOAD_LEN__(p,x)

#define MRFI_P_DST_ADDR(p)              __mrfi_P_DST_ADDR__(p)
#define MRFI_P_SRC_ADDR(p)              __mrfi_P_SRC_ADDR__(p)
#define MRFI_P_PAYLOAD(p)               __mrfi_P_PAYLOAD__(p)

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
#define Mrfi_TimerInit( )   BSP_TIMER_FREE_RUN_INIT( )
#define MRFI_DelayMs( ms ) Mrfi_DelayUsecLong( ms, 0, NULL )
#define MRFI_DelayUsec( us ) Mrfi_DelayUsecLong( 0, us, NULL)
#define MRFI_WaitTimeoutMs( ms, term ) Mrfi_DelayUsecLong( ms, 0, term)
#define MRFI_WaitTimeoutUsec( us, term ) Mrfi_DelayUsecLong( 0, us, term)
#endif

// calculates the offset of the member m in the structure of type t
#define MRFI_OFFSET_OF( t, m ) ((int)((char*)(&(((t*)0)->m))))

// copies the source object at s to the destination object at d only if
// the addresses are different
#define MRFI_MOVE_ONLY_IF_NEEDED(d, s) ( ((d)!=(s)) ? (*(d))=(*(s)) : 0 )
// copies objects regardless of alignment, d = destination, s = source,
// t = type or variable suitable for application of sizeof operator
// if high to low flag (h) is non zero, copy order is high addresses before
// low addresses, if zero, copy order is low addresses before high addresses
// current implementation assumes largest size object is 4, i.e. (u)int32_t
#define MRFI_MOVE_UNALIGNED( d, s, t, h ) \
   ( ( h == 0 ) \
     ? (     MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+0, (uint8_t*)(s)+0),       \
         ( ( sizeof(t) > 1 ) \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+1, (uint8_t*)(s)+1) : 0 ), \
         ( ( sizeof(t) > 2 ) \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+2, (uint8_t*)(s)+2) : 0 ), \
         ( ( sizeof(t) > 3 ) \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+3, (uint8_t*)(s)+3) : 0 ) )\
     : ( ( ( sizeof(t) > 3 ) /* h != 0 */ \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+3, (uint8_t*)(s)+3) : 0 ), \
         ( ( sizeof(t) > 2 ) \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+2, (uint8_t*)(s)+2) : 0 ), \
         ( ( sizeof(t) > 1 ) \
           ? MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+1, (uint8_t*)(s)+1) : 0 ), \
             MRFI_MOVE_ONLY_IF_NEEDED((uint8_t*)(d)+0, (uint8_t*)(s)+0) ) )
                                          

/* ------------------------------------------------------------------------------------------------
 *                                          Typdefs
 * ------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8_t frame[MRFI_MAX_FRAME_SIZE];
  uint8_t rxMetrics[MRFI_RX_METRICS_SIZE];
} mrfiPacket_t;

/* the TimeoutTerminator_t type is the prototype used for functions that test
 * semaphores and are passed to the MRFI_WaitTimeoutUsec and MRFI_WaitTimeoutMS
 * functions.
 */
  typedef bool (*TimeoutTerminator_t)( void );

#ifdef MRFI_TIMER_ALWAYS_ACTIVE

/* the MRFI_ms_event_t type is the prototype of functions that respond to the 1 ms
 * event generator.  Functions of this type will be run in the ISR so care must be
 * taken to ensure the system is not held off for long periods.
 */
#if ( defined __IAR_SYSTEMS_ICC__ ) && ( defined __ICC8051__ )
  typedef void (__idata_reentrant *MRFI_ms_event_t)( void );
#else
  typedef void ( *MRFI_ms_event_t)( void );
#endif

/* This structure holds a time stamp reference.  The time reference can either be in
 * raw or cooked format, which form is determined by the state of the raw member.
 * If the raw member is non zero, the format is raw, otherwise the format is "cooked".
 * in raw format, the lowest "physical_offset" bytes in the time member array hold the
 * captured timer value.  In "cooked" format, the lower "physical_offset" bytes of the
 * time member array hold the fractional portion of a millisecond in 65536ths of a
 * millisecond.
 */

typedef struct
{
  uint8_t order_test    : 1; // used to determine bit field ordering
  uint8_t phy_offset    : 3; // indicates the offset from 0 for the physical timer bytes
  uint8_t overflow      : 1; // indicates the timer overflowed during capture
  uint8_t raw           : 1; // indicates the time stamp is raw or cooked
  uint8_t little_endian : 1; // indicates the numeric values are in little endian format
  uint8_t order_other   : 1; // used for symmetrical placement in order test
} mrfi_time_state_lsb_to_msb_t;

typedef struct
{
  uint8_t order_other   : 1; // used for symmetrical placement in order test
  uint8_t little_endian : 1; // indicates the numeric values are in little endian format
  uint8_t raw           : 1; // indicates the time stamp is raw or cooked
  uint8_t overflow      : 1; // indicates the timer overflowed during capture
  uint8_t phy_offset    : 3; // indicates the offset from 0 for the physical timer bytes
  uint8_t order_test    : 1; // used to determine bit field ordering
} mrfi_time_state_msb_to_lsb_t;

typedef union
{
  mrfi_time_state_lsb_to_msb_t lsb;
  mrfi_time_state_msb_to_lsb_t msb;
} mrfi_time_state_t;

typedef struct
{
  uint32_t milliseconds; // represents the millisecond count
  uint16_t limit; // represents the count limit of the timer referred to
  uint16_t timer; // represents the physical timer value
  uint16_t hopCount; // holds the current time left before changing frequencies in ms
  uint8_t rxts[4];   // points to receive time stamp, filled in on reception of a frame
  uint8_t logicalChnl; // holds the current logical frequency channel being used
  uint8_t rollover;  // indicates the sub-rollover rate of an 8 bit timer
  uint8_t threshold; // the threshold of the rollover member where added counts exist
  mrfi_time_state_t state;
} mrfi_Time_t;

#define MRFI_TIME_MEMBER_SZ( m )                sizeof(((mrfi_Time_t*)0)->m)
#define MRFI_TIME_MEMBER_STRUCT_OFS( m )        MRFI_OFFSET_OF( mrfi_Time_t, m )
#define MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE( m, n ) \
             ((MRFI_TIME_MEMBER_STRUCT_OFS(n) < MRFI_TIME_MEMBER_STRUCT_OFS(m))\
                                                   ? MRFI_TIME_MEMBER_SZ(n) : 0)
#define MRFI_TIME_MEMBER_BUFFER_OFS( m ) \
                     ( ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, milliseconds) ) \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, limit) )        \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, timer) )        \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, hopCount) )     \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, rxts) )         \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, logicalChnl) )  \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, rollover) )     \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, threshold) )    \
                     + ( MRFI_TIME_MEMBER_BUFFER_SZ_INCLUDE(m, state) )        )
#define MRFI_TIME_BUFFER_SZ   ( MRFI_TIME_MEMBER_BUFFER_OFS(state) \
                                + MRFI_TIME_MEMBER_SZ(state) )
#define MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d, m ) \
          MRFI_MOVE_UNALIGNED( (uint8_t*)(d) + MRFI_TIME_MEMBER_STRUCT_OFS(m), \
                               (uint8_t*)(d) + MRFI_TIME_MEMBER_BUFFER_OFS(m), \
                               (d)->m, 1 /* copy high to low */ )
#define MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d, m ) \
          MRFI_MOVE_UNALIGNED( (uint8_t*)(d) + MRFI_TIME_MEMBER_BUFFER_OFS(m), \
                               (uint8_t*)(d) + MRFI_TIME_MEMBER_STRUCT_OFS(m), \
                               (d)->m, 0 /* copy low to high */ )
#define MRFI_TIME_EXPAND_FROM_BUFFER( d ) \
                   ( MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,        state), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,    threshold), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,     rollover), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,  logicalChnl), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,         rxts), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,     hopCount), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,        timer), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d,        limit), \
                     MRFI_TIME_EXPAND_MEMBER_FROM_BUFFER( d, milliseconds)  )
#define MRFI_TIME_COMPRESS_TO_BUFFER( d ) \
                   ( MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d, milliseconds), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,        limit), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,        timer), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,     hopCount), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,         rxts), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,  logicalChnl), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,     rollover), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,    threshold), \
                     MRFI_TIME_COMPRESS_MEMBER_TO_BUFFER( d,        state)  )

extern uint16_t sHopCount;
extern uint8_t sLogicalChannel;
#endif // MRFI_TIMER_ALWAYS_ACTIVE
FHSS_ACTIVE( extern uint16_t sHopRate );
FHSS_ACTIVE( extern bool sTxValid );

/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void    MRFI_Init(void);
uint8_t MRFI_Transmit(mrfiPacket_t *, uint8_t);
void    MRFI_Receive(mrfiPacket_t *);
void    MRFI_RxCompleteISR(void); /* populated by code using MRFI */
uint8_t MRFI_GetRadioState(void);
void    MRFI_RxOn(void);
void    MRFI_RxIdle(void);
int8_t  MRFI_Rssi(void);
void    MRFI_SetLogicalChannel(uint8_t);
uint8_t MRFI_SetRxAddrFilter(uint8_t *);
void    MRFI_EnableRxAddrFilter(void);
void    MRFI_DisableRxAddrFilter(void);
void    MRFI_Sleep(void);
void    MRFI_WakeUp(void);
uint8_t MRFI_RandomByte(void);
#ifndef MRFI_TIMER_ALWAYS_ACTIVE
void    MRFI_DelayMs(uint16_t);
#endif // !MRFI_TIMER_ALWAYS_ACTIVE
void    MRFI_ReplyDelay(void);
void    MRFI_PostKillSem(void);
void    MRFI_SetRFPwr(uint8_t);

bool Mrfi_DelayUsecLong( uint32_t, uint16_t, TimeoutTerminator_t );
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
void MRFI_GetLocalRawTime( mrfi_Time_t* time );
void MRFI_CookTime( mrfi_Time_t* time );
void MRFI_GetTimeDelta( mrfi_Time_t* dest, mrfi_Time_t* start, mrfi_Time_t* end );
void MRFI_SetTime( mrfi_Time_t* t );
MRFI_ms_event_t MRFI_Set_ms_Event( MRFI_ms_event_t evt_func );
MRFI_ms_event_t MRFI_Get_ms_Event( void );
void MRFI_AdjustTimerModulationRate( int32_t delta );
void MRFI_FreqHoppingBckgndr( void );
mrfi_Time_t* MRFI_SetRxTimeStampAddr( mrfi_Time_t* t );
void MRFI_SetTxTimeStampAddr( mrfi_Time_t* t );
#endif // MRFI_TIMER_ALWAYS_ACTIVE
/* ------------------------------------------------------------------------------------------------
 *                                       Global Constants
 * ------------------------------------------------------------------------------------------------
 */
extern const uint8_t mrfiBroadcastAddr[];


/**************************************************************************************************
 */
#endif
