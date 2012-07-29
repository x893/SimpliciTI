/**************************************************************************************************
  Revised:        $Date: 2011-09-29 10:13:10 -0700 (Thu, 29 Sep 2011) $
  Revision:       $Revision: 27756 $

  Copyright 2007-2008 Texas Instruments Incorporated.  All rights reserved.

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
 *   Radios: CC2510, CC2511, CC1110, CC1111
 *   Primary code file for supported radios.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <string.h>
#include <stdbool.h>
#include "mrfi.h"
#include "bsp.h"
#include "bsp_macros.h"
#include "mrfi_defs.h"
#include "../common/mrfi_f1f2.h"
#include "bsp_external/mrfi_board_defs.h"
#include "nwk_pll.h"
#include "bsp_leds.h" // debug only


/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC2510) || (defined MRFI_CC2511)

  #define MRFI_RSSI_OFFSET     71   /* for 250 kbsp. no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define MRFI_RSSI_VALID_DELAY_US    1000

#elif (defined MRFI_CC1110) || (defined MRFI_CC1111)

  #define MRFI_RSSI_OFFSET     73   /* for 433MHz @ 250 kbsp. no units */

  /* Worst case wait period in RX state before RSSI becomes valid.
   * These numbers are from Design Note DN505 with added safety margin.
   */
  #define MRFI_RSSI_VALID_DELAY_US    1300

#else
  #error "ERROR: RSSI Offset value not defined for this radio.
#endif


#define MRFI_MIN_SMPL_FRAME_SIZE            (MRFI_HEADER_SIZE + NWK_HDR_SIZE)

#ifdef MRFI_TIMER_ALWAYS_ACTIVE

// re-map static functions promoted to public for backwards compatibility
#define Mrfi_DelayUsec( a ) MRFI_DelayUsec( a )
#define Mrfi_DelayMs( a ) MRFI_DelayMs( a )

#endif // !MRFI_TIMER_ALWAYS_ACTIVE

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Radio Definitions
 *   - - - - - - - - - -
 */

#if (defined MRFI_CC1110) || (defined MRFI_CC1111)
#define MRFI_SETTING_PA_TABLE0    0x8E
#define MRFI_RADIO_PARTNUM        0x01
#define MRFI_RADIO_MIN_VERSION    3

#elif (defined MRFI_CC2510) || (defined MRFI_CC2511)
#define MRFI_SETTING_PA_TABLE0    0xFE
#define MRFI_RADIO_PARTNUM        0x81
#define MRFI_RADIO_MIN_VERSION    4

#else
#error "ERROR: Unspecified or unsupported radio."
#endif

/* bit of PARTNUM register that signifies chip has USB capability */
#define MRFI_RADIO_PARTNUM_USB_BIT          0x10

/* register RFST - command strobes */
#define SFSTXON       0x00
#define SCAL          0x01
#define SRX           0x02
#define STX           0x03
#define SIDLE         0x04

/* register MARCSTATE - state values */
#define RXTX_SWITCH   0x15
#define RX            0x0D
#define IDLE          0x01

/* register IEN2 - bit definitions */
#define RFIE          BV(0)

/* register S1CON - bit definitions */
#define RFIF_1        BV(1)
#define RFIF_0        BV(0)

/* register DMAARM - bit definitions */
#define ABORT         BV(7)

/* register CLKCON - bit definitions */
#define OSC           BV(6)

/* register SLEEP - bit definitions */
#define XOSC_STB      BV(6)
#define OSC_PD        BV(2)

/* register RFIF - bit definitions */
#define IRQ_DONE      BV(4)
#define IRQ_RXOVFL    BV(6)

/* register RFIM - bit definitions */
#define IM_DONE       BV(4)

/* register PKTSTATUS - bit definitions */
#define MRFI_PKTSTATUS_SFD   BV(3)
#define MRFI_PKTSTATUS_CCA   BV(4)
#define MRFI_PKTSTATUS_CS    BV(6)

/* random number generator */
#define RCTRL_CLOCK_LFSR    BV(2)

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Radio Register Settings
 *   - - - - - - - - - - - - -
 */

/* Main Radio Control State Machine control configuration - Calibrate when going from IDLE to RX or TX. */
#define MRFI_SETTING_MCSM0      0x14

/* Main Radio Control State Machine control configuration - Go to RX state after RX
 * and go to IDLE after TX.
 */
#define MRFI_SETTING_MCSM1      0x3C

/*
 *  Packet Length - Setting for maximum allowed packet length.
 *  The PKTLEN setting does not include the length field but maximum frame size does.
 *  Subtract length field size from maximum frame size to get value for PKTLEN.
 */
#define MRFI_SETTING_PKTLEN     (MRFI_MAX_FRAME_SIZE - MRFI_LENGTH_FIELD_SIZE)

/* Packet automation control - Original value except WHITE_DATA is extracted from SmartRF setting. */
#define MRFI_SETTING_PKTCTRL0   (0x05 | (SMARTRF_SETTING_PKTCTRL0 & BV(6)))

/* Packet automation control - base value is power up value whick has APPEND_STATUS enabled */
#define MRFI_SETTING_PKTCTRL1_BASE              BV(2)
#define MRFI_SETTING_PKTCTRL1_ADDR_FILTER_OFF   MRFI_SETTING_PKTCTRL1_BASE
#define MRFI_SETTING_PKTCTRL1_ADDR_FILTER_ON    (MRFI_SETTING_PKTCTRL1_BASE | (BV(1)|BV(0)))

/* early versions of SmartRF Studio did not export this value */
#ifndef SMARTRF_SETTING_FSCAL1
#define SMARTRF_SETTING_FSCAL1  0x80
#endif

/* TEST0 Various Test Settings - the VCO_SEL_CAL_EN bit must be zero */
#define MRFI_SETTING_TEST0      (SMARTRF_SETTING_TEST0 & ~(BV(1)))

/* if SmartRF setting for PA_TABLE0 is supplied, use that instead of built-in default */
#ifdef SMARTRF_SETTING_PA_TABLE0
#undef MRFI_SETTING_PA_TABLE0
#define MRFI_SETTING_PA_TABLE0   SMARTRF_SETTING_PA_TABLE0
#endif

/*
 * Maximum tramsmit time is calculated from the data rate times the maximum
 * number of bits that can be transmitted.  The data rate is calculated from
 * the DRATE_M and DRATE_E fields of the MDMCFG3 and MDMCFG4 registers
 * respectively.
 */
#ifndef MRFI_XTAL_FREQ_Hz
#define MRFI_XTAL_FREQ_Hz 26000000L
#endif
#define MRFI_SYNC_WORD_SIZE ( ( (SMARTRF_SETTING_MDMCFG2 & 0x3) == 0 ) ? 0 : \
                            ( ( (SMARTRF_SETTING_MDMCFG2 & 0x3) != 3 ) ? 2 : 4 ) )
#define MRFI_PREAMBLE_SIZE ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x00 ) ?  2 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x10 ) ?  3 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x20 ) ?  4 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x30 ) ?  6 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x40 ) ?  8 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x50 ) ? 12 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x60 ) ? 16 : \
                                                                              24 ) ) ) ) ) ) )
#define MRFI_MAX_TRANSMIT_BYTES ( MRFI_MAX_FRAME_SIZE + MRFI_SYNC_WORD_SIZE + MRFI_PREAMBLE_SIZE )
#define MRFI_TRANSMIT_BIT_PERIOD_us (1000000.0*(1L<<28)/((SMARTRF_SETTING_MDMCFG3+256.0)*MRFI_XTAL_FREQ_Hz*(1L<<(SMARTRF_SETTING_MDMCFG4 & 0xF))))
#define MRFI_MAX_TRANSMIT_TIME_us ((long)( MRFI_TRANSMIT_BIT_PERIOD_us * 8 * MRFI_MAX_TRANSMIT_BYTES + 500 ))


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    DMA Configuration Values
 *   - - - - - - - - - - - - - -
 */

/* DMA channel number */
#define MRFI_DMA_CHAN               0

/* DMA configuration data structure size */
#define RXTX_DMA_STRUCT_SIZE        8

/* byte offset 4 (upper bits of LEN are never used so are always set to zero) */
#define RXTX_DMA_VLEN_XFER_BYTES_PLUS_1       ((/*  VLEN = */( 1 ))<<5)
#define RXTX_DMA_VLEN_XFER_BYTES_PLUS_3       ((/*  VLEN = */( 4 ))<<5)

/* byte offset 5 */
#define RXTX_DMA_LEN            (MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE)

/* byte offset 6 */
#define RXTX_DMA_WORDSIZE       (/*  WORDSIZE = */(  0 )  << 7)
#define RXTX_DMA_TMODE          (/*     TMODE = */(  0 )  << 5)
#define RXTX_DMA_TRIG           (/*      TRIG = */( 19 )  << 0)

/* byte offset 7 */
#define RXTX_DMA_SRCINC_PLUS_1  (/*    SRCINC = */( 1 )  << 6)
#define RXTX_DMA_SRCINC_NONE    (/*    SRCINC = */( 0 )  << 6)
#define RXTX_DMA_DESTINC_PLUS_1 (/*   DESTINC = */( 1 )  << 4)
#define RXTX_DMA_DESTINC_NONE   (/*   DESTINC = */( 0 )  << 4)
#define RXTX_DMA_IRQMASK        (/*   IRQMASK = */( 0 )  << 3)
#define RXTX_DMA_M8             (/*        M8 = */( 0 )  << 2)
#define RXTX_DMA_PRIORITY       (/*  PRIORITY = */( 1 )  << 0)

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls
   * to the microsecond routine which can be calibrated independently.
   */
#if defined(SW_TIMER)
#define APP_USEC_VALUE    100
#else
#define APP_USEC_VALUE    500
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */
#define HIGH_BYTE_OF_WORD(x)            ((uint8_t) (((uint16_t)(x)) >> 8))
#define LOW_BYTE_OF_WORD(x)             ((uint8_t) (((uint16_t)(x)) & 0xFF))


#ifndef MRFI_TIMER_ALWAYS_ACTIVE

/* There is no bit in h/w to tell if RSSI in the register is valid or not.
 * The hardware needs to be in RX state for a certain amount of time before
 * a valid RSSI value is calculated and placed in the register. This min
 * wait time is defined by MRFI_BOARD_RSSI_VALID_DELAY_US. We don't need to
 * add such delay every time RSSI value is needed. If the Carier Sense signal
 * is high or CCA signal is high, we know that the RSSI value must be valid.
 * We use that knowledge to reduce our wait time. We break down the delay loop
 * in multiple chunks and during each iteration, check for the CS and CCA
 * signal. If either of these signals is high, we return immediately. Else,
 * we wait for the max delay specified.
 */
#define MRFI_RSSI_VALID_WAIT()                                                \
{                                                                             \
  int16_t delay = MRFI_RSSI_VALID_DELAY_US;                                   \
  do                                                                          \
  {                                                                           \
    if(PKTSTATUS & (MRFI_PKTSTATUS_CCA | MRFI_PKTSTATUS_CS))  \
    {                                                                         \
      break;                                                                  \
    }                                                                         \
    Mrfi_DelayUsec(64); /* sleep */                                           \
    delay -= 64;                                                              \
  }while(delay > 0);                                                          \
}
#endif // !MRFI_TIMER_ALWAYS_ACTIVE

#define MRFI_STROBE_IDLE_AND_WAIT()  \
{                                    \
  RFST = SIDLE;                      \
  while (MARCSTATE != IDLE) ;        \
}

/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static int8_t Mrfi_CalculateRssi(uint8_t);
static void MRFI_PrepareToTx( mrfiPacket_t * pPacket );
static bool mrfi_TxDone( void );

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
static bool Mrfi_CheckSem( void );
static bool Mrfi_ValidateRSSI( void );
#else
static void   Mrfi_DelayUsecSem(uint16_t);
static void Mrfi_DelayUsec(uint16_t);
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8_t mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;
static mrfiPacket_t mrfiIncomingPacket;

/* reply delay support */
static volatile uint8_t  sKillSem = 0;
static volatile uint8_t  sReplyDelayContext = 0;
static          uint16_t sReplyDelayScalar = 0;
static          uint16_t sBackoffHelper = 0;

#if (MRFI_DMA_CHAN == 0)
uint8_t XDATA mrfiDmaCfg[RXTX_DMA_STRUCT_SIZE];
#define MRFI_DMA_CFG_ADDRESS   &mrfiDmaCfg[0]
#endif

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  static bool stx_active = false;
  static MRFI_ms_event_t sOne_ms_event_hook = NULL;
  static int32_t sTmrRateOffset = 0;
  #if BSP_TIMER_SIZE == 8
    static volatile uint8_t sTimerCntHi;
  #endif
  #ifdef NWK_PLL
    static mrfi_Time_t* sTxTimeStampAddr = NULL;
    static mrfi_Time_t* sRxTimeStampAddr = NULL;
  #endif
#endif

  FHSS_ACTIVE( uint16_t sHopCount = 0 );
  FHSS_ACTIVE( uint8_t sLogicalChannel = 0 );
  FHSS_ACTIVE( uint8_t sHopNowSem = 0 );
  FHSS_ACTIVE( uint16_t sHopRate = MRFI_HOP_TIME_ms - 1 );
  FHSS_ACTIVE( bool sTxValid = false );

/**************************************************************************************************
 * @fn          MRFI_Init
 *
 * @brief       Initialize MRFI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Init(void)
{
  INIT_INTERRUPT_PRIORITY( );
  
  /* ------------------------------------------------------------------
   *    Run-time integrity checks
   *   ---------------------------
   */

  memset(&mrfiIncomingPacket, 0x0, sizeof(mrfiIncomingPacket));
  /* verify the correct radio is installed */
  MRFI_ASSERT( (PARTNUM & ~MRFI_RADIO_PARTNUM_USB_BIT) == MRFI_RADIO_PARTNUM );
  MRFI_ASSERT( VERSION >= MRFI_RADIO_MIN_VERSION );  /* obsolete radio version */


  /* ------------------------------------------------------------------
   *    Switch to high speed crystal oscillator.
   *   ------------------------------------------
   */

  /* power up both oscillators - high speed crystal oscillator will power up,
   * the RC oscillator will remain powered-up and selected.
   */
  SLEEP &= ~OSC_PD;

  /* wait for high speed crystal to become stable */
  while(!(SLEEP & XOSC_STB));

  /* switch from RC oscillator to high speed crystal oscillator */
  CLKCON &= ~OSC;

  /* power down the oscillator not selected, i.e. the RC oscillator */
  SLEEP |= OSC_PD;

  /* ------------------------------------------------------------------
   *    Variable Initialization
   *   -------------------------
   */


  /* ------------------------------------------------------------------
   *    DMA Initialization
   *   --------------------
   */
#if (MRFI_DMA_CHAN == 0)
  DMA0CFGH = HIGH_BYTE_OF_WORD( &mrfiDmaCfg[0] );
  DMA0CFGL = LOW_BYTE_OF_WORD ( &mrfiDmaCfg[0] );
#endif


  /* ------------------------------------------------------------------
   *    Configure timer
   *   ----------------------
   */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false;
  Mrfi_TimerInit( );
  #if BSP_TIMER_SIZE == 8
    sTimerCntHi = 0;
  #endif
  sOne_ms_event_hook = NULL;
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  FHSS_ACTIVE( sHopCount = 0 );
  FHSS_ACTIVE( sLogicalChannel = MRFI_RandomByte( ) % MRFI_NUM_LOGICAL_CHANS );
  FHSS_ACTIVE( sHopNowSem = 0 );
  FHSS_ACTIVE( sHopRate = MRFI_HOP_TIME_ms - 1 );


  /* ------------------------------------------------------------------
   *    Configure radio
   *   -----------------
   */

  /* internal radio register configuration */
  MCSM1     = MRFI_SETTING_MCSM1;
  MCSM0     = MRFI_SETTING_MCSM0;
  PKTLEN    = MRFI_SETTING_PKTLEN;
  PKTCTRL0  = MRFI_SETTING_PKTCTRL0;
  PA_TABLE0 = MRFI_SETTING_PA_TABLE0;
  TEST0     = MRFI_SETTING_TEST0;

  /* imported SmartRF radio register configuration */
  FSCTRL1  = SMARTRF_SETTING_FSCTRL1;
  FSCTRL0  = SMARTRF_SETTING_FSCTRL0;
  FREQ2    = SMARTRF_SETTING_FREQ2;
  FREQ1    = SMARTRF_SETTING_FREQ1;
  FREQ0    = SMARTRF_SETTING_FREQ0;
  MDMCFG4  = SMARTRF_SETTING_MDMCFG4;
  MDMCFG3  = SMARTRF_SETTING_MDMCFG3;
  MDMCFG2  = SMARTRF_SETTING_MDMCFG2;
  MDMCFG1  = SMARTRF_SETTING_MDMCFG1;
  MDMCFG0  = SMARTRF_SETTING_MDMCFG0;
  DEVIATN  = SMARTRF_SETTING_DEVIATN;
  FOCCFG   = SMARTRF_SETTING_FOCCFG;
  BSCFG    = SMARTRF_SETTING_BSCFG;
  AGCCTRL2 = SMARTRF_SETTING_AGCCTRL2;
  AGCCTRL1 = SMARTRF_SETTING_AGCCTRL1;
  AGCCTRL0 = SMARTRF_SETTING_AGCCTRL0;
  FREND1   = SMARTRF_SETTING_FREND1;
  FREND0   = SMARTRF_SETTING_FREND0;
  FSCAL3   = SMARTRF_SETTING_FSCAL3;
  FSCAL2   = SMARTRF_SETTING_FSCAL2;
  FSCAL1   = SMARTRF_SETTING_FSCAL1;
  FSCAL0   = SMARTRF_SETTING_FSCAL0;
  TEST2    = SMARTRF_SETTING_TEST2;
  TEST1    = SMARTRF_SETTING_TEST1;


  /* Initial radio state is IDLE state */
  mrfiRadioState = MRFI_RADIO_STATE_IDLE;

  /* set default channel */
  MRFI_SetLogicalChannel( 0 );

  /* Seed for the Random Number Generator */
  {
    uint16_t rndSeed = 0x0;

    /* Put radio in RX mode */
    RFST = SRX;

    /* Delay for valid RSSI. Otherwise same RSSI
     * value could be read every time.
     */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
    MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

    {
      uint8_t i;
      for(i=0; i<16; i++)
      {
        /* use most random bit of RSSI to populate the random seed */
        rndSeed = (rndSeed << 1) | (RSSI & 0x01);
      }
    }

    /* Force the seed to be non-zero by setting one bit, just in case... */
    rndSeed |= 0x0080;

    /* Need to write to RNDL twice to seed it */
    RNDL = rndSeed & 0xFF;
    RNDL = rndSeed >> 8;

    /* Call RxModeOff() instead of an idle strobe.
     * This will clean up any flags that were set while radio was in rx state.
     */
    Mrfi_RxModeOff();
  }


  /*****************************************************************************************
   *                            Compute reply delay scalar
   *
   * Formula from data sheet for all the narrow band radios is:
   *
   *                (256 + DATAR_Mantissa) * 2^(DATAR_Exponent)
   * DATA_RATE =    ------------------------------------------ * f(xosc)
   *                                    2^28
   *
   * To try and keep some accuracy we change the exponent of the denominator
   * to (28 - (exponent from the configuration register)) so we do a division
   * by a smaller number. We find the power of 2 by shifting.
   *
   * The maximum delay needed depends on the MAX_APP_PAYLOAD parameter. Figure
   * out how many bits that will be when overhead is included. Bits/bits-per-second
   * is seconds to transmit (or receive) the maximum frame. We multiply this number
   * by 1000 to find the time in milliseconds. We then additionally multiply by
   * 10 so we can add 5 and divide by 10 later, thus rounding up to the number of
   * milliseconds. This last won't matter for slow transmissions but for faster ones
   * we want to err on the side of being conservative and making sure the radio is on
   * to receive the reply. The semaphore monitor will shut it down. The delay adds in
   * a platform fudge factor that includes processing time on peer plus lags in Rx and
   * processing time on receiver's side. Also includes round trip delays from CCA
   * retries. This portion is included in PLATFORM_FACTOR_CONSTANT defined in mrfi.h.
   *
   * Note: We assume a 26 MHz oscillator frequency for the non-USB target radios
   *       and 24 MHz for the USB-target radios.
   * ***************************************************************************************
   */
#if defined(MRFI_CC2510)  ||  defined(MRFI_CC1110)
#define   MRFI_RADIO_OSC_FREQ         26000000
#elif defined(MRFI_CC2511)  ||  defined(MRFI_CC1111)
#define   MRFI_RADIO_OSC_FREQ         24000000
#endif

#define   PHY_PREAMBLE_SYNC_BYTES    8

  {
    uint32_t dataRate, bits;
    uint16_t exponent, mantissa;

    /* mantissa is in MDMCFG3 */
    mantissa = 256 + SMARTRF_SETTING_MDMCFG3;

    /* exponent is lower nibble of MDMCFG4. */
    exponent = 28 - (SMARTRF_SETTING_MDMCFG4 & 0x0F);

    /* we can now get data rate */
    dataRate = mantissa * (MRFI_RADIO_OSC_FREQ>>exponent);

    bits = ((uint32_t)((PHY_PREAMBLE_SYNC_BYTES + MRFI_MAX_FRAME_SIZE)*8))*10000;

    /* processing on the peer + the Tx/Rx time plus more */
    sReplyDelayScalar = PLATFORM_FACTOR_CONSTANT + (((bits/dataRate)+5)/10);

    /* This helper value is used to scale the backoffs during CCA. At very
     * low data rates we need to backoff longer to prevent continual sampling
     * of valid frames which take longer to send at lower rates. Use the scalar
     * we just calculated divided by 32. With the backoff algorithm backing
     * off up to 16 periods this will result in waiting up to about 1/2 the total
     * scalar value. For high data rates this does not contribute at all. Value
     * is in microseconds.
     */
    sBackoffHelper = MRFI_BACKOFF_PERIOD_USECS + (sReplyDelayScalar>>5)*1000;
  }

  /* ------------------------------------------------------------------
   *    Configure interrupts
   *   ----------------------
   */

  /* enable general RF interrupts */
  IEN2 |= RFIE;

  /* enable global interrupts */
  BSP_ENABLE_INTERRUPTS();
}


/**************************************************************************************************
 * @fn          MRFI_PrepareToTx
 *
 * @brief       Setup dma and other variables for transmit
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      None
 **************************************************************************************************
 */
static void MRFI_PrepareToTx( mrfiPacket_t * pPacket )
{
  uint8_t XDATA * pCfg;

  /* Abort any ongoing DMA transfer */
  DMAARM = ABORT | BV( MRFI_DMA_CHAN );

  MRFI_STROBE_IDLE_AND_WAIT();      // put the radio in a known state
  
  /* Clear any pending DMA interrupts */
  DMAIRQ = ~BV(MRFI_DMA_CHAN);

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = true; // indicate we are in the act of transmitting
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  
  /* configure DMA channel for transmit */

  pCfg = MRFI_DMA_CFG_ADDRESS;
  *pCfg++ = /* offset 0 : */  HIGH_BYTE_OF_WORD( &(pPacket->frame[0]) );  /* SRCADDRH */
  *pCfg++ = /* offset 1 : */  LOW_BYTE_OF_WORD ( &(pPacket->frame[0]) );  /* SRCADDRL */
  *pCfg++ = /* offset 2 : */  HIGH_BYTE_OF_WORD( &X_RFD );  /* DSTADDRH */
  *pCfg++ = /* offset 3 : */  LOW_BYTE_OF_WORD ( &X_RFD );  /* DSTADDRL */
  *pCfg++ = /* offset 4 : */  RXTX_DMA_VLEN_XFER_BYTES_PLUS_1;
  *pCfg++ = /* offset 5 : */  RXTX_DMA_LEN;
  *pCfg++ = /* offset 6 : */  RXTX_DMA_WORDSIZE | RXTX_DMA_TMODE | RXTX_DMA_TRIG;
  *pCfg   = /* offset 7 : */  RXTX_DMA_SRCINC_PLUS_1 | RXTX_DMA_DESTINC_NONE |
                                RXTX_DMA_IRQMASK | RXTX_DMA_M8 | RXTX_DMA_PRIORITY;

  return;
}


/**************************************************************************************************
 * @fn          mrfi_TxDone
 *
 * @brief       Indicates status of transmission completion
 *
 * @param       None
 *
 * @return      true if transmission is complete, false otherwise
 **************************************************************************************************
 */
bool mrfi_TxDone( void )
{
  return (RFIF & IRQ_DONE) || MARCSTATE == RX || MARCSTATE == IDLE;
}


/**************************************************************************************************
 * @fn          MRFI_Transmit
 *
 * @brief       Transmit a packet using CCA algorithm.
 *
 * @param       pPacket - pointer to packet to transmit
 *              txType  - FORCED or CCA
 *
 * @return      Return code indicates success or failure of transmit:
 *                  MRFI_TX_RESULT_SUCCESS - transmit succeeded
 *                  MRFI_TX_RESULT_FAILED  - transmit failed because CCA failed
 **************************************************************************************************
 */
uint8_t MRFI_Transmit(mrfiPacket_t * pPacket, uint8_t txType)
{
  static uint8_t ccaRetries;
  uint8_t returnValue = MRFI_TX_RESULT_SUCCESS;

  /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Turn off reciever. We can ignore/drop incoming packets during transmit. */
  Mrfi_RxModeOff();

  MRFI_PrepareToTx( pPacket ); // setup transmission
  
  /* ------------------------------------------------------------------
   *    Immediate transmit
   *   ---------------------
   */
  if (txType == MRFI_TX_TYPE_FORCED)
  {
#ifdef NWK_PLL
    bspIState_t s;
#endif
    /* ARM the DMA channel */
    DMAARM |= BV( MRFI_DMA_CHAN );

#ifdef NWK_PLL
    /* if sTxTimeStampAddr is not null then the network application has
     * filled it in because it is a PLL packet that requires the transmit
     * time.  After filling in the time stamp at the address specified,
     * delete the reference so future packets will not have a time stamp
     * acquisition.
     */
    BSP_ENTER_CRITICAL_SECTION(s);
    if( sTxTimeStampAddr != NULL ) // if need to add time stamp to packet
    {
      MRFI_GetLocalRawTime(sTxTimeStampAddr); // fill in the packet data
      sTxTimeStampAddr = NULL; // clear the reference
    }
#endif

    /* Strobe TX */
    RFST = STX;
#ifdef NWK_PLL
    BSP_EXIT_CRITICAL_SECTION(s);
#endif

    /* wait for transmit to complete */
    while (!(RFIF & IRQ_DONE));

    /* Clear the interrupt flag */
    RFIF = ~IRQ_DONE;
  }
  else
  {
    /* ------------------------------------------------------------------
     *    CCA transmit
     *   ---------------
     */
#ifdef NWK_PLL
    bspIState_t s;
#endif

    MRFI_ASSERT( txType == MRFI_TX_TYPE_CCA );

    /* set number of CCA retries */
    ccaRetries = MRFI_CCA_RETRIES;

    /* ===============================================================================
     *    CCA Loop
     *  =============
     */
    while(1)
    {
      /* ARM the DMA channel */
      DMAARM |= BV( MRFI_DMA_CHAN );

      /* strobe to enter receive mode */
      RFST = SRX;

      /* Wait for radio to enter the RX mode */
      while(MARCSTATE != RX);

      /* wait for the rssi to be valid. */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
      if( stx_active == false ) // if the channel was changed
      {
          Mrfi_RxModeOff();            // turn off the radio
          MRFI_PrepareToTx( pPacket ); // setup transmission again
          continue; // restart the cca loop
      }
#else // MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

#ifdef NWK_PLL
      /* if sTxTimeStampAddr is not null then the network application has
       * filled it in because it is a PLL packet that requires the transmit
       * time.  Don't remove this reference until we know the packet has
       * actually been transmitted.
       */
      BSP_ENTER_CRITICAL_SECTION(s);
      if( sTxTimeStampAddr != NULL ) // if need to add time stamp to packet
        MRFI_GetLocalRawTime(sTxTimeStampAddr); // fill in the packet data
#endif

      /* Strobe TX-if-CCA */
      RFST = STX;

#ifdef NWK_PLL
      BSP_EXIT_CRITICAL_SECTION(s);
#endif

      if(MARCSTATE != RX) /* ck if exited rx state */
      {
        /* ------------------------------------------------------------------
         *    Clear Channel Assessment passed.
         * ----------------------------------
         */
        /* wait for transmit to complete */
        //while (!(RFIF & IRQ_DONE) && MARCSTATE != RX && MARCSTATE != IDLE);
        Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                            MRFI_MAX_TRANSMIT_TIME_us % 1000,
                            mrfi_TxDone );

#ifdef NWK_PLL
        // Packet transmitted, regardless of packet type, remove reference.
        sTxTimeStampAddr = NULL;
#endif

        /* Clear the interrupt flag */
        RFIF = ~IRQ_DONE;

        break;
      }
      else
      {
        /* ------------------------------------------------------------------
         *    Clear Channel Assessment failed.
         * ----------------------------------
         */

        /* Retry ? */
        if (ccaRetries != 0)
        {
          /* turn off reciever to conserve power during backoff */
          Mrfi_RxModeOff();

          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();

          /* decrement CCA retries before loop continues */
          ccaRetries--;

          MRFI_PrepareToTx( pPacket ); // re-setup transmission
        }
        else  /* No CCA retries are left, abort */
        {
#ifdef NWK_PLL
        // Make sure the reference is cleared if it was a PLL packet
        sTxTimeStampAddr = NULL;
#endif
          /* set return value for failed transmit and break */
          returnValue = MRFI_TX_RESULT_FAILED;
          break;
        }
      } /* CCA Failed */
    } /* CCA loop */
  }/* txType is CCA */

  
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we are in the act of transmitting
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  
  
  /* Done with TX. Clean up time... */

  /* turn radio back off to put it in a known state */
  Mrfi_RxModeOff();

  /* If the radio was in RX state when transmit was attempted,
   * put it back in RX state.
   */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }

  return( returnValue );
}


/**************************************************************************************************
 * @fn          MRFI_Receive
 *
 * @brief       Copies last packet received to the location specified.
 *              This function is meant to be called after the ISR informs
 *              higher level code that there is a newly received packet.
 *
 * @param       pPacket - pointer to location of where to copy received packet
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Receive(mrfiPacket_t * pPacket)
{
  *pPacket = mrfiIncomingPacket;
}


/**************************************************************************************************
 * @fn          MRFI_RfIsr
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
BSP_ISR_FUNCTION( MRFI_RfIsr, RF_VECTOR )
{
  uint8_t frameLen;

  bspIState_t istate;
  
#ifdef NWK_PLL
  BSP_ENTER_CRITICAL_SECTION( istate );

  /*  If the network pll is running then we need to acquire
   *  the receive time stamp (at least as close to the actual
   *  time that the receive occurred)
   */
  if( sRxTimeStampAddr != NULL )
    MRFI_GetLocalRawTime( sRxTimeStampAddr );

  BSP_EXIT_CRITICAL_SECTION( istate );

#endif

  istate = __bsp_GET_ISTATE__();
  BSP_ENABLE_INTERRUPTS();

  /* We should receive this interrupt only in RX state
   * Should never receive it if RX was turned On only for
   * some internal mrfi processing like - during CCA.
   * Otherwise something is terribly wrong.
   */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  /* Check for overflow */
  if ((RFIF & IRQ_DONE) && (RFIF & IRQ_RXOVFL))
  {
    RFIF = ~IRQ_DONE;
    RFIF = ~IRQ_RXOVFL;
    S1CON = 0; /* Clear MCU interrupt flag */

    /* Only way out of this is to go to IDLE state */
    Mrfi_RxModeOff();

    /* zero-out MRFI buffer to help NWK eliminate undetected rogue frames if they pass here */
    memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));

    /* OK to start again... */
    Mrfi_RxModeOn();

    __bsp_RESTORE_ISTATE__(istate);
    
    return;
  }

  RFIF = ~IRQ_DONE;           /* Clear the interrupt at the source */
  S1CON = 0; /* Clear MCU interrupt flag */

  /* ------------------------------------------------------------------
   *    Copy RX Metrics into packet structure
   *   ---------------------------------------
   */
  {
    uint8_t offsetToRxMetrics = mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFS] + 1;
    /* The metrics were DMA'd so they may reside on the frame buffer rather than the
     * metrics buffer. Get them to the proper location.
     */
    memmove(mrfiIncomingPacket.rxMetrics,&mrfiIncomingPacket.frame[offsetToRxMetrics], sizeof(mrfiIncomingPacket.rxMetrics));
  }


  /* ------------------------------------------------------------------
   *    CRC  and frame length check
   *   ------------
   */

  frameLen = mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFS];
  /* determine if CRC or length check failed */
  if (!(mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] & MRFI_RX_METRICS_CRC_OK_MASK) ||
       ((frameLen + MRFI_LENGTH_FIELD_SIZE) > MRFI_MAX_FRAME_SIZE) ||
       (frameLen < MRFI_MIN_SMPL_FRAME_SIZE)
     )
  {
    /* CRC or length check failed - do nothing, skip to end */
  }
  else
  {
    /* CRC passed - continue processing */

    /* ------------------------------------------------------------------
     *    Filtering
     *   -----------
     */

    /* if address is not filtered, receive is successful */
    if (!MRFI_RxAddrIsFiltered(MRFI_P_DST_ADDR(&mrfiIncomingPacket)))
    {
      {
        /* ------------------------------------------------------------------
         *    Receive successful
         *   --------------------
         */

        /* Convert the raw RSSI value and do offset compensation for this radio */
        mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_RSSI_OFS] =
            Mrfi_CalculateRssi(mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_RSSI_OFS]);

        /* Remove the CRC valid bit from the LQI byte */
        mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] =
          (mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] & MRFI_RX_METRICS_LQI_MASK);


        /* call external, higher level "receive complete" processing routine */
        MRFI_RxCompleteISR();
      }
   }
  }

  /* zero-out MRFI buffer to help NWK eliminate undetected rogue frames if they pass here */
  memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));

  /* arm DMA channel for next receive */
  DMAARM |= BV( MRFI_DMA_CHAN );

  __bsp_RESTORE_ISTATE__(istate);
}


/**************************************************************************************************
 * @fn          MRFI_Sleep
 *
 * @brief       Request radio go to sleep.
 *
 * @param       none
 *
 * @return      zero : if successfully went to sleep
 *              non-zero : if sleep was not entered
 **************************************************************************************************
 */
void MRFI_Sleep(void)
{
  /* If radio is not asleep, put it to sleep */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    bspIState_t s;

    /* critical section necessary for watertight testing and setting of state variables */
    BSP_ENTER_CRITICAL_SECTION(s);

    /* go to idle so radio is in a known state before sleeping */
    MRFI_RxIdle();

    /* There is no individual power control to the RF block on this radio.
     * So putting it to Idle is the best we can do.
     */

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;

    BSP_EXIT_CRITICAL_SECTION(s);
  }
}


/**************************************************************************************************
 * @fn          MRFI_WakeUp
 *
 * @brief       Wake up radio from sleep state.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_WakeUp(void)
{
  /* verify high speed crystal oscillator is selected, required for radio operation */
  MRFI_ASSERT( !(CLKCON & OSC) );

  /* if radio is already awake, just ignore wakeup request */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    return;
  }

  /* restore radio registers that are reset during sleep */
  FSCAL3 = SMARTRF_SETTING_FSCAL3;
  FSCAL2 = SMARTRF_SETTING_FSCAL2;
  FSCAL1 = SMARTRF_SETTING_FSCAL1;

  /* enter idle mode */
  mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  MRFI_STROBE_IDLE_AND_WAIT();
  
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
}

/**************************************************************************************************
 * @fn          MRFI_RandomByte
 *
 * @brief       Returns a random byte
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8_t MRFI_RandomByte(void)
{
  /* clock the random generator once to get a new random value */
  ADCCON1 |= RCTRL_CLOCK_LFSR;

  return RNDL;
}



/**************************************************************************************************
 * @fn          Mrfi_RxModeOn
 *
 * @brief       Put radio into receive mode.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOn(void)
{
  uint8_t XDATA * pCfg;

  /* configure DMA channel for receive */
  pCfg = MRFI_DMA_CFG_ADDRESS;
  *pCfg++ = /* offset 0 : */  HIGH_BYTE_OF_WORD( &X_RFD );  /* SRCADDRH */
  *pCfg++ = /* offset 1 : */  LOW_BYTE_OF_WORD ( &X_RFD );  /* SRCADDRL */
  *pCfg++ = /* offset 2 : */  HIGH_BYTE_OF_WORD( &(mrfiIncomingPacket.frame[0]) );  /* DSTADDRH */
  *pCfg++ = /* offset 3 : */  LOW_BYTE_OF_WORD ( &(mrfiIncomingPacket.frame[0]) );  /* DSTADDRL */
  *pCfg++ = /* offset 4 : */  RXTX_DMA_VLEN_XFER_BYTES_PLUS_3;
  *pCfg++ = /* offset 5 : */  RXTX_DMA_LEN;
  *pCfg++ = /* offset 6 : */  RXTX_DMA_WORDSIZE | RXTX_DMA_TMODE | RXTX_DMA_TRIG;
  *pCfg   = /* offset 7 : */  RXTX_DMA_SRCINC_NONE | RXTX_DMA_DESTINC_PLUS_1 |
                              RXTX_DMA_IRQMASK | RXTX_DMA_M8 | RXTX_DMA_PRIORITY;

  /* abort any DMA transfer that might be in progress */
  DMAARM = ABORT | BV( MRFI_DMA_CHAN );

  /* clean out buffer to help protect against spurious frames */
  memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));

  /* arm the dma channel for receive */
  DMAARM |= BV( MRFI_DMA_CHAN );

  /* Clear interrupts */
//  S1CON &= ~(RFIF_1 | RFIF_0); /* Clear MCU interrupt flag */
  S1CON = 0; /* Clear MCU interrupt flag */
  RFIF = ~IRQ_DONE;           /* Clear the interrupt at the source */

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE

  /* send strobe to enter receive mode */
  RFST = SRX;

  /* enable "receive/transmit done" interrupts */
  RFIM |= IM_DONE;
}

/**************************************************************************************************
 * @fn          MRFI_RxOn
 *
 * @brief       Turn on the receiver.  No harm is done if this function is called when
 *              receiver is already on.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxOn(void)
{
  /* radio must be awake before we can move it to RX state */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Put the radio in RX state, if not already */
  if(mrfiRadioState != MRFI_RADIO_STATE_RX)
  {
    mrfiRadioState = MRFI_RADIO_STATE_RX;
    Mrfi_RxModeOn();
  }
}

/**************************************************************************************************
 * @fn          Mrfi_RxModeOff
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOff(void)
{
  /*disable receive interrupts */
  RFIM &= ~IM_DONE;

  /* turn off radio */
  MRFI_STROBE_IDLE_AND_WAIT();

  /* Abort any ongoing DMA transfer */
  DMAARM = ABORT | BV( MRFI_DMA_CHAN );

  /* Clear any pending DMA interrupts */
  DMAIRQ = ~BV(MRFI_DMA_CHAN);

  /* flush the receive FIFO of any residual data */
  /* no need for flush. only 1 byte deep fifo. */

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE

  /* clear receive interrupt */
  S1CON = 0; /* Clear MCU interrupt flag */
  RFIF = ~IRQ_DONE;           /* Clear the interrupt at the source */
}


/**************************************************************************************************
 * @fn          MRFI_RxIdle
 *
 * @brief       Put radio in idle mode (receiver if off).  No harm is done this function is
 *              called when radio is already idle.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxIdle(void)
{
  /* radio must be awake to move it to idle mode */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* if radio is on, turn it off */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOff();
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  }
}

/**************************************************************************************************
 * @fn          MRFI_FreqHoppingBckgndr
 *
 * @brief       This function needs to be called periodicaly to manage frequency hopping
 *
 * @param       none
 *
 * @return      nothing
 **************************************************************************************************
 */
#ifdef FREQUENCY_HOPPING
void MRFI_FreqHoppingBckgndr( void )
{
  if( stx_active == false && sHopNowSem != 0 )
  {
    sHopNowSem = 0; // clear the semaphore

    // wait for the radio to not be busy
    while( !( MARCSTATE == IDLE
              || MARCSTATE == RX
                 && ( PKTSTATUS & MRFI_PKTSTATUS_SFD ) == 0 ) );

    // change the channel
    MRFI_SetLogicalChannel( sLogicalChannel );
  }
  return;
}
#endif // FREQUENCY_HOPPING

#ifdef NWK_PLL
/**************************************************************************************************
 * @fn          MRFI_SetRxTimeStampAddr
 *
 * @brief       Sets the address at which receive time stamps should be written to.
 *              this address must be statically constant as it is accessed from the
 *              receive ISR.
 *
 * @param       t
 *                The address at which a mrfi_Time_t object exists.
 *
 * @return      The previous address stored or NULL of no previous address stored.
 **************************************************************************************************
 */
mrfi_Time_t* MRFI_SetRxTimeStampAddr( mrfi_Time_t* t )
{
  mrfi_Time_t* tmp = sRxTimeStampAddr; // save the previous address
  sRxTimeStampAddr = t;                // assign the new address
  return tmp;                          // return the old address
}

/**************************************************************************************************
 * @fn          MRFI_SetTxTimeStampAddr
 *
 * @brief       Sets the address at which transmit time stamps should be written to.
 *              Call this function just before calling a any function which transmits
 *              a PLL packet requiring a transmit time stamp to be inserted.  This value
 *              will be reset to NULL once the time stamp has been filled in.  Therefore,
 *              this function must be called before each PLL packet is sent.
 *
 * @param       t
 *                The address at which a mrfi_Time_t object exists.
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetTxTimeStampAddr( mrfi_Time_t* t )
{
  sTxTimeStampAddr = t;                // assign the new address
  return;
}

#endif

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
/**************************************************************************************************
 * @fn          MRFI_AdjustTimerModulationRate
 *
 * @brief       Adjusts the timer modulation rate value which adjusts the average rate
 *              at which the timer ISR is entered.
 *
 * @param       delta
 *                Represents the change in value to modify the modulation rate by in
 *                q8.24 format (signed)
 *
 * @return      nothing
 **************************************************************************************************
 */
void MRFI_AdjustTimerModulationRate( int32_t delta )
{
  BSP_CRITICAL_STATEMENT( sTmrRateOffset = delta );
}

/**************************************************************************************************
 * @fn          MRFI_Set_ms_Event
 *
 * @brief       Assigns the passed event hook function to the 1 millisecond event hook.
 *                A NULL value can be used to indicate no hook function is to be called.
 *              NOTE: This function is normally used by the NWK layer and the user should
 *                    not generally call this function but instead call the similar
 *                    NWK layer 1 ms hook installer NWK_Set_ms_Event.
 *
 * @param       evt_hook
 *                evt_hook points to a function which will be called each time 1ms of
 *                system time has elapsed.
 *
 * @return      the previous hook function pointer or NULL if none assigned.
 **************************************************************************************************
 */
MRFI_ms_event_t MRFI_Set_ms_Event( MRFI_ms_event_t evt_hook )
{
  MRFI_ms_event_t tmp = sOne_ms_event_hook; // copy over the current event hook
  BSP_CRITICAL_STATEMENT( sOne_ms_event_hook = evt_hook ); // assign the new event hook
  return tmp;                               // return the previous event hook
}

/**************************************************************************************************
 * @fn          MRFI_Get_ms_Event
 *
 * @brief       Returns the currently assigned 1 millisecond event hook function pointer.
 *
 * @param       none
 *
 * @return      the currently assigned hook function pointer or NULL if none assigned.
 **************************************************************************************************
 */
MRFI_ms_event_t MRFI_Get_ms_Event( void )
{
  return sOne_ms_event_hook; // return the current hook function
}

/**************************************************************************************************
 * @fn          Mrfi_ValidateRSSI
 *
 * @brief       Returns true if the RSSI indication is valid
 *                There is no bit in h/w to tell if RSSI in the register is valid or not.
 *                The hardware needs to be in RX state for a certain amount of time
 *                before a valid RSSI value is calculated and placed in the register.
 *                This min wait time is defined by MRFI_BOARD_RSSI_VALID_DELAY_US. We
 *                don't need to add such delay every time RSSI value is needed. If the
 *                Carier Sense signal is high or CCA signal is high, we know that the
 *                RSSI value must be valid.  We use that knowledge to reduce our wait
 *                time.  This function checks for the CS and CCA signals. If either of
 *                these signals is high, we return immediately. Else, we wait for the
 *                max delay specified.
 * @param       none
 *
 * @return      true if RSSI indicates as valid
 **************************************************************************************************
 */
static bool Mrfi_ValidateRSSI( void )
{
  return PKTSTATUS & (MRFI_PKTSTATUS_CCA | MRFI_PKTSTATUS_CS);
}

/**************************************************************************************************
 * @fn          MRFI_GetLocalRawTime
 *
 * @brief       Fills in the time structure passed in with the local time of this radio.
 *                This code is unique in that it always takes the same amount of time
 *                to run through the critical section so the time offset from capture
 *                is always constant.  This is important for accurate time stamps on
 *                transmitted packets.
 *
 * @param       time
 *                a pointer to a BSP_RawTime_t time structure to be filled in
 *
 * @return      void (nothing)
 **************************************************************************************************
 */
void MRFI_GetLocalRawTime( mrfi_Time_t* time )
{
  uint8_t overflow;    // holds overflow information
  bspIState_t s;
  mrfi_Time_t t;
  

  t.state.lsb.order_test = 1; // identify the compiler ordering
  t.state.lsb.order_other = 0;

  // fill in the time structure elements
  t.state.lsb.raw = 1; // indicate this is raw

  t.limit = BSP_TIMER_CLK_KHZ; // indicate the limiting value of the timer

  // fill in the timer parameters
#if BSP_TIMER_SIZE == 8 // if using an 8 bit timer
  t.rollover = MRFI_ROLLOVER_LIMIT;
  t.threshold = MRFI_ROLLOVER_ROLLOVERS - MRFI_ROLLOVER_EXTRAS;
  t.state.lsb.phy_offset = 1;
#else // for all other timers
  t.state.lsb.phy_offset = MRFI_TIMER_SZ;
#endif
  // indicate our endianess
  t.state.lsb.little_endian = ( ( BSP_LITTLE_ENDIAN != 0 ) ? 1 : 0 );

  // hold off interrupts so we get a clean snapshot of the time value
  BSP_ENTER_CRITICAL_SECTION( s );

  // capture the timer count value checking for an overflow in the processs
  BSP_TIMER_GET_TIMER_COUNT( t.timer );     // get the timer count value
  overflow = BSP_TIMER_CHECK_OVERFLOW_FLAG( ); // get overflow status

  t.milliseconds = MRFI_Time; // get current milliseconds count

  FHSS_ACTIVE( t.hopCount = sHopCount ); // get current hop count value

  FHSS_ACTIVE( t.logicalChnl = sLogicalChannel ); // get current channel

  BSP_EXIT_CRITICAL_SECTION( s ); // copy complete, re-enable interrupts

  t.state.lsb.overflow = ( ( overflow != 0) ? 1 : 0 );

  MRFI_TIME_COMPRESS_TO_BUFFER( &t );
  
  memcpy( time, &t, sizeof( mrfi_Time_t ) );

  return;
}

/**************************************************************************************************
 * @fn          MRFI_CookTime
 *
 * @brief       Corrects the raw time stamp if an overflow was detected during
 *                its acquisition.
 *
 * @param       time
 *                a pointer to a BSP_RawTime_t time structure holding a raw time stamp
 *
 * @return      void (nothing)
 **************************************************************************************************
 */
void MRFI_CookTime( mrfi_Time_t* time )
{
  // if this time needs converting
  if( ( ( time->state.lsb.order_test != 0 )
       ? time->state.lsb.raw : time->state.msb.raw ) )
  {
    uint16_t carry;
    uint32_t fraction = 0;

    // if endianess of raw time stamp does not match our endiness
    if( ( ( time->state.lsb.order_test != 0 )
              ? time->state.lsb.little_endian : time->state.msb.little_endian )
        != ( ( BSP_LITTLE_ENDIAN != 0 ) ? 1 : 0 ) )
    { // swap endianess of the value
      REVERSE_32(time->limit);
      REVERSE_16(time->timer);
      REVERSE_32(time->milliseconds);
      FHSS_ACTIVE( REVERSE_16(time->hopCount) );
      if( time->state.lsb.order_test != 0 )
        time->state.lsb.little_endian = ( ( BSP_LITTLE_ENDIAN != 0 ) ? 1 : 0 );
      else
        time->state.msb.little_endian = ( ( BSP_LITTLE_ENDIAN != 0 ) ? 1 : 0 );
    }

    // if an 8 bit timer, change the rollover and extras counts into 16 bit counts
    if( ( (time->state.lsb.order_test != 0 )
         ? time->state.lsb.phy_offset : time->state.msb.phy_offset ) == 1 )
    {
      // get number of rollovers that have occurred
      fraction = ( time->timer >> 8 ) & 0xFF;
      fraction *= time->rollover; // calculate the number of counts from rollovers

      // if need to add in extra counts for some rollovers
      if( ( ( time->timer >> 8 ) & 0xFF ) > time->threshold )
        // add in those extra counts
        fraction += ( ( time->timer >> 8 ) & 0xFF ) - time->threshold;

      // finally, add in any additional partial rollover counts
      fraction += time->timer & 0xFF;
      if( time->state.lsb.order_test != 0 )
        time->state.lsb.phy_offset = MRFI_TIMER_SZ;
      else
        time->state.msb.phy_offset = MRFI_TIMER_SZ;
    }
    else // if a 16 bit timer, just fill in the values directly from the time array
      fraction = time->timer;

    // at this point, the two physical timer bytes will look like a 16 bit timer value

    // change the physical timer values into a fraction of a millisecond

    // normalize value to fractional milliseconds
    fraction <<= 16;
    fraction /= time->limit;
    if( ( ( time->state.lsb.order_test != 0 ) // if timer overflow occurred
              ? time->state.lsb.overflow : time->state.msb.overflow ) != 0
            && fraction < 0x8000UL )
      carry = 1;
    else // if no overflow
      carry = 0; // clear the carry value, clear the upper byte too
    // save the fractional value back into the time structure
    time->timer = fraction & 0xFFFFUL;

    // now, the physical timer bytes have been normalized to a fraction of a ms

    if( carry != 0 ) // if need to propagate an overflow carry
    {
      // adjust hop count value
      FHSS_ACTIVE( time->hopCount += carry ); // add in any carry value

      // add carry into milliseconds count
      time->milliseconds += carry;
    }

    if( time->state.lsb.order_test != 0 )
      time->state.lsb.raw = 0;
    else
      time->state.msb.raw = 0;
  }

  return;
}

/**************************************************************************************************
 * @fn          MRFI_GetTimeDelta
 *
 * @brief       Calculates the time delta of end-start and places it in dest.
 *              Also calculates the delta between the respective hop counts too.
 *                NOTE: it is allowed to have dest be the same as either start or end.
 *                NOTE: both start and end will be converted to cooked times
 *                      regardless what form they were in when this function was called.
 *
 * @param       dest  -- where to place the differenc of times
 *              start -- the starting time
 *              end   -- the ending time
 *
 * @return      void (nothing)
 **************************************************************************************************
 */
void MRFI_GetTimeDelta( mrfi_Time_t* dest, mrfi_Time_t* start, mrfi_Time_t* end )
{
  mrfi_Time_t result;
  MRFI_CookTime( start ); // make sure the start time is in normalized form
  MRFI_CookTime( end );   // make sure the end time is in normalized form

  // calculate time delta
  result.milliseconds = end->milliseconds; // initialize accumulator
  if( end->timer < start->timer ) // if a borrow would occur
    result.milliseconds -= 1; // propagate it
  result.milliseconds -= start->milliseconds; // subtract the milliseconds
  result.timer = end->timer - start->timer; // subtract the timers

  // indicate this time is in normalized form
  result.state.lsb.order_test = 1; // identify the compiler ordering
  result.state.lsb.order_other = 0;
  result.state.lsb.raw = 0;

  *dest = result; // copy the result into the destination

  return;
}

/**************************************************************************************************
 * @fn          MRFI_SetTime
 *
 * @brief       Sets the current time value to that passed
 *
 * @param       t    -- the time value to set the time clock to
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetTime( mrfi_Time_t* t )
{
  bspIState_t s;
  BSP_ENTER_CRITICAL_SECTION( s );

  // fill in the non timer values, don't worry about the timer values as they are
  // fractional and this should get us close enough.
    MRFI_Time = t->milliseconds;
    FHSS_ACTIVE( sHopCount = t->hopCount );
    FHSS_ACTIVE( sLogicalChannel = t->logicalChnl );

  BSP_EXIT_CRITICAL_SECTION( s );

  return;
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsecLong -- Frequency Hopping Enabled
 *
 * @brief       Delay the number of microseconds specified by the parameters passed as
 *                 ms * 1000 + us
 *              If the parameter <term> is not NULL, then it must point to a function
 *              which is called during the delay period and if that function returns
 *              true the delay period is truncated at that point.  If the parameter
 *              <term> is NULL or never returns true, the function returns after the
 *              entire delay period has transpired.
 *
 * input parameters
 * @param   ms   - number of milliseconds to delay
 * @param   us   - number of microseconds to delay
 * @param   term - function pointer to semaphore test function to truncate delay period
 *
 * @return      status; true if timeout truncated due to semaphore test, false otherwise
 ****************************************************************************************************
 */
bool Mrfi_DelayUsecLong(uint32_t ms, uint16_t us, TimeoutTerminator_t term)
{
  bool status = false;
  bool overflow;
#if BSP_LITTLE_ENDIAN != 0
  union
  {
    struct
    {
      uint16_t tmr;
      uint32_t time;
    } formatted;
    struct
    {
      uint8_t tmr_lo;
      uint8_t tmr_hi;
      uint8_t ms_lo;
      uint8_t ms_mid_lo;
      uint8_t ms_mid_hi;
      uint8_t ms_hi;
    } bytes;
    struct
    {
      uint16_t tmr;
      uint16_t ms_lo;
      uint16_t ms_hi;
    } words;
    struct
    {
      uint32_t fixed;
      uint16_t ms_hi;
    } dword;
  } time;
#else
  union
  {
    struct
    {
      uint32_t time;
      uint16_t tmr;
    } formatted;
    struct
    {
      uint8_t ms_hi;
      uint8_t ms_mid_hi;
      uint8_t ms_mid_lo;
      uint8_t ms_lo;
      uint8_t tmr_hi;
      uint8_t tmr_lo;
    } bytes;
    struct
    {
      uint16_t ms_hi;
      uint16_t ms_lo;
      uint16_t tmr;
    } words;
    struct
    {
      uint16_t ms_hi;
      uint32_t fixed;
    } dword;
  } time;
#endif

  // capture the current time stamp
  BSP_CRITICAL_STATEMENT(
  BSP_TIMER_GET_TIMER_COUNT( time.formatted.tmr );
  overflow = BSP_TIMER_CHECK_OVERFLOW_FLAG( );
  time.formatted.time = MRFI_Time );

  if( overflow != 0 && time.bytes.tmr_lo < 100 ) // if an overflow occurred
    time.formatted.time++; // update captured ms value

  // remove any milliseconds from the microsecond value
  // we may be here for several loops but since the loop period is much shorter
  // than the milliseconds we are counting we are ahead of the game.
  while( us > 999 )
  {
    us -= 1000; // remove this millisecond
    ms++;       // add it back in
    // manage the FHSS schedule, only allow pumps if there is at least 5
    // milliseconds of time delay remaining
    FHSS_ACTIVE( nwk_pllBackgrounder( stx_active != false || ms < 5 ) );
  }

  // calculate the number of timer counts that are needed to expire
  // i.e. us /= 1000;
  // this ugly form is necessary as the BSP_CLK_MHZ macro could be a float
  // and that would bring in lots of unnecessary library code, also we don't
  // want to do a divide by a non radix two value, just the multiply
  {
    uint32_t tmp = us;
    tmp *= ( BSP_TIMER_CLK_KHZ * 32UL ) / 125UL;
    us = *( (uint16_t*)( (uint8_t*) &tmp + 1
         + ( ( BSP_LITTLE_ENDIAN == 0 ) ? 1 : 0 ) ) ); // shift right by 8
  }


#if BSP_TIMER_SIZE == 8
  // if an eight bit timer then we need to add in the microseconds a little differently
  // note that we ignore any rollover extras

  // first add in the number of rollovers that need to occur
  while( us >= BSP_ROLLOVER_LIMIT )
  {
    us -= BSP_ROLLOVER_LIMIT; // remove this rollover value
    time.bytes.tmr_hi++;      // log this rollover value
  }

  // next, see if the sum will force yet another rollover
  // note that even though we only use the low byte of the timer, the microsecond
  // value is a 16 bit value so the test is valid even if the sum is greater
  // than 255.
  if( time.bytes.tmr_lo + us >= BSP_ROLLOVER_LIMIT ) // if so,
  {
    time.bytes.tmr_hi++; // update the rollover count
    time.bytes.tmr_lo -= BSP_ROLLOVER_LIMIT; // adjust the timer value
  }

  // add remaining micro seconds, logicically it cannot overflow at this point
  time.bytes.tmr_lo += (uint8_t) us;

  // finally, adjust rollover count if it has overflowed
  while( time.bytes.tmr_hi >= BSP_ROLLOVER_ROLLOVERS )
  {
    time.bytes.tmr_hi -= BSP_ROLLOVER_ROLLOVERS;
    ms++;
  }
#else // if a sixteen bit timer
  time.formatted.tmr += us; // add in final timer counts
  if( time.formatted.tmr < us ) // if an overflow occurred
    ms++; // account for it

  // adjust for any rollover values ( should only happen once but possibly more
  while( time.formatted.tmr >= BSP_ROLLOVER_LIMIT )
  {
    time.formatted.tmr -= BSP_ROLLOVER_LIMIT; // remove this millisecond
    ms++; // account for it
  }
#endif

  // if there is at least one millisecond boundary to cross
  if( ms != 0 )
  {
    bool test;

    time.formatted.time += ms; // combine millisecond values

    // wait for millisecond count to reach delay time
    do // wait for milliseconds to timeout
    {
      if( term != NULL && ( status = term( ) ) != false ) // if the semaphore triggers
        goto bail; // exit the delay routine

      if( time.bytes.ms_lo != ( MRFI_Time & 0xFF ) ) // have we seen least one ms
      {
        // manage the FHSS schedule, only allow pumps if there is at least 5
        // milliseconds of time delay remaining
        FHSS_ACTIVE( nwk_pllBackgrounder( stx_active != false
                                          || time.formatted.time - ms < 5 ) );
      }

      // make sure time is counted even if interrupts are disabled.
      if( !BSP_INTERRUPTS_ARE_ENABLED() && BSP_TIMER_CHECK_OVERFLOW_FLAG( ) )
      {
        BSP_TIMER_MAN_CLEAR_OVERFLOW_FLAG( ); // clear the flag
        MRFI_Time++; // count the millisecond
      }

      BSP_CRITICAL_STATEMENT( ms = MRFI_Time ); // update the test value

      // if in upper half of values, work in signed mode
      if( ( time.bytes.ms_hi & 0x80) != 0 )
        test = (int32_t)ms < (int32_t)time.formatted.time;
      else // if no overflow occurred, work in unsigned mode
        test = ms < time.formatted.time;

    } while( test != false );
  }

  // at this point all we have left is the remaining number of microseconds
  // in the current millisecond before the delay is complete

  ms = time.dword.fixed; // get 16.16 expiration time
  do
  {
    // get snapshot of current timer value
    BSP_CRITICAL_STATEMENT(
      BSP_TIMER_GET_TIMER_COUNT( time.formatted.tmr );
      time.words.ms_lo = (uint16_t)MRFI_Time;
    );
  }
  // wait for microseconds to expire or semaphore to be set
  while( time.dword.fixed < ms
          && ( term == NULL || ( status = term( ) ) == false ) );

bail: // jump to here on early exit
  return status; // either timed out or the semaphore was asserted
}

/****************************************************************************************************
 * @fn          Mrfi_CheckSem -- Frequency Hopping Enabled 
 *
 * @brief       tests the sKillSem status 
 *
 * input parameters 
 * @param       none 
 *
 * @return      true if sKillSem != 0 
 ****************************************************************************************************
 */
static bool Mrfi_CheckSem( void )
{
  return sKillSem;
}

/**************************************************************************************************
 * @fn          MRFI_ReplyDelay -- Frequency Hopping Enabled
 *
 * @brief       Delay number of milliseconds scaled by data rate.  Check semaphore for
 *              early-out.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_ReplyDelay()
{
  BSP_CRITICAL_STATEMENT( sReplyDelayContext = 1 );
  Mrfi_DelayUsecLong( sReplyDelayScalar, 0, Mrfi_CheckSem );
  BSP_CRITICAL_STATEMENT( sKillSem = sReplyDelayContext = 0 );
}

#else // MRFI_TIMER_ALWAYS_ACTIVE

/****************************************************************************************************
 * @fn          Mrfi_DelayUsec -- Frequency Hopping Disabled
 *
 * @brief       Execute a delay loop using HW timer. The macro actually used to do the delay
 *              is not thread-safe. This routine makes the delay execution thread-safe by breaking
 *              up the requested delay into small chunks and executing each chunk as a critical
 *              section. The chunk size is choosen to be the smallest value used by MRFI. The delay
 *              is only approximate because of the overhead computations. It errs on the side of
 *              being too long.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
void Mrfi_DelayUsec(uint16_t howLong)
{
  bspIState_t s;
  uint16_t count = howLong/MRFI_MAX_DELAY_US;

  if (howLong)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(s);
    } while (count--);
  }

  return;
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsecSem -- Frequency Hopping Disabled
 *
 * @brief       Execute a delay loop using a HW timer. See comments for Mrfi_DelayUsec().
 *              Delay specified number of microseconds checking semaphore for
 *              early-out. Run in a separate thread when the reply delay is
 *              invoked. Cleaner then trying to make MRFI_DelayUsec() thread-safe
 *              and reentrant.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
static void Mrfi_DelayUsecSem(uint16_t howLong)
{
  bspIState_t s;
  uint16_t count = howLong/MRFI_MAX_DELAY_US;

  if (howLong)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(s);
      if (sKillSem)
      {
        break;
      }
    } while (count--);
  }

  return;
}

/**************************************************************************************************
 * @fn          MRFI_DelayMs -- Frequency Hopping Disabled
 *
 * @brief       Delay the specified number of milliseconds.
 *
 * @param       milliseconds - delay time
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_DelayMs(uint16_t milliseconds)
{
  while (milliseconds)
  {
    Mrfi_DelayUsec( APP_USEC_VALUE );
    milliseconds--;
  }
}

/**************************************************************************************************
 * @fn          MRFI_ReplyDelay -- Frequency Hopping Disabled
 *
 * @brief       Delay number of milliseconds scaled by data rate. Check semaphore for
 *              early-out. Run in a separate thread when the reply delay is
 *              invoked. Cleaner then trying to make MRFI_DelayMs() thread-safe
 *              and reentrant.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_ReplyDelay()
{
  bspIState_t s;
  uint16_t    milliseconds = sReplyDelayScalar;

  BSP_ENTER_CRITICAL_SECTION(s);
  sReplyDelayContext = 1;
  BSP_EXIT_CRITICAL_SECTION(s);

  while (milliseconds)
  {
    Mrfi_DelayUsecSem( APP_USEC_VALUE );
    if (sKillSem)
    {
      break;
    }
    milliseconds--;
  }

  BSP_ENTER_CRITICAL_SECTION(s);
  sKillSem           = 0;
  sReplyDelayContext = 0;
  BSP_EXIT_CRITICAL_SECTION(s);
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsecLong -- Frequency Hopping Disabled
 *
 * @brief       Delay the number of microseconds specified by the parameters passed as
 *                 ms * 1000 + us
 *              If the parameter <term> is not NULL, then it must point to a function
 *              which is called during the delay period and if that function returns
 *              true the delay period is truncated at that point.  If the parameter
 *              <term> is NULL or never returns true, the function returns after the
 *              entire delay period has transpired.
 *
 * input parameters
 * @param   ms   - number of milliseconds to delay
 * @param   us   - number of microseconds to delay
 * @param   term - function pointer to semaphore test function to truncate delay period
 *
 * @return      status; true if timeout truncated due to semaphore test, false otherwise
 ****************************************************************************************************
 */
bool Mrfi_DelayUsecLong(uint32_t ms, uint16_t us, TimeoutTerminator_t term)
{
  bool timeout = false;
  bspIState_t s;
  uint16_t count;

  while (!timeout && ms)
  {
    count = APP_USEC_VALUE / MRFI_MAX_DELAY_US;
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(s);
      if (term != NULL)
        timeout = term( );
    } while (!timeout && count--);
    ms--;
  }
  count = us/MRFI_MAX_DELAY_US;
  if (!timeout && us)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(s);
      if (term != NULL)
        timeout = term( );
    } while (!timeout && count--);
  }
  return timeout;
}

#endif // MRFI_TIMER_ALWAYS_ACTIVE

/**************************************************************************************************
 * @fn          MRFI_PostKillSem
 *
 * @brief       Post to the loop-kill semaphore that will be checked by the iteration loops
 *              that control the delay thread.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_PostKillSem(void)
{

  if (sReplyDelayContext)
  {
    sKillSem = 1;
  }

  return;
}

/**************************************************************************************************
 * @fn          Mrfi_RandomBackoffDelay
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RandomBackoffDelay(void)
{
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    MRFI_DelayUsec( sBackoffHelper * (MRFI_RandomByte() & 0x0F) + 1 );
#else
  uint8_t backoffs;
  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (MRFI_RandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  for (i=0; i<backoffs; i++)
  {
    Mrfi_DelayUsec( sBackoffHelper );
  }
#endif
}

/**************************************************************************************************
 * @fn          MRFI_Rssi
 *
 * @brief       Returns "live" RSSI value
 *
 * @param       none
 *
 * @return      RSSI value in units of dBm.
 **************************************************************************************************
 */
int8_t MRFI_Rssi(void)
{
  /* Radio must be in RX state to measure rssi. */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  /* Wait for the RSSI to be valid:
   * Just having the Radio ON is not enough to read
   * the correct RSSI value. The Radio must in RX mode for
   * a certain duration. This duration depends on
   * the baud rate and the received signal strength itself.
   */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
  MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

  /* Convert RSSI to decimal and do offset compensation. */
  return( Mrfi_CalculateRssi(RSSI) );
}

/**************************************************************************************************
 * @fn          Mrfi_CalculateRssi
 *
 * @brief       Does binary to decimal conversiont and offset compensation.
 *
 * @param       none
 *
 * @return      RSSI value in units of dBm.
 **************************************************************************************************
 */
int8_t Mrfi_CalculateRssi(uint8_t rawValue)
{
  int16_t rssi;

  /* The raw value is in 2's complement and in half db steps. Convert it to
   * decimal taking into account the offset value.
   */
  if(rawValue >= 128)
  {
    rssi = (int16_t)(rawValue - 256)/2 - MRFI_RSSI_OFFSET;
  }
  else
  {
    rssi = (rawValue/2) - MRFI_RSSI_OFFSET;
  }

  /* Restrict this value to least value can be held in an 8 bit signed int */
  if(rssi < -128)
  {
    rssi = -128;
  }

  return rssi;
}

/**************************************************************************************************
 * @fn          MRFI_GetRadioState
 *
 * @brief       Returns the current radio state.
 *
 * @param       none
 *
 * @return      radio state - off/idle/rx
 **************************************************************************************************
 */
uint8_t MRFI_GetRadioState(void)
{
  return mrfiRadioState;
}

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
/**************************************************************************************************
 * @fn          MRFI_TimerISR
 *
 * @brief       Manages the timer interrupts.
 *              Everytime the timer overflows, this routine increments the
 *              remaining bytes in the MRFI_Time array.
 *              It also manages the 1 ms event counter sPending_ms_Events
 *              See the above macro for implementation
 *
 * @param       none
 *
 * @return      void (nothing)
 **************************************************************************************************
 */
//#if defined __IAR_SYSTEMS_ICC__ && defined BSP_TIMER_SIZE && BSP_TIMER_SIZE == 8
#if defined __IAR_SYSTEMS_ICC__
  // when using the 8 bit timers, use idata stack to minimize isr overhead time
  #pragma vector=BSP_TIMER_VECTOR
  __idata_reentrant __interrupt void Mrfi_TimerISR(void);
  #pragma vector=BSP_TIMER_VECTOR
  __idata_reentrant __interrupt void Mrfi_TimerISR(void)
#else
  BSP_ISR_FUNCTION(Mrfi_TimerISR, BSP_TIMER_VECTOR)
#endif
{
  bspIState_t istate = __bsp_GET_ISTATE__();
  BSP_ENABLE_INTERRUPTS();

#if BSP_TIMER_SIZE == 8
  /* For 8 bit timers we emulate the upper byte of the timer in software, sort of.
   * Actually, the upper byte does not count the number of 256 tick rollovers but
   * instead it counts some lower value which is used to make the final rollover
   * value match what would be needed for a 1ms compare time.  Since multiple
   * rollovers are required for the 8 bit timer, at some point, to maintain
   * the most resolution possible, the compare value needs to be incremented so
   * the final rollover is as close as possible to a 1ms timing.  The following
   * code does this and if we are not on the final rollover value, then the ISR
   * is exited before updating the 1ms time counter array.
   */
    {
      if( sTimerCntHi == 0 )
        BSP_TIMER_SET_OVERFLOW_VALUE( MRFI_ROLLOVER_LIMIT + 1 );
      if( sTimerCntHi == MRFI_ROLLOVER_EXTRAS )
        BSP_TIMER_SET_OVERFLOW_VALUE( MRFI_ROLLOVER_LIMIT );
      if( ++sTimerCntHi == MRFI_ROLLOVER_ROLLOVERS )
      {
        static union{ int32_t modulation; int8_t bytes[4]; };
        modulation += sTmrRateOffset;
        {
          int16_t limit = bytes[3];
          limit += MRFI_ROLLOVER_LIMIT;
          BSP_TIMER_SET_OVERFLOW_VALUE( limit );
        }
        bytes[3] = 0; // clear upper byte
        sTimerCntHi = 0;
      }
      else /* if the counter has not rolled over */
      {
        __bsp_RESTORE_ISTATE__(istate);
        return; /* bail here because 1 ms has not transpired */
      }
    }
#else // if 16 bit timer code is needed
    {
      static union{ int32_t modulation; int8_t bytes[4]; };
      modulation += sTmrRateOffset;
      {
        uint16_t limit = bytes[3];
        limit += MRFI_ROLLOVER_LIMIT;
        BSP_TIMER_SET_OVERFLOW_VALUE( limit );
      }
      bytes[3] = 0; // clear upper byte
    }
#endif

  MRFI_Time++;

#ifdef FREQUENCY_HOPPING
  if( sHopCount == 0 ) /* if ready to hop frequencies */
  {
    sHopCount = sHopRate;
    sHopNowSem = 1; /* indicate a frequency change is due */

    // manage the channel indexer in the isr so if a semaphore is missed,
    // the channel index is always up to date with the current channel
    sLogicalChannel++; // increment the channel index

    // check to see if the channel index is rolling over
    #if MRFI_NUM_LOGICAL_CHANS <= 255
      if( sLogicalChannel >= MRFI_NUM_LOGICAL_CHANS )
        sLogicalChannel = 0;
    #endif

  }
  else /* if not ready to hop */
    sHopCount--; /* decrement hop counter */
  
  if( sHopCount > FHSS_HOP_MARGIN + 2
      && sHopCount < MRFI_HOP_TIME_ms - 1 - FHSS_HOP_MARGIN )
    sTxValid = true;
  else
    sTxValid = false;
#endif

  if( sOne_ms_event_hook != NULL ) /* if a 1 millisecond hook function exists */
    sOne_ms_event_hook( ); /* then call it */

  BSP_TIMER_CLEAR_OVERFLOW_FLAG( ); /* clear the event */

  __bsp_RESTORE_ISTATE__(istate);

  return;
}

#endif // MRFI_TIMER_ALWAYS_ACTIVE

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#if (MRFI_DMA_CHAN != 0)
#error "ERROR: Code implementation requires use of DMA channel zero."
/*  Using a channel other than zero is not difficult to implement.  The hardware
 *  requires channels 1-4 to use a common configuration structure.  For this module
 *  to use a channel other than zero, this data structure would need to be integrated
 *  with the external code.  Hooks are provided to make this as easy as possible.
 */
#endif


#define MRFI_RADIO_TX_FIFO_SIZE     64  /* from datasheet */

/* verify largest possible packet fits within FIFO buffer */
#if ((MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE) > MRFI_RADIO_TX_FIFO_SIZE)
#error "ERROR:  Maximum possible packet length exceeds FIFO buffer.  Decrease value of maximum application payload."
#endif

/* verify that the SmartRF file supplied is compatible */
#if ((!defined SMARTRF_RADIO_CC2510) && \
     (!defined SMARTRF_RADIO_CC2511) && \
     (!defined SMARTRF_RADIO_CC1110) && \
     (!defined SMARTRF_RADIO_CC1111))
#error "ERROR:  The SmartRF export file is not compatible."
#endif


/**************************************************************************************************
*/
