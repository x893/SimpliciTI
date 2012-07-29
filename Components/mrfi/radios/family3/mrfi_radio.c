/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007-2009 Texas Instruments Incorporated.  All rights reserved.

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
 *   Radios: CC2520
 *   Primary code file for supported radios.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <string.h>
#include "bsp_macros.h"
#include "bsp.h"
#include "mrfi_spi.h"
#include "mrfi.h"
#include "mrfi_defs.h"
#include "bsp_external/mrfi_board_defs.h"
#include "nwk_pll.h"
#include "bsp_leds.h" // debug only


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *     IEEE 802.15.4 definitions
 *   - - - - - - - - - - - - - - -
 */
#define IEEE_PHY_PACKET_SIZE_MASK   0x7F
#define IEEE_USECS_PER_SYMBOL       16


/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */
#ifdef MRFI_PA_LNA_ENABLED
uint8_t mrfiLnaHighGainMode = 1;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC2520)

  #define MRFI_RSSI_OFFSET    76   /* no units */

  /*
   *  For RSSI to be valid, we must wait for 20 symbol periods:
   *  - 12 symbols to go from idle to rx state
   *  - 8 symbols to calculate the RSSI value
   */
  #define MRFI_RSSI_VALID_DELAY_US    (20 * IEEE_USECS_PER_SYMBOL)

  #define MRFI_VREG_SETTLE_TIME_USECS        100    /* microseconds */

#else
  #error "ERROR: Some of the radio params are not defined for this radio."
#endif

#define MRFI_SYNC_WORD_SIZE 1
#define MRFI_PREAMBLE_SIZE  ( ( ( MDMCTRL0 >> 1 ) & 0xF ) + 2 )
#define MRFI_CRC_SIZE       ( ( FRMCTRL0 >> 5 ) & 2 )
#define MRFI_MAX_TRANSMIT_BYTES ( MRFI_MAX_FRAME_SIZE + MRFI_SYNC_WORD_SIZE + MRFI_PREAMBLE_SIZE + MRFI_CRC_SIZE )
#define MRFI_TRANSMIT_BIT_PERIOD_us 4.0
#define MRFI_MAX_TRANSMIT_TIME_us ((unsigned long)( MRFI_TRANSMIT_BIT_PERIOD_us * 8 * MRFI_MAX_TRANSMIT_BYTES + 500))

#define MRFI_DSN_OFFSET                     __mrfi_DSN_OFS__
#define MRFI_FCF_OFFSET                     __mrfi_FCF_OFS__


/*
 *  The FCF (Frame Control Field) will always have this value for *all* frames:
 *
 *    bits    description                         setting
 *  --------------------------------------------------------------------------------------
 *      0-2   Frame Type                          001 - data frame
 *        3   Security Enabled                      0 - security disabled
 *        4   Frame Pending                         0 - no pending data
 *        5   Ack Request                           0 - no Ack request
 *        6   PAN ID Compression                    0 - no PAN ID compression
 *        7   Reserved                              0 - reserved
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *      8-9   Reserved                             00 - reserved
 *    10-11   Destination Addressing Mode          10 - PAN ID + 16-bit short address
 *    12-13   Frame Version                        00 - IEEE Std 802.15.4-2003
 *    14-15   Source Addressing Mode               10 - PAN ID + 16-bit short address
 *
 */
#define MRFI_FCF_0_7              (0x01)
#define MRFI_FCF_8_15             (0x88)
#define MRFI_MIN_SMPL_FRAME_SIZE  (MRFI_HEADER_SIZE + NWK_HDR_SIZE)

#ifdef MRFI_TIMER_ALWAYS_ACTIVE

// re-map static functions promoted to public for backwards compatibility
#define Mrfi_DelayUsec( a ) MRFI_DelayUsec( a )
#define Mrfi_DelayMs( a ) MRFI_DelayMs( a )

#endif // !MRFI_TIMER_ALWAYS_ACTIVE

/* Random number generator params */
#define MRFI_RANDOM_OFFSET                   67
#define MRFI_RANDOM_MULTIPLIER              109

#define MRFI_FILTER_ADDRESS_SET       BV(1)
#define MRFI_FILTER_ADDRESS_ENABLED   BV(2)

/* ------------------------------------------------------------------------------------------------
 *                                          Radio Abstraction
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC2520)
  #define MRFI_RADIO_PARTNUM          0x84
  #define MRFI_RADIO_MIN_VERSION      0x00
#else
  #error "ERROR: Missing or unrecognized radio."
#endif

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls 
   * to the microsecond routine which can be calibrated independently.
   */
#if defined(SW_TIMER)
#define APP_USEC_VALUE    1000
#else
#define APP_USEC_VALUE    1000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */
/*
 *  Logical channel table - this table translates logical channel into
 *  actual radio channel number. Each derived channel is
 *  masked with 0xFF to prevent generation of an illegal channel number.
 *
 *  This table is easily customized.  Just replace or add entries as needed.
 *  If the number of entries changes, the corresponding #define must also
 *  be adjusted.  It is located in mrfi_defs.h and is called __mrfi_NUM_LOGICAL_CHANS__.
 *  The static assert below ensures that there is no mismatch.
 */
#ifndef FREQUENCY_HOPPING
static const uint8_t mrfiLogicalChanTable[] =
{
  15,
  20,
  25,
  26
};
#else
static const uint8_t mrfiLogicalChanTable[] =
{
  20, 11, 25, 15, 26,
  11, 20, 25, 26, 15,
  26, 25, 11, 15, 20,
  11, 20, 15, 26, 25,
  11, 15, 20, 26, 25
};
#endif

/* verify number of table entries matches the corresponding #define */
BSP_STATIC_ASSERT(__mrfi_NUM_LOGICAL_CHANS__ == ((sizeof(mrfiLogicalChanTable)/sizeof(mrfiLogicalChanTable[0])) * sizeof(mrfiLogicalChanTable[0])));

#if !defined(MRFI_PA_LNA_ENABLED)
/*
 *  RF Power setting table - this table translates logical power value
 *  to radio register setting.  The logical power value is used directly
 *  as an index into the power setting table. The values in the table are
 *  from low to high. The default settings set 3 values: -20 dBm, -10 dBm,
 *  and 0 dBm. The default at startup is the highest value. Note that these
 *  are approximate depending on the radio. Information is taken from the
 *  data sheet.
 *
 *  This table is easily customized.  Just replace or add entries as needed.
 *  If the number of entries changes, the corresponding #define must also
 *  be adjusted.  It is located in mrfi_defs.h and is called __mrfi_NUM_POWER_SETTINGS__.
 *  The static assert below ensures that there is no mismatch.
 */
static const uint8_t mrfiRFPowerTable[] =
{
  0x03,
  0x2C,
  0x32
};
#else  /* !MRFI_PA_LNA_ENABLED */
/* If LNA enabled we use 5 dBm, 10dBm and 15dBm as the entries defaulting to 15 */
static const uint8_t mrfiRFPowerTable[] =
{
  0x49,
  0x79,
  0xE0
};
#endif  /* !MRFI_PA_LNA_ENABLED */

/* verify number of table entries matches the corresponding #define */
BSP_STATIC_ASSERT(__mrfi_NUM_POWER_SETTINGS__ == ((sizeof(mrfiRFPowerTable)/sizeof(mrfiRFPowerTable[0])) * sizeof(mrfiRFPowerTable[0])));

/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void Mrfi_FiFoPIsr(void); /* this called from mrfi_board.c */

static void   MRFI_PrepareToTx( mrfiPacket_t * );
static void   MRFI_CompleteTxPrep( mrfiPacket_t * );
static bool mrfi_TxDone( void );
static bool mrfi_IdleTestTimeOut( void );
static void   Mrfi_TurnOnRadioPower(void);
static void   Mrfi_TurnOffRadioPower(void);
static int8_t Mrfi_CalculateRssi(uint8_t rawValue);
static void   Mrfi_SpiSendTxPkt( mrfiPacket_t * pPacket, uint8_t cmd );

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
//static void Mrfi_DelayUsecLong(uint32_t count, TimeoutTerminator_t term);
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
static uint8_t      mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;
static uint8_t      mrfiRndSeed = 0;
static mrfiPacket_t mrfiIncomingPacket;

static uint8_t mrfiFilterAddr[4] = {0, 0, 0, 0};
static uint8_t mrfiAddrFilterStatus = 0x0;
#ifndef FREQUENCY_HOPPING
static uint8_t mrfiCurrentLogicalChannel = 0; /* Default logical channel */
#endif
static uint8_t mrfiCurrentPowerLevel = MRFI_NUM_POWER_SETTINGS - 1;

static bool rx_isr_context = false;

/* reply delay support */
static volatile uint8_t  sKillSem = 0;
static volatile uint8_t  sReplyDelayContext = 0;
static          uint16_t sReplyDelayScalar = 0;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  static bool stx_active = false;
  static MRFI_ms_event_t sOne_ms_event_hook = NULL;
  static int32_t sTmrRateOffset = 0;
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

/* ------------------------------------------------------------------------------------------------
 *                                       Local Macros
 * ------------------------------------------------------------------------------------------------
 */
#ifndef MRFI_TIMER_ALWAYS_ACTIVE

#define MRFI_RSSI_VALID_WAIT()                                                \
{                                                                             \
  int16_t delay = MRFI_RSSI_VALID_DELAY_US;                                   \
  do                                                                          \
  {                                                                           \
    if(mrfiSpiCmdStrobe(SNOP) & RSSI_VALID)                                   \
    {                                                                         \
      break;                                                                  \
    }                                                                         \
    Mrfi_DelayUsec(64); /* sleep */                                           \
    delay -= 64;                                                              \
  }while(delay > 0);                                                          \
}                                                                             \

#endif // !MRFI_TIMER_ALWAYS_ACTIVE

/* See Bug #1 CC2520 errata - swrz024.pdf. Flush must be done twice. */
#define MRFI_RADIO_FLUSH_RX_BUFFER()                                          \
{                                                                             \
  bspIState_t s;                                                              \
  BSP_ENTER_CRITICAL_SECTION(s);                                              \
  mrfiSpiCmdStrobe(SFLUSHRX);                                                 \
  mrfiSpiCmdStrobe(SFLUSHRX);                                                 \
  BSP_EXIT_CRITICAL_SECTION(s);                                               \
}

#define MRFI_RADIO_FLUSH_TX_BUFFER()  mrfiSpiCmdStrobe(SFLUSHTX)
#define MRFI_SAMPLED_CCA()            (SAMPLED_CCA_BV & mrfiSpiReadReg(FSMSTAT1))


/* ------------------------------------------------------------------------------------------------
 *                                       Functions
 * ------------------------------------------------------------------------------------------------
 */

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
  memset(&mrfiIncomingPacket, 0x0, sizeof(mrfiIncomingPacket));

  /* ------------------------------------------------------------------
   *    Configure timer
   *   ----------------------
   */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  sOne_ms_event_hook = NULL;
  /* initialize the timer for operation with the pll if frequency hoppin is enabled */
  Mrfi_TimerInit( );
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  FHSS_ACTIVE( sHopCount = 0 );
  FHSS_ACTIVE( sLogicalChannel = MRFI_RandomByte( ) % MRFI_NUM_LOGICAL_CHANS );
  FHSS_ACTIVE( sHopNowSem = 0 );
  FHSS_ACTIVE( sHopRate = MRFI_HOP_TIME_ms - 1 );

  /* Configure Output lines */
  MRFI_CONFIG_RESETN_PIN_AS_OUTPUT();
  MRFI_CONFIG_VREG_EN_PIN_AS_OUTPUT();

  /* Configure Input lines */
  MRFI_CONFIG_TX_FRAME_DONE_AS_INPUT();
  MRFI_CONFIG_FIFO_AS_INPUT();
  MRFI_CONFIG_FIFOP_AS_INPUT();

  /* Initialize SPI */
  mrfiSpiInit();

  /* Power up the radio chip */
  Mrfi_TurnOnRadioPower();

  /* Confirm that we are talking to the right hardware */
  MRFI_ASSERT(mrfiSpiReadReg(CHIPID) == MRFI_RADIO_PARTNUM);

  /* Random Number Generator:
   * The seed value for the randon number generator logic
   * is derived from the radio.
   */

  /* Set radio in rx mode, but with symbol search disabled. Used for RSSI
   * measurments or when we don't care about the received frames.
   */
  mrfiSpiWriteReg(FRMCTRL0, FRMCTRL0_RESET_VALUE | RX_MODE_RSSI_ONLY);

  /* Turn on the receiver */
  mrfiSpiCmdStrobe(SRXON);

  /*
   *  Wait for RSSI to be valid. RANDOM command strobe can be used
   *  to generate random number only after this.
   */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
  MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE


  /* Get random byte from the radio */
  mrfiRndSeed = mrfiSpiRandomByte();

 /*
  *  The seed value must not be zero.  If it is, the pseudo random sequence
  *  will be always be zero. There is an extremely small chance this seed could
  *  randomly be zero (more likely some type of hardware problem would cause
  *  this). If it is zero, initialize it to something.
  */
  if(mrfiRndSeed == 0)
  {
      mrfiRndSeed = 0x80;
  }

  /* Random number initialization is done. Turn the radio off */
  Mrfi_TurnOffRadioPower();

  /* Initial radio state is - OFF state */
  mrfiRadioState = MRFI_RADIO_STATE_OFF;

  /**********************************************************************************
   *                            Compute reply delay scalar
   *
   * The IEEE radio has a fixed data rate of 250 Kbps. Data rate inference
   * from radio regsiters is not necessary for this radio.
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
   * **********************************************************************************
   */

#define   PHY_PREAMBLE_SYNC_BYTES     8

  {
    uint32_t bits, dataRate = 250000;

    bits = ((uint32_t)((PHY_PREAMBLE_SYNC_BYTES + MRFI_MAX_FRAME_SIZE)*8))*10000;

    /* processing on the peer + the Tx/Rx time plus more */
    sReplyDelayScalar = PLATFORM_FACTOR_CONSTANT + (((bits/dataRate)+5)/10);
  }

#ifndef FREQUENCY_HOPPING
    /* set default channel */
  MRFI_SetLogicalChannel( mrfiCurrentLogicalChannel );
#endif

  /* set default power */
  MRFI_SetRFPwr(mrfiCurrentPowerLevel);

  /* Clean out buffer to protect against spurious frames */
  memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));
  memset(mrfiIncomingPacket.rxMetrics, 0x00, sizeof(mrfiIncomingPacket.rxMetrics));

  BSP_ENABLE_INTERRUPTS();

  /* Random delay: This prevents devices on the same power source from repeated
   *  transmit collisions on power up.
   */
  Mrfi_RandomBackoffDelay();
}


/**************************************************************************************************
 * @fn          MRFI_WakeUp
 *
 * @brief       Wake up radio from off/sleep state.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_WakeUp(void)
{
  /* if radio is asleep, wake it up */
  if(mrfiRadioState == MRFI_RADIO_STATE_OFF)
  {
    /* enter idle mode */
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE

    /* turn on radio power */
    Mrfi_TurnOnRadioPower();

    /* Configure the radio registers. All radio settings that are lost
     * on MRFI_Sleep() call must be restored here. Since we are putting the
     * radio in LPM2 power mode, all register and memory values that are
     * different from reset must be restored.
     */

    MRFI_BOARD_CONFIG_RADIO_GPIO();

#ifdef MRFI_PA_LNA_ENABLED

    /* Init ports */
    MRFI_BOARD_PA_LNA_CONFIG_PORTS();

    if(mrfiLnaHighGainMode)
    {
      /* Set LNA to High Gain Mode */
      MRFI_BOARD_PA_LNA_HGM();
    }
    else
    {
     /* Set LNA to Low Gain Mode */
      MRFI_BOARD_PA_LNA_LGM();
    }

#endif

    /* Set FIFO_P threshold to max (127). Thus a FIFO_P signal is set whenever
     * a full frame is received.
     */
    mrfiSpiWriteReg(FIFOPCTRL, 0x7F);

    /* Accept only DATA frames. Reject CMD/BECAON/ACK frames. */
    mrfiSpiWriteReg(FRMFILT1, 0x10);

    /* Restore the address filter settings */
    if(mrfiAddrFilterStatus & MRFI_FILTER_ADDRESS_SET)
    {
      MRFI_SetRxAddrFilter(mrfiFilterAddr);
    }

    if(mrfiAddrFilterStatus & MRFI_FILTER_ADDRESS_ENABLED)
    {
      MRFI_EnableRxAddrFilter();
    }
    else
    {
      MRFI_DisableRxAddrFilter();
    }

    /* Following values need to be changed from their reset value.
     * See Table-21 CC2520 datasheet.
     */

    mrfiSpiWriteReg(TXPOWER,  mrfiRFPowerTable[mrfiCurrentPowerLevel]);
    mrfiSpiWriteReg(CCACTRL0, 0xF8);
    mrfiSpiWriteReg(MDMCTRL0, 0x85);
    mrfiSpiWriteReg(MDMCTRL1, 0x14);
    mrfiSpiWriteReg(RXCTRL,   0x3F);
    mrfiSpiWriteReg(FSCTRL,   0x5A);
    mrfiSpiWriteReg(FSCAL1,   0x2B);
    mrfiSpiWriteReg(AGCCTRL1, 0x11);
    mrfiSpiWriteReg(ADCTEST0, 0x10);
    mrfiSpiWriteReg(ADCTEST1, 0x0E);
    mrfiSpiWriteReg(ADCTEST2, 0x03);

#ifndef FREQUENCY_HOPPING
    /* set channel */
    MRFI_SetLogicalChannel(mrfiCurrentLogicalChannel);
#endif

  }
}


/**************************************************************************************************
 * @fn          MRFI_SetRxAddrFilter
 *
 * @brief       Set the address used for filtering received packets.
 *
 * @param       pAddr - pointer to address to use for filtering
 *
 * @return      0 - success; 1 - failure
 **************************************************************************************************
 */
uint8_t MRFI_SetRxAddrFilter(uint8_t * pAddr)
{
  /*
   *  Determine if filter address is valid.  Cannot be a reserved value.
   *   -Reserved PAN ID's of 0xFFFF and 0xFFFE.
   *   -Reserved short address of 0xFFFF.
   */
  if ((((pAddr[0] == 0xFF) || (pAddr[0] == 0xFE)) && (pAddr[1] == 0xFF))  ||
        (pAddr[2] == 0xFF) && ((pAddr[3] == 0xFF)))
  {
    /* unable to set filter address */
    return( 1 );
  }

  /* Can access radio only if it is not OFF */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* set the hardware address registers */
    spiWriteRamByte(0x3F2, pAddr[0]); /* PANIDL */
    spiWriteRamByte(0x3F3, pAddr[1]); /* PANIDH */
    spiWriteRamByte(0x3F4, pAddr[2]); /* SHORTADDRL */
    spiWriteRamByte(0x3F5, pAddr[3]); /* SHORTADDRH */
  }

  /* Save the address so we can restore it after sleep. */
  mrfiFilterAddr[0] = pAddr[0];
  mrfiFilterAddr[1] = pAddr[1];
  mrfiFilterAddr[2] = pAddr[2];
  mrfiFilterAddr[3] = pAddr[3];

  /* remember if address was set */
  mrfiAddrFilterStatus |= MRFI_FILTER_ADDRESS_SET;

  /* successfully set filter address */
  return( 0 );
}


/**************************************************************************************************
 * @fn          MRFI_EnableRxAddrFilter
 *
 * @brief       Enable received packet filtering.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_EnableRxAddrFilter(void)
{
  MRFI_ASSERT( mrfiAddrFilterStatus & MRFI_FILTER_ADDRESS_SET ); /* filter address not set */

  mrfiAddrFilterStatus |= MRFI_FILTER_ADDRESS_ENABLED;

  /* Can access radio only if it is not OFF */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* enable hardware filtering on the radio */
    mrfiSpiBitSet(FRMFILT0, 0);
  }
}


/**************************************************************************************************
 * @fn          MRFI_DisableRxAddrFilter
 *
 * @brief       Disable received packet filtering.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_DisableRxAddrFilter(void)
{
  mrfiAddrFilterStatus &= ~(MRFI_FILTER_ADDRESS_ENABLED);

  /* Can access radio only if it is not OFF */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* disable hardware filtering on the radio */
    mrfiSpiBitClear(FRMFILT0, 0);
  }
}


/**************************************************************************************************
 * @fn          MRFI_PrepareToTx
 *
 * @brief       Setup fifo and other variables for transmit
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      None
 **************************************************************************************************
 */
static void MRFI_PrepareToTx( mrfiPacket_t * pPacket )
{
  uint8_t txBufLen;
  uint8_t frameLen;
  uint8_t *p;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE               //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
stx_active = true; // indicate we are in the act of transmitting   //<<<<<<<<<<<<<<<<<<<<<<<
#endif // MRFI_TIMER_ALWAYS_ACTIVE            //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  /* flush FIFO of any previous transmit that did not go out */
  MRFI_RADIO_FLUSH_TX_BUFFER();

  /* set point at beginning of outgoing frame */
  p = &pPacket->frame[MRFI_LENGTH_FIELD_OFS];

  /* get number of bytes in the packet (does not include the length byte) */
  txBufLen = *p;

  /*
   *  Write the length byte to the FIFO.  This length does *not* include the length field
   *  itself but does include the size of the FCS (generically known as RX metrics) which
   *  is generated automatically by the radio.
   */
  frameLen = txBufLen + MRFI_RX_METRICS_SIZE;

  mrfiSpiWriteTxFifo(&frameLen, 1);

  /* skip the length field which we already sent to FIFO. */
  p++;

#ifndef NWK_PLL
  /* write packet bytes to FIFO */
  mrfiSpiWriteTxFifo(p, txBufLen);
#else // defined NWK_PLL
  /* write only packet header data to FIFO */
  mrfiSpiWriteTxFifo(p, MRFI_PAYLOAD_OFFSET);
#endif
}



/**************************************************************************************************
 * @fn          MRFI_CompleteTxPrep
 *
 * @brief       Finalize setup for transmit
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      None
 **************************************************************************************************
 */
static void MRFI_CompleteTxPrep( mrfiPacket_t * pPacket )
{
#ifdef NWK_PLL
  uint8_t txBufLen;
  uint8_t *p;

  /* set point at beginning of outgoing frame */
  p = &pPacket->frame[MRFI_LENGTH_FIELD_OFS];

  /* get number of bytes in the packet (does not include the length byte) */
  txBufLen = *p;

  /* move pointer to remainder of frame */
  p = &pPacket->frame[MRFI_PAYLOAD_OFFSET + 1];

  if( sTxTimeStampAddr != NULL ) // if need to add time stamp to packet
    MRFI_GetLocalRawTime(sTxTimeStampAddr); // fill in the packet data

  /* write packet bytes to FIFO */
  mrfiSpiWriteTxFifo(p, txBufLen - MRFI_PAYLOAD_OFFSET);
#else
  (void) pPacket;
#endif
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
  return MRFI_TX_DONE_STATUS();
}


/**************************************************************************************************
 * @fn          MRFI_Transmit
 *
 * @brief       Transmit a packet.
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
  bspIState_t s;
  uint8_t txResult = MRFI_TX_RESULT_SUCCESS;
  static uint8_t dsn = 0;

  /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* TX_DONE line status must be low. If high, some state logic problem. */
  MRFI_ASSERT(!MRFI_TX_DONE_STATUS());


  /* Turn off reciever. We ignore/drop incoming packets during transmit. */
  Mrfi_RxModeOff();


  /* --------------------------------------
   *    Populate the IEEE fields in frame
   *   ------------------------------------
   */

  /* set the sequence number, also known as DSN (Data Sequence Number) */
  pPacket->frame[MRFI_DSN_OFFSET]   = dsn++;
  pPacket->frame[MRFI_FCF_OFFSET]   = MRFI_FCF_0_7;
  pPacket->frame[MRFI_FCF_OFFSET+1] = MRFI_FCF_8_15;

  MRFI_PrepareToTx( pPacket ); /* transfer the packet to the radio */

  /* Forced transmit */
  if(txType == MRFI_TX_TYPE_FORCED)
  {
    /* NOTE: Bug (#1) described in the errata swrz024.pdf for CC2520:
     * We never strobe TXON when the radio is in receive state.
     * If this is changed, must implement the bug workaround as described in the
     * errata (flush the Rx FIFO).
     */
#ifdef NWK_PLL
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      if( stx_active == false ) // if the channel was changed
      {
        BSP_EXIT_CRITICAL_SECTION(s);
        Mrfi_RxModeOff();            // turn off the radio
        MRFI_PrepareToTx( pPacket ); // setup transmission again
        continue; // restart the loop
      }
    MRFI_CompleteTxPrep( pPacket );
    } while( 0 );
#endif

    /* strobe transmit */
    mrfiSpiCmdStrobe(STXON);

#ifdef NWK_PLL
      BSP_EXIT_CRITICAL_SECTION(s);
#endif

    /* wait for transmit to complete */
    Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                        MRFI_MAX_TRANSMIT_TIME_us % 1000,
                        mrfi_TxDone );

    /* Clear the TX_FRM_DONE exception flag register in the radio. */
    mrfiSpiBitClear(EXCFLAG0, 1);
  }
  else /* CCA Transmit */
  {
    /* set number of CCA retries */
    uint8_t ccaRetries = MRFI_CCA_RETRIES;

    MRFI_ASSERT( txType == MRFI_TX_TYPE_CCA );

    /* ======================================================================
    *    CCA Algorithm Loop
    * ======================================================================
    */
    while(1)
    {
      /* Turn ON the receiver to perform CCA. Can not call Mrfi_RxModeOn(),
      * since that will enable the rx interrupt, which we do not want.
      */
      mrfiSpiCmdStrobe(SRXON);

      /* Wait for RSSI to be valid. */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

      BSP_ENTER_CRITICAL_SECTION(s);
#ifdef NWK_PLL
      if( stx_active == false ) // if the channel was changed
      {
        BSP_EXIT_CRITICAL_SECTION(s);
        Mrfi_RxModeOff();            // turn off the radio
        MRFI_PrepareToTx( pPacket ); // setup transmission again
        continue; // restart the cca loop
      }

      MRFI_CompleteTxPrep( pPacket );
#endif

      /* Request transmit on cca */
      mrfiSpiCmdStrobe(STXONCCA);
      BSP_EXIT_CRITICAL_SECTION(s);

      /* If sampled CCA is set, transmit has begun. */
      if(MRFI_SAMPLED_CCA())
      {
        /* wait for transmit to complete */
        Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                            MRFI_MAX_TRANSMIT_TIME_us % 1000,
                            mrfi_TxDone );

        /* Clear the TX_FRM_DONE exception flag register in the radio. */
        mrfiSpiBitClear(EXCFLAG0, 1);

        /* transmit is done. break out of CCA algorithm loop */
        break;
      }
      else
      {
        /* ------------------------------------------------------------------
         *    Clear Channel Assessment failed.
         * ------------------------------------------------------------------
         */

        /* Retry ? */
        if(ccaRetries != 0)
        {
          /* turn off reciever to conserve power during backoff */
          Mrfi_RxModeOff();

          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();

          /* decrement CCA retries before loop continues */
          ccaRetries--;

#ifdef NWK_PLL
          /* re-write the data to the fifo so that the time stamp can be
           * re-introduced to the packet at the latest possible moment.
           */
          MRFI_PrepareToTx( pPacket ); // setup transmission again
#endif
        }
        else  /* No CCA retries left, abort */
        {
          /* set return value for failed transmit and break */
          txResult = MRFI_TX_RESULT_FAILED;
          break;
        }
      }
    } /* End CCA Algorithm Loop */
  }

#ifdef NWK_PLL
  stx_active = false; // indicate we are in the act of transmitting

  // Packet transmitted, regardless of packet type, remove reference.
  sTxTimeStampAddr = NULL;
#endif

  /* turn radio back off to put it in a known state */
  Mrfi_RxModeOff();

  /* If the radio was in RX state when transmit was attempted,
   * put it back in RX state.
   */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }

  /* return the result of the transmit */
  return( txResult );
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
  /* copy last received packed into indicated memory */
  *pPacket = mrfiIncomingPacket;
}


/**************************************************************************************************
 * @fn          MRFI_RxOn
 *
 * @brief       Turn on the receiver.  No harm is done if this function
 *              is called when receiver is already on.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxOn(void)
{
  /* radio must be powered ON before we can move it to RX state */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Put the radio in RX state, if not already */
  if(mrfiRadioState != MRFI_RADIO_STATE_RX)
  {
    mrfiRadioState = MRFI_RADIO_STATE_RX;
    Mrfi_RxModeOn();
  }
}


/**************************************************************************************************
 * @fn          MRFI_RxIdle
 *
 * @brief       Put radio in idle mode (receiver is off).  No harm is done if
 *              this function is called when radio is already idle.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxIdle(void)
{
  /* radio must be powered ON to move it to idle mode */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* if radio is on, turn it off */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOff();
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  }
}


/**************************************************************************************************
 * @fn          MRFI_Sleep
 *
 * @brief       Request radio go to sleep (Power OFF the radio chip).
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Sleep(void)
{
  /* If radio is not asleep, put it to sleep */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* go to idle so radio is in a known state before sleeping */
    MRFI_RxIdle();

    /* turn off power to the radio */
    Mrfi_TurnOffRadioPower();

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;
  }
}


/**************************************************************************************************
 * @fn          MRFI_SetLogicalChannel
 *
 * @brief       Set logical channel.
 *
 * @param       chan - logical channel number
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetLogicalChannel(uint8_t chan)
{
  uint8_t phyChannel;

  /* logical channel is not valid */
  MRFI_ASSERT( chan < MRFI_NUM_LOGICAL_CHANS );

  /* make sure radio is off before changing channels */
  Mrfi_RxModeOff();

  /* convert logical channel number into physical channel number */
  phyChannel = mrfiLogicalChanTable[chan];

  /* write frequency value of new channel */
#ifndef FREQUENCY_HOPPING
  mrfiSpiWriteReg(FREQCTRL, (FREQCTRL_BASE_VALUE + (FREQCTRL_FREQ_2405MHZ + 5 * ((phyChannel) - 11))));

  /* remember this. need it when waking up. */
  mrfiCurrentLogicalChannel = chan;
#else
  /* frequency hopping requires more channels than are available using 802.15.4
   * so don't do any calculations, just jam the MHz offset from 2400MHz into the
   * control register
   */
  //mrfiSpiWriteReg(FREQCTRL, FREQCTRL_BASE_VALUE + phyChannel);
  mrfiSpiWriteReg(FREQCTRL, (FREQCTRL_BASE_VALUE + (FREQCTRL_FREQ_2405MHZ + 5 * ((phyChannel) - 11))));
#endif

  /* Put the radio back in RX state, if it was in RX before channel change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
}


/**************************************************************************************************
 * @fn          MRFI_SetRFPwr
 *
 * @brief       Set RF power level.
 *
 * @param       idx - index into power level table
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetRFPwr(uint8_t idx)
{
  /* is power level specified valid? */
  MRFI_ASSERT( idx < MRFI_NUM_POWER_SETTINGS );

  /* make sure radio is off before changing power level */
  Mrfi_RxModeOff();

  /* write value of new power level */
  mrfiSpiWriteReg(TXPOWER, mrfiRFPowerTable[idx]);

  /* remember this. need it when waking up. */
  mrfiCurrentPowerLevel = idx;

  /* Put the radio back in RX state, if it was in RX before change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
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
   * a certain duration.
   */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
  MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

  /* Read and convert RSSI to decimal and do offset compensation. */
  return( Mrfi_CalculateRssi(mrfiSpiReadReg(RSSI)) );
}


/**************************************************************************************************
 * @fn          MRFI_RandomByte
 *
 * @brief       Returns a random byte. This is a pseudo-random number generator.
 *              The generated sequence will repeat every 256 values.
 *              The sequence itself depends on the initial seed value.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8_t MRFI_RandomByte(void)
{
  mrfiRndSeed = (mrfiRndSeed*MRFI_RANDOM_MULTIPLIER) + MRFI_RANDOM_OFFSET;

  return mrfiRndSeed;
}

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
 * @fn          MRFI_DelayMs
 *
 * @brief       Delay the specified number of milliseconds.
 *
 * @param       milliseconds - delay time
 *
 * @return      none
 **************************************************************************************************
 */
#ifndef MRFI_TIMER_ALWAYS_ACTIVE
void MRFI_DelayMs(uint16_t milliseconds)
{
  while(milliseconds)
  {
    Mrfi_DelayUsec( APP_USEC_VALUE );
    milliseconds--;
  }
}
#endif // MRFI_TIMER_ALWAYS_ACTIVE

/**************************************************************************************************
 * @fn          MRFI_ReplyDelay
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
#ifndef MRFI_TIMER_ALWAYS_ACTIVE
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
#endif // MRFI_TIMER_ALWAYS_ACTIVE


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


/**************************************************************************************************
 * @fn          Mrfi_TurnOnRadioPower
 *
 * @brief       Power ON the radio chip.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_TurnOnRadioPower(void)
{
  /* put radio chip into reset */
  MRFI_DRIVE_RESETN_PIN_LOW();

  /* enable the voltage regulator */
  MRFI_DRIVE_VREG_EN_PIN_HIGH();

  /* wait for the chip to power up */
  Mrfi_DelayUsec(MRFI_VREG_SETTLE_TIME_USECS);

  /* release from reset */
  MRFI_DRIVE_RESETN_PIN_HIGH();

  /* wait for the radio crystal oscillator to stabilize */
  MRFI_SPI_SET_CHIP_SELECT_ON();
  while (!MRFI_SPI_SO_IS_HIGH());
  MRFI_SPI_SET_CHIP_SELECT_OFF();
}


/**************************************************************************************************
 * @fn          Mrfi_TurnOffRadioPower
 *
 * @brief       Power OFF the radio chip.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_TurnOffRadioPower(void)
{
  /* put chip into reset and then turn off voltage regulator */
  MRFI_DRIVE_RESETN_PIN_LOW();
  MRFI_DRIVE_VREG_EN_PIN_LOW();
}


/**************************************************************************************************
 * @fn          Mrfi_RxModeOff
 *
 * @brief       Disable frame receiving.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOff(void)
{
  /* NOTE: Bug (#1) described in the errata swrz024.pdf for CC2520:
   * The sequence of sending the RFOFF strobe takes care of the bug.
   * If this is changed, ensure that the bug workaround is in place.
   */

  /*disable receive interrupts */
  MRFI_DISABLE_RX_INTERRUPT();

  /* turn off radio */
  mrfiSpiCmdStrobe(SRFOFF);

  /* flush the receive FIFO of any residual data */
  MRFI_RADIO_FLUSH_RX_BUFFER();

  /* clear receive interrupt */
  MRFI_CLEAR_RX_INTERRUPT_FLAG();

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
}


/**************************************************************************************************
 * @fn          Mrfi_RxModeOn
 *
 * @brief       Enable frame receiving.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOn(void)
{
  /* NOTE: Bug #1 described in the errata swrz024.pdf for CC2520:
   * This function is never called if the radio is already in receive state.
   * If this is changed, must implement the bug workaround as described in the
   * errata (flush the Rx FIFO).
   */

  /* clear any residual receive interrupt */
  MRFI_CLEAR_RX_INTERRUPT_FLAG();

  /* send strobe to enter receive mode */
  mrfiSpiCmdStrobe(SRXON);

  /* enable receive interrupts */
  MRFI_ENABLE_RX_INTERRUPT();

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
}


/**************************************************************************************************
 * @fn          Mrfi_FiFoPIsr
 *
 * @brief       Interrupt Service Routine for handling FIFO_P event.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void Mrfi_FiFoPIsr(void)
{
  uint8_t numBytes;
  uint8_t i;

#ifdef NWK_PLL
  /*  If the network pll is running then we need to acquire
   *  the receive time stamp (at least as close to the actual
   *  time that the receive occurred)
   */
  if( sRxTimeStampAddr != NULL )
    MRFI_GetLocalRawTime( sRxTimeStampAddr );

#endif

  /* NOTE: Bug #2 described in the errata swrz024.pdf for CC2520:
   * There is a possiblity of small glitch in the fifo_p signal
   * (2 cycle of 32 MHz). Workaround is to make sure that fifo_p signal stays
   * high for longer than that. Else, it is a false alarm.
   */
  if(!MRFI_FIFOP_STATUS()) return;
  if(!MRFI_FIFOP_STATUS()) return;

  /* Ah... it is for real... Continue processing. */

  /* We should receive this interrupt only in RX state
   * Should never receive it if RX was turned ON only for
   * some internal mrfi processing like - during CCA.
   * Otherwise something is terribly wrong.
   */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  do
  {
    /*
     * Pend on frame rx completion. First time through this always passes.
     * Later, though, it is possible that the Rx FIFO has bytes but we
     * havn't received a complete frame yet.
     */
    while( !MRFI_FIFOP_STATUS() );

    /* Check for Rx overflow. Checking here means we may flush a valid frame */
    if( MRFI_FIFOP_STATUS() && !MRFI_FIFO_STATUS() )
    {
      /* Flush receive FIFO to recover from overflow */
      MRFI_RADIO_FLUSH_RX_BUFFER();
      break;
    }

    /* clear interrupt flag so we can detect another frame later. */
    MRFI_CLEAR_RX_INTERRUPT_FLAG();

    /*
     *  Determine number of bytes to be read from receive FIFO.  The first byte
     *  has the number of bytes in the packet.  A mask must be applied though
     *  to strip off unused bits.  The number of bytes in the packet does not
     *  include the length byte itself but does include the FCS (generically known
     *  as RX metrics).
     */
    mrfiSpiReadRxFifo(&numBytes, 1);
    numBytes &= IEEE_PHY_PACKET_SIZE_MASK;

    /* see if frame will fit in maximum available buffer or is too small */
    if (((numBytes + MRFI_LENGTH_FIELD_SIZE - MRFI_RX_METRICS_SIZE) > MRFI_MAX_FRAME_SIZE) ||
         (numBytes < MRFI_MIN_SMPL_FRAME_SIZE))
    {
      uint8_t dummy;
      /* packet is too big or too small. remove it from FIFO */
      for (i=0; i<numBytes; i++)
      {
        /* read and discard bytes from FIFO */
        mrfiSpiReadRxFifo(&dummy, 1);
      }
    }
    else
    {
      uint8_t *p, nextByte;

      /* Clear out my buffer to remove leftovers in case a bogus packet gets through */
      for(i=0; i < MRFI_MAX_FRAME_SIZE; i++)
      {
        mrfiIncomingPacket.frame[i] = 0;
      }

      /* set pointer at first byte of frame storage */
      p  = &mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFS];

      /*
       *  Store frame length into the incoming packet memory.  Size of rx metrics
       *  is subtracted to get the MRFI frame length which separates rx metrics from
       *  the frame length.
       */
      *p = numBytes - MRFI_RX_METRICS_SIZE;

      /* read frame bytes from rx FIFO and store into incoming packet memory */
      p++;
      mrfiSpiReadRxFifo(p, numBytes-MRFI_RX_METRICS_SIZE);

      /* The next two bytes in the rx fifo are:
       * - RSSI of the received frams
       * - CRC OK bit and the 7 bit wide correlation value.
       * Read this rx metrics and store to incoming packet.
       */

      /* Add the RSSI offset to get the proper RSSI value. */
      mrfiSpiReadRxFifo(&nextByte, 1);
      mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_RSSI_OFS] = Mrfi_CalculateRssi(nextByte);

      /* The second byte has 7 bits of Correlation value and 1 bit of
       * CRC pass/fail info. Remove the CRC pass/fail bit info.
       * Also note that for CC2520 radio this is the correlation value and not
       * the LQI value. Some convertion is needed to extract the LQI value.
       * This convertion is left to the application at this time.
       */
      mrfiSpiReadRxFifo(&nextByte, 1);
      mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] = nextByte & MRFI_RX_METRICS_LQI_MASK;

      MRFI_DISABLE_RX_INTERRUPT();  // disable radio sync interrupt so no more occur
      rx_isr_context = true;        // deactivate sync pin interrupt enable/disable macros
      BSP_ENABLE_INTERRUPTS( );     // enable interrupts so higher priority irqs can occur
        
      /* Eliminate frames that are the correct size but we can tell are bogus
       * by their frame control fields OR if CRC failed.
       */
      if( (nextByte & MRFI_RX_METRICS_CRC_OK_MASK) &&
          (mrfiIncomingPacket.frame[MRFI_FCF_OFFSET] == MRFI_FCF_0_7) &&
          (mrfiIncomingPacket.frame[MRFI_FCF_OFFSET+1] == MRFI_FCF_8_15))
      {
        /* call external, higher level "receive complete" (CRC checking is done by hardware) */
        MRFI_RxCompleteISR();
      }

      BSP_DISABLE_INTERRUPTS( );    // disable interrupts so we can enable radio sync interrupt again
      rx_isr_context = false;       // activate sync pin interrupt enable/disable macros
      MRFI_ENABLE_RX_INTERRUPT( );  // enable radio sync interrupt again
     }

    /* If the client code takes long time to process the frame,
     * rx fifo could overflow during this time. As soon as this condition is
     * reached, the radio fsm stops all activities till the rx fifo is flushed.
     * It also puts the fifo signal low. When we come out of this while loop,
     * we really don't know if it is because of overflow condition or
     * there is no data in the fifo. So we must check for overflow condition
     * before exiting the ISR otherwise it could get stuck in this "overflow"
     * state forever.
     */

  } while( MRFI_FIFO_STATUS() ); /* Continue as long as there is some data in FIFO */

  /* Check if we exited the loop due to fifo overflow.
   * and not due to no data in fifo.
   */
  if( MRFI_FIFOP_STATUS() && !MRFI_FIFO_STATUS() )
  {
    /* Flush receive FIFO to recover from overflow */
    MRFI_RADIO_FLUSH_RX_BUFFER();
  }
}


/**************************************************************************************************
 * @fn          Mrfi_CalculateRssi
 *
 * @brief       Does binary to decimal convertion and offset compensation.
 *
 * @param       raw rssi value
 *
 * @return      RSSI value in units of dBm.
 **************************************************************************************************
 */
int8_t Mrfi_CalculateRssi(uint8_t rawValue)
{
  int16_t rssi;

  /* The raw value is in 2's complement and in 1 dB steps. Convert it to
   * decimal taking into account the offset value.
   */
  if(rawValue >= 128)
  {
    rssi = (int16_t)(rawValue - 256) - MRFI_RSSI_OFFSET;
  }
  else
  {
    rssi = rawValue - MRFI_RSSI_OFFSET;
  }

  /* Restrict this value to least value can be held in an 8 bit signed int */
  if(rssi < -128)
  {
    rssi = -128;
  }

  return rssi;
}


/**************************************************************************************************
 * @fn          Mrfi_RandomBackoffDelay
 *
 * @brief       Waits for random amount of time before returning.
 *              The range is: 1*250 to 16*250 us.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RandomBackoffDelay(void)
{
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    Mrfi_DelayUsecLong( (MRFI_RandomByte() & 0x0F) + 1, 0, NULL );
#else
  uint8_t backoffs;
  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (MRFI_RandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  for (i=0; i<backoffs; i++)
    Mrfi_DelayUsec( MRFI_BACKOFF_PERIOD_USECS );
#endif
}


/**************************************************************************************************
 * @fn          mrfi_IdleTestTimeOut
 *
 * @brief       Indicates status of transmission completion
 *
 * @param       None
 *
 * @return      true if transmission is complete, false otherwise
 **************************************************************************************************
 */
bool mrfi_IdleTestTimeOut( void )
{
  return MRFI_TX_DONE_STATUS() && !MRFI_FIFO_STATUS();
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
    Mrfi_DelayUsecLong( ( MRFI_MAX_TRANSMIT_TIME_us * 2 ) / 1000,
                        ( MRFI_MAX_TRANSMIT_TIME_us * 2 ) % 1000,
                        mrfi_IdleTestTimeOut );

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
 * @param       none
 *
 * @return      true if RSSI indicates as valid
 **************************************************************************************************
 */
static bool Mrfi_ValidateRSSI( void )
{
  return mrfiSpiCmdStrobe(SNOP) & RSSI_VALID;
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
  t.state.lsb.phy_offset = MRFI_TIMER_SZ;

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
    // if we are not in the middle of a communciation we can manage the FHSS schedule
    // otherwise we don't want to allow any channel changes during communication to the
    // radio or we will mess with any currently ongoing communciation.  It is assumed
    // any delay during communcation here will be small and so should not impact FHSS
    // significantly
    if( sActiveSPI == false )
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
    us = tmp >> 8;
  }


  time.formatted.tmr += us; // add in final timer counts
  if( time.formatted.tmr < us ) // if an overflow occurred
    ms++; // account for it

  // adjust for any rollover values ( should only happen once but possibly more
  while( time.formatted.tmr >= BSP_ROLLOVER_LIMIT )
  {
    time.formatted.tmr -= BSP_ROLLOVER_LIMIT; // remove this millisecond
    ms++; // account for it
  }

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
        // if we are not in the middle of a communciation we can manage the FHSS schedule
        // otherwise we don't want to allow any channel changes during communication to the
        // radio or we will mess with any currently ongoing communciation.  It is assumed
        // any delay during communcation here will be small and so should not impact FHSS
        // significantly
        if( sActiveSPI == false )
          // manage the FHSS schedule, only allow pumps if there is at least 5
          // milliseconds of time delay remaining
          FHSS_ACTIVE( nwk_pllBackgrounder( stx_active != false
                                            || time.formatted.time - ms < 5 ) );
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


/****************************************************************************************************
 * @fn          Mrfi_DelayUsecSem -- Frequency Hopping Enabled
 *
 * @brief       Continuously test the semaphore
 *              Delay specified number of microseconds checking semaphore for
 *              early-out.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
static void Mrfi_DelayUsecSem(uint16_t howLong)
{
  Mrfi_DelayUsecLong( 0, howLong, Mrfi_CheckSem ); // delay with semaphore dependency
  return; // time out complete
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

#else // MRFI_TIMER_ALWAYS_ACTIVE


/****************************************************************************************************
 * @fn          Mrfi_DelayUsec
 *
 * @brief       Execute a delay loop using the MAC timer. The macro actually used to do the delay
 *              is not reentrant. This routine makes the delay execution reentrant by breaking up
 *              the requested delay up into small chunks and executing each chunk as a critical
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
static void Mrfi_DelayUsec(uint16_t howLong)
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
 * @fn          Mrfi_DelayUsecSem
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
BSP_ISR_FUNCTION(Mrfi_TimerISR, BSP_TIMER_VECTOR)
{
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

  return;
}

#endif // MRFI_TIMER_ALWAYS_ACTIVE

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */


#define MRFI_RADIO_TX_FIFO_SIZE     128  /* from datasheet */

/* verify largest possible packet fits within FIFO buffer */
#if ((MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE) > MRFI_RADIO_TX_FIFO_SIZE)
#error "ERROR:  Maximum possible packet length exceeds FIFO buffer.  Decrease value of maximum application payload."
#endif

/**************************************************************************************************
*/
