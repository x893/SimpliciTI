/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2008 Texas Instruments Incorporated.  All rights reserved.

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
 *   Radios: CC2430
 *   Primary code file for supported radios.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <string.h>
#include MCU_H
#include "mrfi.h"
#include "bsp.h"
#include "bsp_macros.h"
#include "bsp_external/mrfi_board_defs.h"
#include "nwk_pll.h"
#include "bsp_leds.h"


/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */

#define MRFI_RSSI_OFFSET    -45   /* no units */

/*
 *  For RSSI to be valid, we must wait for 20 symbol periods:
 *  - 12 symbols to go from idle to rx state
 *  -  8 symbols to calculate the RSSI value
 *  - 10 symbols to add robustness which appears to be necessary
 */
#define MRFI_RSSI_VALID_DELAY_US (30 * IEEE_USECS_PER_SYMBOL)


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Values imported from mrfi_defs.h
 *   - - - - - - - - - - - - - - - - - -
 */
#define MRFI_DSN_OFFSET             __mrfi_DSN_OFS__
#define MRFI_FCF_OFFSET             __mrfi_FCF_OFS__

#define MRFI_FCF_0_7              (0x01)
#define MRFI_FCF_8_15             (0x88)
#define MRFI_MIN_SMPL_FRAME_SIZE  (MRFI_HEADER_SIZE + NWK_HDR_SIZE)

#ifdef MRFI_TIMER_ALWAYS_ACTIVE

// re-map static functions promoted to public for backwards compatibility
#define Mrfi_DelayUsec( a ) MRFI_DelayUsec( a )
#define Mrfi_DelayMs( a ) MRFI_DelayMs( a )

#endif // MRFI_TIMER_ALWAYS_ACTIVE

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Radio Definitions
 *   - - - - - - - - - -
 */

/* immediate strobe processor command instructions */
#define ISTXCALN      0xE1
#define ISRXON        0xE2
#define ISTXON        0xE3
#define ISTXONCCA     0xE4
#define ISRFOFF       0xE5
#define ISFLUSHRX     0xE6
#define ISFLUSHTX     0xE7
#define ISACK         0xE8
#define ISACKPEND     0xE9

#if defined(MRFI_CC2430)
#define MRFI_RADIO_PARTNUM        0x85
#define MRFI_RADIO_MIN_VERSION    3      /* minimum version is Rev D */
#elif defined(MRFI_CC2431)
#define MRFI_RADIO_PARTNUM        0x89
#define MRFI_RADIO_MIN_VERSION    4
#else
#error ERROR: No part number or radio version defined for radio
#endif

/* FSCTRLL */
#define FREQ_2405MHZ    0x65

/* MDMCTRL0H */
#define ADDR_DECODE     BV(3)

/* RFSTATUS */
#define TX_ACTIVE       BV(4)
#define FIFO            BV(3)
#define FIFOP           BV(2)
#define SFD             BV(1)
#define CCA             BV(0)

/* RFIF */
#define IRQ_TXDONE      BV(6)

/* RFIM */
#define IM_TXDONE       BV(6)

/* IEN2 */
#define RFIE            BV(0)

/* SLEEP */
#define XOSC_STB        BV(6)
#define OSC_PD          BV(2)

/* CLKCON */
#define OSC32K          BV(7)
#define OSC             BV(6)

/* RFPWR */
#define ADI_RADIO_PD    BV(4)
#define RREG_RADIO_PD   BV(3)

/* RFIF */
#define IRQ_TXDONE      BV(6)
#define IRQ_FIFOP       BV(5)
#define IRQ_SFD         BV(4)

/* RFIM */
#define IM_TXDONE       BV(6)
#define IM_FIFOP        BV(5)
#define IM_SFD          BV(4)

/* MDMCTRL1L */
#define MDMCTRL1L_RESET_VALUE         0x00
#define RX_MODE(x)                    ((x) << 0)
#define RX_MODE_INFINITE_RECEPTION    RX_MODE(2)
#define RX_MODE_NORMAL_OPERATION      RX_MODE(0)

/* FSMSTATE */
#define FSM_FFCTRL_STATE_RX_INF       31      /* infinite reception state - not documented in datasheet */


/* ADCCON1 */
#define RCTRL1                        BV(3)
#define RCTRL0                        BV(2)
#define RCTRL_BITS                    (RCTRL1 | RCTRL0)
#define RCTRL_CLOCK_LFSR              RCTRL0


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *     IEEE 802.15.4 definitions
 *   - - - - - - - - - - - - - - -
 */
#define IEEE_PHY_PACKET_SIZE_MASK   0x7F
#define IEEE_USECS_PER_SYMBOL       16

/* -------------------------------------------------------------------  */
/*  TBD these need to move to board def file */

/* 32 kHz clock source select in CLKCON */
#if !defined (OSC32K_CRYSTAL_INSTALLED) || (defined (OSC32K_CRYSTAL_INSTALLED) && (OSC32K_CRYSTAL_INSTALLED == TRUE))
#define OSC_32KHZ  0x00 /* external 32 KHz xosc */
#else
#define OSC_32KHZ  0x80 /* internal 32 KHz rcosc */
#endif

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls
   * to the microsecond routine which can be calibrated independently.
   */
#if defined(SW_TIMER)
#define APP_USEC_VALUE    250
#else
#define APP_USEC_VALUE    1500
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

#define MRFI_SYNC_WORD_SIZE 1
#define MRFI_MDMCTRL0L_DFLT 0xE2
#define MRFI_PREAMBLE_SIZE  ( ( MRFI_MDMCTRL0L_DFLT & 0xF ) + 1 )
#define MRFI_CRC_SIZE       ( ( MRFI_MDMCTRL0L_DFLT >> 4 ) & 2 )
#define MRFI_MAX_TRANSMIT_BYTES ( MRFI_MAX_FRAME_SIZE + MRFI_SYNC_WORD_SIZE + MRFI_PREAMBLE_SIZE + MRFI_CRC_SIZE )
#define MRFI_TRANSMIT_BIT_PERIOD_us 4.0
#define MRFI_MAX_TRANSMIT_TIME_us ((unsigned long)( MRFI_TRANSMIT_BIT_PERIOD_us * 8 * MRFI_MAX_TRANSMIT_BYTES + 500))

 /* Flush must be done twice - datasheet. */
#define MRFI_RADIO_FLUSH_RX_BUFFER() {RFST = ISFLUSHRX; RFST = ISFLUSHRX;}

/* ------------------------------------------------------------------------------------------------
 *                                    Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void MRFI_PrepareToTx( mrfiPacket_t * );
static void MRFI_CompleteTxPrep( mrfiPacket_t * );
static bool mrfi_TxDone( void );

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
static bool Mrfi_CheckSem( void );
static bool Mrfi_ValidateRSSI( void );
#else
static void Mrfi_DelayUsecSem(uint16_t);
static void Mrfi_DelayUsec(uint16_t);
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
#if !defined(MRFI_PA_LNA_ENABLED)
static const uint8_t mrfiRFPowerTable[] =
{
  0x06,
  0x0B,
  0x7F
};
#else  /* !MRFI_PA_LNA_ENABLED */
/* If MRFI_PA_LNA_ENABLED use 0 dBm, 13 dBm, and 19 dBm instead. */
static const uint8_t mrfiRFPowerTable[] =
{
  0x06,
  0x13,
  0xFF
};
#endif

/* verify number of table entries matches the corresponding #define */
BSP_STATIC_ASSERT(__mrfi_NUM_POWER_SETTINGS__ == ((sizeof(mrfiRFPowerTable)/sizeof(mrfiRFPowerTable[0])) * sizeof(mrfiRFPowerTable[0])));

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8_t mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;
static mrfiPacket_t mrfiIncomingPacket;

/* reply delay support */
static volatile uint8_t  sReplyDelayContext = 0;
static volatile uint8_t  sKillSem = 0;
static          uint16_t sReplyDelayScalar = 0;

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
  MRFI_ASSERT( CHIPID == MRFI_RADIO_PARTNUM );      /* wrong radio */
  MRFI_ASSERT( CHVER  >= MRFI_RADIO_MIN_VERSION );  /* obsolete radio version */

  /* ------------------------------------------------------------------
   *    Configure IO ports
   *   ---------------------------
   */

#if defined(MRFI_PA_LNA_ENABLED) && defined(BSP_BOARD_SRF04EB)
  MRFI_BOARD_PA_LNA_CONFIG_PORTS();
  MRFI_BOARD_PA_LNA_HGM();
#endif


  /* ------------------------------------------------------------------
   *    Configure clock to use XOSC
   *   -----------------------------
   */
  SLEEP &= ~OSC_PD;                       /* turn on 16MHz RC and 32MHz XOSC */
  while (!(SLEEP & XOSC_STB));            /* wait for 32MHz XOSC stable */
  asm("NOP");                             /* chip bug workaround */
  {
    uint16_t i;

    /* Require 63us delay for all revs */
    for (i=0; i<504; i++)
    {
      asm("NOP");
    }
  }
  CLKCON = (0x00 | OSC_32KHZ);            /* 32MHz XOSC */
  while (CLKCON != (0x00 | OSC_32KHZ));
  SLEEP |= OSC_PD;                        /* turn off 16MHz RC */


  /* ------------------------------------------------------------------
   *    Variable Initialization
   *   -------------------------
   */

#ifdef MRFI_ASSERTS_ARE_ON
  PANIDL = 0xFF;
  PANIDH = 0xFF;
#endif


  /* ------------------------------------------------------------------
   *    Initialize Random Seed Value
   *   -------------------------------
   */

  /* turn on radio power, pend for the power-up delay */
  RFPWR &= ~RREG_RADIO_PD;
  while((RFPWR & ADI_RADIO_PD));

  /*
   *  Set radio for infinite reception.  Once radio reaches this state,
   *  it will stay in receive mode regardless RF activity.
   */
  MDMCTRL1L = MDMCTRL1L_RESET_VALUE | RX_MODE_INFINITE_RECEPTION;

  /* turn on the receiver */
  RFST = ISRXON;

  /*
   *  Wait for radio to reach infinite reception state.  Once it does,
   *  The least significant bit of ADTSTH should be pretty random.
   */
  while (FSMSTATE != FSM_FFCTRL_STATE_RX_INF)

  /* put 16 random bits into the seed value */
  {
    uint16_t rndSeed;
    uint8_t  i;

    rndSeed = 0;

    for(i=0; i<16; i++)
    {
      /* use most random bit of analog to digital receive conversion to populate the random seed */
      rndSeed = (rndSeed << 1) | (ADCTSTH & 0x01);
    }

    /*
     *  The seed value must not be zero.  If it is, the pseudo random sequence will be always be zero.
     *  There is an extremely small chance this seed could randomly be zero (more likely some type of
     *  hardware problem would cause this).  To solve this, a single bit is forced to be one.  This
     *  slightly reduces the randomness but guarantees a good seed value.
     */
    rndSeed |= 0x0080;

    /*
     *  Two writes to RNDL will set the random seed.  A write to RNDL copies current contents
     *  of RNDL to RNDH before writing new the value to RNDL.
     */
    RNDL = rndSeed & 0xFF;
    RNDL = rndSeed >> 8;
  }

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
  
  /* turn off the receiver, flush RX FIFO just in case something got in there */
  RFST = ISRFOFF;

  /* flush the rx buffer */
  MRFI_RADIO_FLUSH_RX_BUFFER();

  /* take receiver out of infinite reception mode; set back to normal operation */
  MDMCTRL1L = MDMCTRL1L_RESET_VALUE | RX_MODE_NORMAL_OPERATION;

  /* turn radio back off */
  RFPWR |= RREG_RADIO_PD;

  /* Initial radio state is OFF state */
  mrfiRadioState = MRFI_RADIO_STATE_OFF;
  /* ------------------------------------------------------------------
   *    Configure Radio Registers
   *   ---------------------------
   */

  /* tuning adjustments for optimal radio performance; details available in datasheet */
  RXCTRL0H = 0x32;
  RXCTRL0L = 0xF5;

  /* disable address filtering */
  MDMCTRL0H &= ~ADDR_DECODE;

  /* set FIFOP threshold to maximum */
  IOCFG0 = 127;

  /* set default channel */
  MRFI_SetLogicalChannel( 0 );

  /* set default power */
  MRFI_SetRFPwr(MRFI_NUM_POWER_SETTINGS - 1);

  /* enable general RF interrupts */
  IEN2 |= RFIE;


  /* ------------------------------------------------------------------
   *    Final Initialization
   *   -----------------------
   */


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

#define   PHY_PREAMBLE_SYNC_BYTES    8

  {
    uint32_t bits, dataRate = 250000;

    bits = ((uint32_t)((PHY_PREAMBLE_SYNC_BYTES + MRFI_MAX_FRAME_SIZE)*8))*10000;

    /* processing on the peer + the Tx/Rx time plus more */
    sReplyDelayScalar = PLATFORM_FACTOR_CONSTANT + (((bits/dataRate)+5)/10);
  }

  /* enable global interrupts */
  BSP_ENABLE_INTERRUPTS();

  /*
   *  Random delay - This prevents devices on the same power source from repeated
   *  transmit collisions on power up.
   */
  Mrfi_RandomBackoffDelay();
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
  uint8_t * p;
  uint8_t i;
  uint8_t txBufLen;
 
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = true; // indicate we are in the act of transmitting
#endif // MRFI_TIMER_ALWAYS_ACTIVE

  /* flush FIFO of any previous transmit that did not go out */
  RFST = ISFLUSHTX;

  /* set point at beginning of outgoing frame */
  p = &pPacket->frame[MRFI_LENGTH_FIELD_OFS];
  
  /* get number of bytes in the packet (does not include the length byte) */
  txBufLen = *p;

  /*
   *  Write the length byte to the FIFO.  This length does *not* include the length field
   *  itself but does include the size of the FCS (generically known as RX metrics) which
   *  is generated automatically by the radio.
   */
  RFD = txBufLen + MRFI_RX_METRICS_SIZE;

#ifndef NWK_PLL
  /* write packet bytes to FIFO */
  for (i=0; i<txBufLen; i++)
  {
    p++;
    RFD = *p;
  }
#else // defined NWK_PLL
  /* write only packet header data to fifo */
  for (i=0; i<MRFI_PAYLOAD_OFFSET; i++)
  {
    p++;
    RFD = *p;
  }
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
  uint8_t * p;
  uint8_t i;
  uint8_t txBufLen;

  /* set point at beginning of outgoing frame */
  p = &pPacket->frame[MRFI_LENGTH_FIELD_OFS];
  
  /* get number of bytes in the packet (does not include the length byte) */
  txBufLen = *p;

  /* move pointer to remainder of frame */
  p = &pPacket->frame[MRFI_PAYLOAD_OFFSET];

  if( sTxTimeStampAddr != NULL ) // if need to add time stamp to packet
    MRFI_GetLocalRawTime(sTxTimeStampAddr); // fill in the packet data

  /* write remaining bytes to fifo */
  for (i = MRFI_PAYLOAD_OFFSET; i<txBufLen; i++)
  {
    p++;
    RFD = *p;
  }
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
  return (RFIF & IRQ_TXDONE) != 0;
}



/**************************************************************************************************
 * @fn          MRFI_Transmit
 *
 * @brief       Transmit a packet using CCA algorithm.
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      Return code indicates success or failure of transmit:
 *                  MRFI_TX_RESULT_SUCCESS - transmit succeeded
 *                  MRFI_TX_RESULT_FAILED  - transmit failed because CCA failed
 **************************************************************************************************
 */
uint8_t test = 0;
uint8_t MRFI_Transmit(mrfiPacket_t * pPacket, uint8_t txType)
{
  static uint8_t dsn = 0;
  uint8_t txResult = MRFI_TX_RESULT_SUCCESS;

  /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );


  /* ------------------------------------------------------------------
   *    Initialize hardware for transmit
   *   -----------------------------------
   */

  /* turn off reciever */
  Mrfi_RxModeOff();

  /* clear 'transmit done' interrupt flag, this bit is tested to see when transmit completes */
  RFIF &= ~IRQ_TXDONE;


  /* ------------------------------------------------------------------
   *    Populate the IEEE fields in frame
   *   ------------------------------------
   */

  /* set the sequence number, also known as DSN (Data Sequence Number) */
  pPacket->frame[MRFI_DSN_OFFSET] = dsn;

  /* increment the sequence number, value is retained (static variable) for use in next transmit */
  dsn++;

  /*
   *  Populate the FCF (Frame Control Field) with the following settings.
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
  pPacket->frame[MRFI_FCF_OFFSET]   = MRFI_FCF_0_7;
  pPacket->frame[MRFI_FCF_OFFSET+1] = MRFI_FCF_8_15;


  /* ------------------------------------------------------------------
   *    Write packet to transmit FIFO
   *   --------------------------------
   */
  MRFI_PrepareToTx( pPacket );

  /* ------------------------------------------------------------------
   *    Immediate transmit
   *   ---------------------
   */
  if (txType == MRFI_TX_TYPE_FORCED)
  {
#ifdef NWK_PLL
    bspIState_t s;

    /* if sTxTimeStampAddr is not null then the network application has
     * filled it in because it is a PLL packet that requires the transmit
     * time.  After filling in the time stamp at the address specified,
     * delete the reference so future packets will not have a time stamp
     * acquisition.
     */
    BSP_ENTER_CRITICAL_SECTION(s);
    MRFI_CompleteTxPrep( pPacket );
    sTxTimeStampAddr = NULL; // clear the reference
#endif

    /* strobe transmit */
    RFST = ISTXON;

#ifdef NWK_PLL
    BSP_EXIT_CRITICAL_SECTION(s);
#endif

    /* wait for transmit to complete */
    Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                        MRFI_MAX_TRANSMIT_TIME_us % 1000,
                        mrfi_TxDone );

    /* transmit is done */
  }
  else
  {
    /* ------------------------------------------------------------------
     *    CCA transmit
     *   ---------------
     */
    MRFI_ASSERT( txType == MRFI_TX_TYPE_CCA );

    {
      bspIState_t s;
      uint8_t txActive;
      uint8_t ccaRetries;

      /* set number of CCA retries */
      ccaRetries = MRFI_CCA_RETRIES;


      /* ===============================================================================
       *    CCA Algorithm Loop
       *   ====================
       */
      for (;;)
      {
        /* Turn ON the receiver to perform CCA. Can not call Mrfi_RxModeOn(),
         * since that will enable the rx interrupt, which we do not want.
         */
        RFST = ISRXON;

        /*
         *  Wait for CCA to be valid.
         */
        Mrfi_DelayUsec( MRFI_RSSI_VALID_DELAY_US );

#ifdef NWK_PLL
        if( stx_active == false ) // if the channel was changed
        {
          Mrfi_RxModeOff();            // turn off the radio
          MRFI_PrepareToTx( pPacket ); // setup transmission again
          continue; // restart the cca loop
        }

        /* if sTxTimeStampAddr is not null then the network application has
         * filled it in because it is a PLL packet that requires the transmit
         * time.  After filling in the time stamp at the address specified,
         * delete the reference so future packets will not have a time stamp
         * acquisition.
         */
        BSP_ENTER_CRITICAL_SECTION(s);
        MRFI_CompleteTxPrep( pPacket );

#endif

        /*
         *  Initiate transmit with CCA.  Command is strobed and then status is
         *  immediately checked.  If status shows transmit is active, this means
         *  that CCA passed and the transmit has gone out.  A critical section
         *  guarantees timing status check happens immediately after strobe.
         */
#ifndef NWK_PLL
        BSP_ENTER_CRITICAL_SECTION(s);
#endif
        RFST = ISTXONCCA;
        txActive = RFSTATUS & TX_ACTIVE;
        BSP_EXIT_CRITICAL_SECTION(s);

        /* see transmit went out */
        if (txActive)
        {
          /* ----------|  CCA Passed |---------- */

          /* wait for transmit to complete */
          Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                              MRFI_MAX_TRANSMIT_TIME_us % 1000,
                              mrfi_TxDone );

#ifdef NWK_PLL
          // Packet transmitted, regardless of packet type, remove reference.
          sTxTimeStampAddr = NULL;
#endif

          /* transmit is done. break out of CCA algorithm loop */
          break;
        }
        else
        {
          /* ----------|  CCA Failed |---------- */

          /* if no CCA retries are left, transmit failed so abort */
          if (ccaRetries == 0)
          {
#ifdef NWK_PLL
            // Make sure the reference is cleared if it was a PLL packet
            sTxTimeStampAddr = NULL;
#endif

            /* set return value for failed transmit */
            txResult = MRFI_TX_RESULT_FAILED;

            /* break out of CCA algorithm loop */
            break;
          }

          /* decrement CCA retries before loop continues */
          ccaRetries--;

          /* turn off reciever to conserve power during backoff */
          Mrfi_RxModeOff();

          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();

#ifdef NWK_PLL
          /* re-write the data to the fifo so that the time stamp can be
           * re-introduced to the packet at the latest possible moment.
           */
          MRFI_PrepareToTx( pPacket ); // setup transmission again

#endif
        }
      }
      /*
       *  ---  end CCA Algorithm Loop ---
       * =============================================================================== */
    }
  }

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we are in the act of transmitting
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  
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
 * @fn          MRFI_RxIsr
 *
 * @brief       Receive interrupt.  Reads incoming packet from radio FIFO.  If CRC passes the
 *              external function MRFI_RxCompleteISR() is called.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
BSP_ISR_FUNCTION( MRFI_RxIsr, RF_VECTOR )
{
  uint8_t numBytes;
  uint8_t i, crcOK;

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

  /* Clear the MCU interrupt. */
  S1CON = 0x00;

  /* Process FIFOP interrupt */
  if(RFIF & IRQ_FIFOP)
  {
    /* We should receive this interrupt only in RX state
     * Should never receive it if RX was turned On only for
     * some internal mrfi processing like - during CCA.
     * Otherwise something is terribly wrong.
     */
    MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

    /* While there is at least one frame in the Rx FIFO */
    while(RFIF & IRQ_FIFOP)
    {
      /* Check for Rx overflow. Checking here means we may flush a valid frame */
      if ((RFSTATUS & FIFOP) && (!(RFSTATUS & FIFO)))
      {
        /* flush receive FIFO to recover from overflow (per datasheet, flush must be done twice) */
        MRFI_RADIO_FLUSH_RX_BUFFER();
        break;
      }

      /* ------------------------------------------------------------------
       *    Read packet from FIFO
       *   -----------------------
       */

      /*
       *  Determine number of bytes to be read from receive FIFO.  The first byte
       *  has the number of bytes in the packet.  A mask must be applied though
       *  to strip off unused bits.  The number of bytes in the packet does not
       *  include the length byte itself but does include the FCS (generically known
       *  as RX metrics).
       */
      numBytes = RFD & IEEE_PHY_PACKET_SIZE_MASK;

      /* see if frame will fit in maximum available buffer or is too small */
      if (((numBytes + MRFI_LENGTH_FIELD_SIZE - MRFI_RX_METRICS_SIZE) > MRFI_MAX_FRAME_SIZE) ||
           (numBytes < MRFI_MIN_SMPL_FRAME_SIZE))
      {
        /* packet is too big or too small. remove it from FIFO */
        for (i=0; i<numBytes; i++)
        {
          /* read and discard bytes from FIFO */
          RFD;
        }
      }
      else
      {
        uint8_t *p, *p1;

        /* set pointer at first byte of frame storage */
        p  = &mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFS];
        p1 = mrfiIncomingPacket.frame;

        /* Clear out my buffer to remove leftovers in case a bogus packet gets through */
        memset(p1, 0x0, sizeof(mrfiIncomingPacket.frame));

        /*
         *  Store frame length into the incoming packet memory.  Size of rx metrics
         *  is subtracted to get the MRFI frame length which separates rx metrics from
         *  the frame length.
         */
        *p = numBytes - MRFI_RX_METRICS_SIZE;

        /* read frame bytes from receive FIFO and store into incoming packet memory */
        for (i=0; i<numBytes-MRFI_RX_METRICS_SIZE; i++)
        {
          p++;
          *p = RFD;
        }

        /* read rx metrics and store to incoming packet */

        /* Add the RSSI offset to get the proper RSSI value. */
        mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_RSSI_OFS] = RFD + MRFI_RSSI_OFFSET;

        /* The second byte has 7 bits of Correlation value and 1 bit of
         * CRC pass/fail info. Remove the CRC pass/fail bit info.
         * Also note that for CC2430 radio this is the correlation value and not
         * the LQI value. Some convertion is needed to extract the LQI value.
         * This convertion is left to the application at this time.
         */
        crcOK = RFD;   /* get CRC/LQI byte */

        if (crcOK & MRFI_RX_METRICS_CRC_OK_MASK)
        {
          /* CRC OK. Save LQI info */
          mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] = (crcOK & MRFI_RX_METRICS_LQI_MASK);

          /* Eliminate frames that are the correct size but we can tell are bogus
           * by their frame control fields.
           */
          if ((p1[MRFI_FCF_OFFSET] == MRFI_FCF_0_7) &&
              (p1[MRFI_FCF_OFFSET+1] == MRFI_FCF_8_15))
          {
            /* call external, higher level "receive complete" */
            MRFI_RxCompleteISR();
          }
        }
      } /* Frame fits in the buffer. */

      /* Clear the interrupt source flag. This must be done after reading
       * the frame from the buffer. Otherwise the flag remains set. If another
       * frame is sitting in the buffer, the IRQ_FIFOP will be immediately set
       * again.
       */
       RFIF &= ~IRQ_FIFOP;
    }   /* While there is at least one frame in the Rx FIFO */
  }     /* Process FIFOP interrupt */
  else
  {
    /* Don't assert here. It is possible that the MCU interrupt was set by
     * FIFOP but we processed it in the while() loop of the FIFOP handler in
     * the previous run of this ISR.
     */

    /* If any other RF interrupt is enabled, add that handler here. */
  }

  /* Bugzilla Chip Bug#297: Don't delete */
  RFIF = 0xFF;
  
  __bsp_RESTORE_ISTATE__(istate);

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
  FSCTRLL = FREQ_2405MHZ + (5 * (phyChannel - 11));
#else
  /* frequency hopping requires more channels than are available using 802.15.4
   * so don't do any calculations, just jam the MHz offset from 2400MHz into the
   * control register
   */
//  FSCTRLL = FREQ_2405MHZ + phyChannel;    // non 802.15.4 channels for FHSS code
  FSCTRLL = FREQ_2405MHZ + (5 * (phyChannel - 11));
#endif

  /* Put the radio back in RX state, if it was in RX before channel change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
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
  /* if radio is asleep, wake it up */
  if(mrfiRadioState == MRFI_RADIO_STATE_OFF)
  {
    /* enter idle mode */
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;

    /* turn on radio power, pend for the power-up delay */
    RFPWR &= ~RREG_RADIO_PD;
    while((RFPWR & ADI_RADIO_PD));

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
  }
}


/**************************************************************************************************
 * @fn          MRFI_Sleep
 *
 * @brief       Request radio go to sleep.
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
    RFPWR |= RREG_RADIO_PD;

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;
  }
}


/**************************************************************************************************
 * @fn          MRFI_SetRxAddrFilter
 *
 * @brief       Set the address used for filtering received packets.
 *
 * @param       pAddr - pointer to address to use for filtering
 *
 * @return      none
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

  /* set the hardware address registers */
  PANIDL     = pAddr[0];
  PANIDH     = pAddr[1];
  SHORTADDRL = pAddr[2];
  SHORTADDRH = pAddr[3];

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
  MRFI_ASSERT( (PANIDL != 0xFF) && (PANIDH != 0xFF) ); /* filter address not set */

  /* enable hardware filtering on the radio */
  MDMCTRL0H |= ADDR_DECODE;
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
  /* disable hardware filtering on the radio */
  MDMCTRL0H &= ~ADDR_DECODE;
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
  int8_t rssi;

  /* Radio must be in RX state to measure rssi. */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  /*
   *  Assuming that the Radio was just turned on, we must wait for
   *  RSSI to be valid.
   */
  Mrfi_DelayUsec( MRFI_RSSI_VALID_DELAY_US );

  /* read RSSI value from hardware */
  rssi = RSSIL;

  /* apply offset given in datasheet */
  rssi = rssi + MRFI_RSSI_OFFSET;

  /* return RSSI value */
  return( rssi );
}


/**************************************************************************************************
 * @fn          MRFI_RandomByte
 *
 * @brief       Returns a random byte using a special hardware feature that generates new
 *              random values based on the truly random seed set earlier.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8_t MRFI_RandomByte(void)
{
  /* clock the random generator to get a new random value */
  ADCCON1 = (ADCCON1 & ~RCTRL_BITS) | RCTRL_CLOCK_LFSR;

  /* return newly randomized value from hardware */
  return(RNDH);
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
  while (milliseconds)
  {
    Mrfi_DelayUsec( APP_USEC_VALUE );
    milliseconds--;
  }
}

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
  /* send strobe to enter receive mode */
  RFST = ISRXON;

  /* enable receive interrupts */
  RFIM |= IM_FIFOP;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
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
  RFIM &= ~IM_FIFOP;

  /* turn off radio */
  RFST = ISRFOFF;

  /* flush the receive FIFO of any residual data */
  MRFI_RADIO_FLUSH_RX_BUFFER();

  /* clear receive interrupt */
  RFIF = ~IRQ_FIFOP;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
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
  uint8_t backoffs;
  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (MRFI_RandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  for (i=0; i<backoffs; i++)
  {
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
    MRFI_DelayUsec( MRFI_BACKOFF_PERIOD_USECS );
#else
    Mrfi_DelayUsec( MRFI_BACKOFF_PERIOD_USECS );
#endif
  }
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsec
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
#ifndef MRFI_TIMER_ALWAYS_ACTIVE
static void Mrfi_DelayUsec(uint16_t howLong)
{
  bspIState_t intState;
  uint16_t count = howLong/MRFI_MAX_DELAY_US;

  if (howLong)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(intState);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(intState);
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
 * @fn          MRFI_SetRFPwr
 *
 * @brief       Set RF pwoer level.
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

  /* make sure radio is off before changing power levels */
  Mrfi_RxModeOff();

  /* write value of new power level */
  TXCTRLL = mrfiRFPowerTable[idx];

  /* Put the radio back in RX state, if it was in RX before change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
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
    while( ( RFSTATUS & SFD ) != 0 );

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
      REVERSE_16(time->limit);
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
        static int32_t modulation = 0;
        modulation += sTmrRateOffset;
        {
          int32_t limit = modulation >> 24;
          limit += MRFI_ROLLOVER_LIMIT;
          BSP_TIMER_SET_OVERFLOW_VALUE( limit );
        }
        modulation &= BF_CLR( 24, 8 ); // clear upper byte
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
      static int32_t modulation = 0;
      modulation += sTmrRateOffset;
      {
        int32_t limit = modulation >> 24;
        limit += MRFI_ROLLOVER_LIMIT;
        BSP_TIMER_SET_OVERFLOW_VALUE( limit );
      }
      modulation &= BF_CLR( 24, 8 ); // clear upper byte
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

/*
 *  The current implementation requires an address size of four bytes.  These four bytes are
 *  spread across the PAN ID and the short address.  A larger address size is possible by using
 *  long address instead of short address.  The change is not difficult but it does require
 *  code modification.
 */
#if (MRFI_ADDR_SIZE != 4)
#error "ERROR:  Address size must be four bytes.  A different address size requires code modification."
#endif

 #define MRFI_RADIO_TX_FIFO_SIZE     128  /* from datasheet */

/* verify largest possible packet fits within FIFO buffer */
#if ((MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE) > MRFI_RADIO_TX_FIFO_SIZE)
#error "ERROR:  Maximum possible packet length exceeds FIFO buffer.  Decrease value of maximum application payload."
#endif

/*
 *  These asserts happen if there is extraneous compiler padding of arrays.
 *  Modify compiler settings for no padding, or, if that is not possible,
 *  comment out the offending asserts.
 */
BSP_STATIC_ASSERT(sizeof(mrfiLogicalChanTable) == ((sizeof(mrfiLogicalChanTable)/sizeof(mrfiLogicalChanTable[0])) * sizeof(mrfiLogicalChanTable[0])));
BSP_STATIC_ASSERT(sizeof(mrfiBroadcastAddr) == ((sizeof(mrfiBroadcastAddr)/sizeof(mrfiBroadcastAddr[0])) * sizeof(mrfiBroadcastAddr[0])));


/**************************************************************************************************
*/
