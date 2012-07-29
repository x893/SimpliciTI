/**************************************************************************************************
  Revised:        $Date: 2009-01-13 12:04:00 -0700 (Wed, 13 Jan 2009) $
  Revision:       $Revision: 18768 $

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
 *   Radios: CC2500, CC1100, CC1101
 *   Primary code file for supported radios.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <string.h>
#include "mrfi.h"
#include "bsp.h"
#include "bsp_macros.h"
#include "bsp_external/mrfi_board_defs.h"
#include "mrfi_defs.h"
#include "mrfi_spi.h"
#include "../common/mrfi_f1f2.h"
#include "nwk_pll.h"
#include "bsp_leds.h" // debug only

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */

#if (defined MRFI_CC2500)

 #define MRFI_RSSI_OFFSET    72   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define MRFI_RSSI_VALID_DELAY_US    1000

#elif (defined MRFI_CC1100)

  #define MRFI_RSSI_OFFSET    79   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define MRFI_RSSI_VALID_DELAY_US    1300

#elif (defined MRFI_CC1101) || (defined MRFI_CC1100E_470) || (defined MRFI_CC1100E_950)

  #define MRFI_RSSI_OFFSET    74   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
  #define MRFI_RSSI_VALID_DELAY_US    1300

#else
  #error "ERROR: RSSI offset value not defined for this radio"
#endif

#define MRFI_RANDOM_OFFSET                   67
#define MRFI_RANDOM_MULTIPLIER              109
#define MRFI_MIN_SMPL_FRAME_SIZE            (MRFI_HEADER_SIZE + NWK_HDR_SIZE)

/* GDO functionality */
#define MRFI_GDO_SYNC           6
#define MRFI_GDO_CCA            9
#define MRFI_GDO_PA_PD          27  /* low when transmit is active, low during sleep */
#define MRFI_GDO_LNA_PD         28  /* low when receive is active, low during sleep */

/* ---------- Radio Abstraction ---------- */
#if (defined MRFI_CC1100)
#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      3

#elif (defined MRFI_CC1101)
#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      4

#elif (defined MRFI_CC110L)
#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      7

#elif (defined MRFI_CC1100E_470)
#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      5

#elif (defined MRFI_CC1100E_950)
#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      5

#elif (defined MRFI_CC2500)
#define MRFI_RADIO_PARTNUM          0x80
#define MRFI_RADIO_MIN_VERSION      3
#else
#error "ERROR: Missing or unrecognized radio."
#endif

/* GDO0 output pin configuration */
#define MRFI_SETTING_IOCFG0     MRFI_GDO_SYNC

/* Main Radio Control State Machine control configuration:
 * Auto Calibrate - when going from IDLE to RX/TX
 * PO_TIMEOUT is extracted from SmartRF setting.
 * XOSC is OFF in Sleep state.
 */
#define MRFI_SETTING_MCSM0      (0x10 | (SMARTRF_SETTING_MCSM0 & (BV(2)|BV(3))))

/* Main Radio Control State Machine control configuration:
 * - Remain RX state after RX
 * - Go to IDLE after TX
 * - RSSI below threshold and NOT receiving.
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

/* FIFO threshold - this register has fields that need to be configured for the CC1101 */
#define MRFI_SETTING_FIFOTHR    (0x07 | (SMARTRF_SETTING_FIFOTHR & (BV(4)|BV(5)|BV(6))))

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

#ifdef MRFI_TIMER_ALWAYS_ACTIVE

// re-map static functions promoted to public for backwards compatibility
#define Mrfi_DelayUsec( a ) MRFI_DelayUsec( a )
#define Mrfi_DelayMs( a ) MRFI_DelayMs( a )

#endif // !MRFI_TIMER_ALWAYS_ACTIVE

#define MRFI_PKTSTATUS_CCA BV(4)
#define MRFI_PKTSTATUS_CS  BV(6)

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls
   * to the microsecond routine which can be calibrated independently.
   */
#if defined(SW_TIMER)
#define APP_USEC_VALUE    496
#else
#define APP_USEC_VALUE    1000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_SYNC_PIN_IS_HIGH()						MRFI_GDO0_PIN_IS_HIGH()
//#define MRFI_ENABLE_SYNC_PIN_INT()				MRFI_ENABLE_GDO0_INT()

#ifdef BSP_BOARD_STM32
	#define MRFI_ENABLE_SYNC_PIN_INT()				MRFI_GDO0_SYNC_INT_ENABLE  ( rx_isr_context )
	//#define MRFI_DISABLE_SYNC_PIN_INT()			MRFI_DISABLE_GDO0_INT()
	#define MRFI_DISABLE_SYNC_PIN_INT()				MRFI_GDO0_SYNC_INT_DISABLE ( rx_isr_context )
#else
	#define MRFI_ENABLE_SYNC_PIN_INT()				( INFIX( P, __mrfi_GDO0_PORT__, IE )  |=  ( ( rx_isr_context == true ) ? 0 : BV(__mrfi_GDO0_BIT__) ) )
	//#define MRFI_DISABLE_SYNC_PIN_INT()			MRFI_DISABLE_GDO0_INT()
	#define MRFI_DISABLE_SYNC_PIN_INT()				( INFIX( P, __mrfi_GDO0_PORT__, IE )  &= ~( ( rx_isr_context == true ) ? 0 : BV(__mrfi_GDO0_BIT__) ) )
#endif
#define MRFI_SYNC_PIN_INT_IS_ENABLED()              MRFI_GDO0_INT_IS_ENABLED()
#define MRFI_CLEAR_SYNC_PIN_INT_FLAG()              MRFI_CLEAR_GDO0_INT_FLAG()
#define MRFI_SYNC_PIN_INT_FLAG_IS_SET()             MRFI_GDO0_INT_FLAG_IS_SET()
#define MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT()     MRFI_CONFIG_GDO0_FALLING_EDGE_INT()

#define MRFI_PAPD_PIN_IS_HIGH()                     MRFI_SYNC_PIN_IS_HIGH()
#define MRFI_CLEAR_PAPD_PIN_INT_FLAG()              MRFI_CLEAR_SYNC_PIN_INT_FLAG()
#define MRFI_PAPD_INT_FLAG_IS_SET()                 MRFI_SYNC_PIN_INT_FLAG_IS_SET()
#define MRFI_CONFIG_PAPD_FALLING_EDGE_INT()         MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT()

#define MRFI_CONFIG_GDO0_AS_PAPD_SIGNAL()           mrfiSpiWriteReg(IOCFG0, MRFI_GDO_PA_PD)
#define MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL()           mrfiSpiWriteReg(IOCFG0, MRFI_GDO_SYNC)

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
    if(mrfiSpiReadReg(PKTSTATUS) & (MRFI_PKTSTATUS_CCA | MRFI_PKTSTATUS_CS))  \
    {                                                                         \
      break;                                                                  \
    }                                                                         \
    Mrfi_DelayUsec(64); /* sleep */                                           \
    delay -= 64;                                                              \
  }while(delay > 0);                                                          \
}                                                                             \

#endif // !MRFI_TIMER_ALWAYS_ACTIVE

#define MRFI_STROBE_IDLE_AND_WAIT()                   \
{                                                     \
  mrfiSpiCmdStrobe( SIDLE );                          \
  while (mrfiSpiCmdStrobe( SNOP ) & 0xF0) ;           \
}

/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */
static const uint8_t mrfiRadioCfg[][2] =
{
  /* internal radio configuration */
  {  IOCFG0,    MRFI_SETTING_IOCFG0       },
  {  MCSM1,     MRFI_SETTING_MCSM1        }, /* CCA mode, RX_OFF_MODE and TX_OFF_MODE */
  {  MCSM0,     MRFI_SETTING_MCSM0        }, /* AUTO_CAL and XOSC state in sleep */
  {  PKTLEN,    MRFI_SETTING_PKTLEN       },
  {  PKTCTRL0,  MRFI_SETTING_PKTCTRL0     },
#ifdef MRFI_CC1101
  {  FIFOTHR,   MRFI_SETTING_FIFOTHR      },
#endif

  /* imported SmartRF radio configuration */
  {  FSCTRL1,   SMARTRF_SETTING_FSCTRL1   },
  {  FSCTRL0,   SMARTRF_SETTING_FSCTRL0   },
  {  FREQ2,     SMARTRF_SETTING_FREQ2     },
  {  FREQ1,     SMARTRF_SETTING_FREQ1     },
  {  FREQ0,     SMARTRF_SETTING_FREQ0     },
  {  MDMCFG4,   SMARTRF_SETTING_MDMCFG4   },
  {  MDMCFG3,   SMARTRF_SETTING_MDMCFG3   },
  {  MDMCFG2,   SMARTRF_SETTING_MDMCFG2   },
  {  MDMCFG1,   SMARTRF_SETTING_MDMCFG1   },
  {  MDMCFG0,   SMARTRF_SETTING_MDMCFG0   },
  {  DEVIATN,   SMARTRF_SETTING_DEVIATN   },
  {  FOCCFG,    SMARTRF_SETTING_FOCCFG    },
  {  BSCFG,     SMARTRF_SETTING_BSCFG     },
  {  AGCCTRL2,  SMARTRF_SETTING_AGCCTRL2  },
  {  AGCCTRL1,  SMARTRF_SETTING_AGCCTRL1  },
  {  AGCCTRL0,  SMARTRF_SETTING_AGCCTRL0  },
  {  FREND1,    SMARTRF_SETTING_FREND1    },
  {  FREND0,    SMARTRF_SETTING_FREND0    },
  {  FSCAL3,    SMARTRF_SETTING_FSCAL3    },
  {  FSCAL2,    SMARTRF_SETTING_FSCAL2    },
  {  FSCAL1,    SMARTRF_SETTING_FSCAL1    },
  {  FSCAL0,    SMARTRF_SETTING_FSCAL0    },
  {  TEST2,     SMARTRF_SETTING_TEST2     },
  {  TEST1,     SMARTRF_SETTING_TEST1     },
  {  TEST0,     SMARTRF_SETTING_TEST0     },
};


/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void MRFI_GpioIsr(void); /* this called from mrfi_board.c */
static void Mrfi_SyncPinRxIsr(void);
static void MRFI_PrepareToTx( mrfiPacket_t * );
static void MRFI_CompleteTxPrep( mrfiPacket_t * );
static int8_t Mrfi_CalculateRssi(uint8_t rawValue);

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
//static void Mrfi_DelayUsecLong(uint32_t count, TimeoutTerminator_t term);
static bool Mrfi_CheckSem( void );
static bool Mrfi_ValidateRSSI( void );
#else
static void Mrfi_DelayUsec(uint16_t);
static void Mrfi_DelayUsecSem(uint16_t howLong);
#endif
static bool mrfi_TxCCADone( void );
static bool mrfi_TxImmediateDone( void );

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8_t mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;
static mrfiPacket_t mrfiIncomingPacket;
static uint8_t mrfiRndSeed = 0;

/* reply delay support */
static volatile uint8_t  sKillSem = 0;
static volatile uint8_t  sReplyDelayContext = 0;
static          uint16_t sReplyDelayScalar = 0;
static          uint16_t sBackoffHelper = 0;

static bool rx_isr_context = false;

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
	/* ------------------------------------------------------------------
	 *    Initialization
	 *   -----------------
	 */
	memset(&mrfiIncomingPacket, 0x0, sizeof(mrfiIncomingPacket));

	/* ------------------------------------------------------------------
	 *    Configure timer
	 *   ----------------------
	 */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
	stx_active = false;
	/* initialize the timer for operation if pll is enabled */
	Mrfi_TimerInit( );
	sOne_ms_event_hook = NULL;
#endif // MRFI_TIMER_ALWAYS_ACTIVE
	
	FHSS_ACTIVE( sHopCount = 0 );
	FHSS_ACTIVE( sLogicalChannel = MRFI_RandomByte( ) % MRFI_NUM_LOGICAL_CHANS );
	FHSS_ACTIVE( sHopNowSem = 0 );
	FHSS_ACTIVE( sHopRate = MRFI_HOP_TIME_ms - 1 );

	/* initialize GPIO pins */
	MRFI_CONFIG_GDO0_PIN_AS_INPUT();

	/* initialize SPI */
	mrfiSpiInit();

	/* ------------------------------------------------------------------
	 *    Radio power-up reset
	 *   ----------------------
	 */
	MRFI_ASSERT(MRFI_SPI_CSN_IS_HIGH());

	/* pulse CSn low then high */
	MRFI_SPI_DRIVE_CSN_LOW();
	Mrfi_DelayUsec(10);
	MRFI_SPI_DRIVE_CSN_HIGH();

	/* hold CSn high for at least 40 microseconds */
	Mrfi_DelayUsec(40);

	/* pull CSn low and wait for SO to go low */
	MRFI_SPI_DRIVE_CSN_LOW();
	while (MRFI_SPI_SO_IS_HIGH());

	/* directly send strobe command - cannot use function as it affects CSn pin */
	MRFI_SPI_WRITE_BYTE(SRES);
	MRFI_SPI_WAIT_DONE();

	/* wait for SO to go low again, reset is complete at that point */
	while (MRFI_SPI_SO_IS_HIGH());

	/* return CSn pin to its default high level */
	MRFI_SPI_DRIVE_CSN_HIGH();

	/* ------------------------------------------------------------------
	 *    Run-time integrity checks
	 *   ---------------------------
	 */

	/* verify that SPI is working, PKTLEN is an arbitrary read/write register used for testing */
#ifdef MRFI_ASSERTS_ARE_ON
	#define TEST_VALUE 0xA5
	mrfiSpiWriteReg( PKTLEN, TEST_VALUE );
	MRFI_ASSERT( mrfiSpiReadReg( PKTLEN ) == TEST_VALUE ); /* SPI is not responding */
#endif

	/* verify the correct radio is installed */
	MRFI_ASSERT( mrfiSpiReadReg( PARTNUM ) == MRFI_RADIO_PARTNUM );      /* incorrect radio specified */
	MRFI_ASSERT( mrfiSpiReadReg( VERSION ) >= MRFI_RADIO_MIN_VERSION );  /* obsolete radio specified  */

	/* ------------------------------------------------------------------
	 *    Configure radio
	 *   -----------------
	 */

	/* initialize radio registers */
	{
		uint8_t i;
		for (i = 0; i < (sizeof(mrfiRadioCfg) / sizeof(mrfiRadioCfg[0])); i++)
		{
			mrfiSpiWriteReg(mrfiRadioCfg[i][0], mrfiRadioCfg[i][1]);
		}
	}

	/* Initial radio state is IDLE state */
	mrfiRadioState = MRFI_RADIO_STATE_IDLE;

	/* set default channel */
	MRFI_SetLogicalChannel( 0 );

	/* set default power */
	MRFI_SetRFPwr(MRFI_NUM_POWER_SETTINGS - 1);

	/* Generate Random seed:
	 * We will use the RSSI value to generate our random seed.
	 */

	/* Put the radio in RX state */
	mrfiSpiCmdStrobe( SRX );

	/* delay for the rssi to be valid */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
	MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
	MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE

	/* use most random bit of rssi to populate the random seed */
	{
		uint8_t i;
		for(i=0; i<16; i++)
		{
			mrfiRndSeed = (mrfiRndSeed << 1) | (mrfiSpiReadReg(RSSI) & 0x01);
		}
	}

	/* Force the seed to be non-zero by setting one bit, just in case... */
	mrfiRndSeed |= 0x0080;

	/* Turn off RF. */
	Mrfi_RxModeOff();

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
	* Note that we assume a 26 MHz clock for the radio...
	* ***************************************************************************************
	*/
#define   MRFI_RADIO_OSC_FREQ        26000000
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

	/* Clean out buffer to protect against spurious frames */
	memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));
	memset(mrfiIncomingPacket.rxMetrics, 0x00, sizeof(mrfiIncomingPacket.rxMetrics));
	
	/* ------------------------------------------------------------------
	 *    Configure interrupts
	 *   ----------------------
	 */

	/*
	 *  Configure and enable the SYNC signal interrupt.
	 *
	 *  This interrupt is used to indicate receive.  The SYNC signal goes
	 *  high when a receive OR a transmit begins.  It goes high once the
	 *  sync word is received or transmitted and then goes low again once
	 *  the packet completes.
	 */
	MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL();
	MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT();
	MRFI_CLEAR_SYNC_PIN_INT_FLAG();

	/* enable global interrupts */
	BSP_ENABLE_INTERRUPTS();
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
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = true; // indicate we are in the act of transmitting
#endif // MRFI_TIMER_ALWAYS_ACTIVE

   mrfiSpiCmdStrobe( SFTX ); // flush the tx fifo

#ifndef NWK_PLL
  {
    /* compute number of bytes to write to transmit FIFO */
    uint8_t txBufLen = pPacket->frame[MRFI_LENGTH_FIELD_OFS]
                                                   + MRFI_LENGTH_FIELD_SIZE;
    mrfiSpiWriteTxFifo(pPacket->frame, txBufLen);
  }
#else // defined NWK_PLL
  mrfiSpiWriteTxFifo(pPacket->frame, MRFI_PAYLOAD_OFFSET);
#endif
  return;
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
  /* compute number of bytes to write to transmit FIFO */
  uint8_t txBufLen = pPacket->frame[MRFI_LENGTH_FIELD_OFS]
                             + MRFI_LENGTH_FIELD_SIZE - MRFI_PAYLOAD_OFFSET;

  /* ------------------------------------------------------------------
   *    Write packet to transmit FIFO
   *   --------------------------------
   */

  /* if sTxTimeStampAddr is not null then the network application has
   * filled it in because it is a PLL packet that requires the transmit
   * time.  After filling in the time stamp at the address specified,
   * delete the reference so future packets will not have a time stamp
   * acquisition.
   */
  if( sTxTimeStampAddr != NULL ) // if need to add time stamp to packet
    MRFI_GetLocalRawTime(sTxTimeStampAddr); // fill in the packet data

  mrfiSpiWriteTxFifo(pPacket->frame + MRFI_PAYLOAD_OFFSET, txBufLen);
#else // defined NWK_PLL
  (void) pPacket;
#endif
}


/**************************************************************************************************
 * @fn          mrfi_TxCCADone
 *
 * @brief       Indicates status of transmission completion
 *
 * @param       None
 *
 * @return      true if transmission is complete, false otherwise
 **************************************************************************************************
 */
bool mrfi_TxCCADone( void )
{
	return MRFI_PAPD_PIN_IS_HIGH(); // || marc_state == RF_SM_RX || marc_state == RF_SM_IDLE;
}

/**************************************************************************************************
 * @fn          mrfi_TxImmediateDone
 *
 * @brief       Indicates status of transmission completion
 *
 * @param       None
 *
 * @return      true if transmission is complete, false otherwise
 **************************************************************************************************
 */
bool mrfi_TxImmediateDone( void )
{
  return MRFI_SYNC_PIN_INT_FLAG_IS_SET(); // || marc_state == RF_SM_RX || marc_state == RF_SM_IDLE;
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
#ifdef NWK_PLL
    bspIState_t s;
#endif
  uint8_t ccaRetries;
  uint8_t returnValue = MRFI_TX_RESULT_SUCCESS;

  /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Turn off reciever. We can ignore/drop incoming packets during transmit. */
  Mrfi_RxModeOff();

  MRFI_PrepareToTx( pPacket );


  /* ------------------------------------------------------------------
   *    Immediate transmit
   *   ---------------------
   */
  if (txType == MRFI_TX_TYPE_FORCED)
  {
//#ifdef NWK_PLL
//    BSP_ENTER_CRITICAL_SECTION(s);
//#endif
//    MRFI_CompleteTxPrep( pPacket );
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

    /* Issue the TX strobe. */
    mrfiSpiCmdStrobe( STX );
#ifdef NWK_PLL
    BSP_EXIT_CRITICAL_SECTION(s);
#endif

    /* Wait for transmit to complete */
    Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                        MRFI_MAX_TRANSMIT_TIME_us % 1000,
                        mrfi_TxImmediateDone );

    /* Clear the interrupt flag */
    MRFI_CLEAR_SYNC_PIN_INT_FLAG();
  }
  else
  {
    /* ------------------------------------------------------------------
     *    CCA transmit
     *   ---------------
     */

    MRFI_ASSERT( txType == MRFI_TX_TYPE_CCA );

    /* set number of CCA retries */
    ccaRetries = MRFI_CCA_RETRIES;

    /* For CCA algorithm, we need to know the transition from the RX state to
     * the TX state. There is no need for SYNC signal in this logic. So we
     * can re-configure the GDO_0 output from the radio to be PA_PD signal
     * instead of the SYNC signal.
     * Since both SYNC and PA_PD are used as falling edge interrupts, we
     * don't need to reconfigure the MCU input.
     */
    MRFI_CONFIG_GDO0_AS_PAPD_SIGNAL();

    /* ===============================================================================
     *    Main Loop
     *  =============
     */
    for (;;)
    {
      /* Radio must be in RX mode for CCA to happen.
       * Otherwise it will transmit without CCA happening.
       */

      /* Can not use the Mrfi_RxModeOn() function here since it turns on the
       * Rx interrupt, which we don't want in this case.
       */
      mrfiSpiCmdStrobe( SRX );

      /* wait for the rssi to be valid. */
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_WaitTimeoutUsec(MRFI_RSSI_VALID_DELAY_US, Mrfi_ValidateRSSI);
#else // MRFI_TIMER_ALWAYS_ACTIVE
      MRFI_RSSI_VALID_WAIT();
#endif // MRFI_TIMER_ALWAYS_ACTIVE
#ifdef NWK_PLL
        BSP_ENTER_CRITICAL_SECTION(s);
        if( stx_active == false ) // if the channel was changed
        {
          BSP_EXIT_CRITICAL_SECTION(s);
          Mrfi_RxModeOff();            // turn off the radio
          MRFI_PrepareToTx( pPacket ); // setup transmission again
          continue; // restart the cca loop
        }

      MRFI_CompleteTxPrep( pPacket );
#endif

      /*
       *  Clear the PA_PD pin interrupt flag.  This flag, not the interrupt itself,
       *  is used to capture the transition that indicates a transmit was started.
       *  The pin level cannot be used to indicate transmit success as timing may
       *  prevent the transition from being detected.  The interrupt latch captures
       *  the event regardless of timing.
       */
      MRFI_CLEAR_PAPD_PIN_INT_FLAG();

      /* Issue the TX strobe. */
      mrfiSpiCmdStrobe( STX );

#ifdef NWK_PLL
      BSP_EXIT_CRITICAL_SECTION(s);
#endif

      /* Delay long enough for the PA_PD signal to indicate a
       * successful transmit. This is the 250 XOSC periods
       * (9.6 us for a 26 MHz crystal) See section 19.6 of 2500 datasheet.
       * Found out that we need a delay of atleast 20 us on CC2500 and
       * 25 us on CC1100 to see the PA_PD signal change.
       */
      Mrfi_DelayUsec(25);


      /* PA_PD signal goes from HIGH to LOW when going from RX state.
       * This transition is trapped as a falling edge interrupt flag
       * to indicate that CCA passed and the transmit has started.
       */
      if (MRFI_PAPD_INT_FLAG_IS_SET())
      {
        /* ------------------------------------------------------------------
        *    Clear Channel Assessment passed.
        *   ----------------------------------
        */

        /* Clear the PA_PD int flag */
        MRFI_CLEAR_PAPD_PIN_INT_FLAG();

        Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                            MRFI_MAX_TRANSMIT_TIME_us % 1000,
                            mrfi_TxCCADone );

        /* transmit done, break */
        break;
      }
      else
      {
        /* ------------------------------------------------------------------
         *    Clear Channel Assessment failed.
         *   ----------------------------------
         */

        /* Turn off radio and save some power during backoff */

        /* NOTE: Can't use Mrfi_RxModeOff() - since it tries to update the
         * sync signal status which we are not using during the TX operation.
         */
        MRFI_STROBE_IDLE_AND_WAIT();

        /* flush the receive FIFO of any residual data */
        mrfiSpiCmdStrobe( SFRX );

        /* Retry ? */
        if (ccaRetries != 0)
        {
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
          stx_active = false;
#endif
          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();

          MRFI_PrepareToTx( pPacket ); // setup transmission again

          /* decrement CCA retries before loop continues */
          ccaRetries--;
        }
        else /* No CCA retries are left, abort */
        {
          /* set return value for failed transmit and break */
          returnValue = MRFI_TX_RESULT_FAILED;
          break;
        }
      } /* CCA Failed */
    } /* CCA loop */
  }/* txType is CCA */

  /* Done with TX. Clean up time... */

  /* Radio is already in IDLE state */

#ifdef NWK_PLL
  stx_active = false;
  // Packet transmitted, regardless of packet type, remove reference.
  sTxTimeStampAddr = NULL;
#endif
  /*
   * Flush the transmit FIFO.  It must be flushed so that
   * the next transmit can start with a clean slate.
   */
  mrfiSpiCmdStrobe( SFTX );

  /* Restore GDO_0 to be SYNC signal */
  MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL();

  /* If the radio was in RX state when transmit was attempted,
   * put it back to Rx On state.
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
 * @fn          Mrfi_SyncPinRxIsr
 *
 * @brief       This interrupt is called when the SYNC signal transition from high to low.
 *              The sync signal is routed to the sync pin which is a GPIO pin.  This high-to-low
 *              transition signifies a receive has completed.  The SYNC signal also goes from
 *              high to low when a transmit completes.   This is protected against within the
 *              transmit function by disabling sync pin interrupts until transmit completes.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_SyncPinRxIsr(void)
{
	uint8_t frameLen = 0x00;
	uint8_t rxBytes;

	/* We should receive this interrupt only in RX state
	 * Should never receive it if RX was turned On only for
	 * some internal mrfi processing like - during CCA.
	 * Otherwise something is terribly wrong.
	 */
	MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

	/* ------------------------------------------------------------------
	 *    Get RXBYTES
	 *   -------------
	 */

	/*
	 *  Read the RXBYTES register from the radio.
	 *  Bit description of RXBYTES register:
	 *    bit 7     - RXFIFO_OVERFLOW, set if receive overflow occurred
	 *    bits 6:0  - NUM_BYTES, number of bytes in receive FIFO
	 *
	 *  Due a chip bug, the RXBYTES register must read the same value twice
	 *  in a row to guarantee an accurate value.
	 */
	{
		uint8_t rxBytesVerify;
		rxBytesVerify = mrfiSpiReadReg( RXBYTES );
		do {
			rxBytes = rxBytesVerify;
			rxBytesVerify = mrfiSpiReadReg( RXBYTES );
		}
		while (rxBytes != rxBytesVerify);
	}

	/* ------------------------------------------------------------------
	 *    FIFO empty?
	 *   -------------
	 */

	/*
	 *  See if the receive FIFIO is empty before attempting to read from it.
	 *  It is possible nothing the FIFO is empty even though the interrupt fired.
	 *  This can happen if address check is enabled and a non-matching packet is
	 *  received.  In that case, the radio automatically removes the packet from
	 *  the FIFO.
	 */
	if (rxBytes == 0)
	{
		/* receive FIFO is empty - do nothing, skip to end */
	}
	else
	{
		/* receive FIFO is not empty, continue processing */

		/* ------------------------------------------------------------------
		 *    Process frame length
		 *   ----------------------
		 */

		/* read the first byte from FIFO - the packet length */
		mrfiSpiReadRxFifo(&frameLen, MRFI_LENGTH_FIELD_SIZE);

		/*
		 *  Make sure that the frame length just read corresponds to number of bytes in the buffer.
		 *  If these do not match up something is wrong.
		 *
		 *  This can happen for several reasons:
		 *   1) Incoming packet has an incorrect format or is corrupted.
		 *   2) The receive FIFO overflowed.  Overflow is indicated by the high
		 *      bit of rxBytes.  This guarantees the value of rxBytes value will not
		 *      match the number of bytes in the FIFO for overflow condition.
		 *   3) Interrupts were blocked for an abnormally long time which
		 *      allowed a following packet to at least start filling the
		 *      receive FIFO.  In this case, all received and partially received
		 *      packets will be lost - the packet in the FIFO and the packet coming in.
		 *      This is the price the user pays if they implement a giant
		 *      critical section.
		 *   4) A failed transmit forced radio to IDLE state to flush the transmit FIFO.
		 *      This could cause an active receive to be cut short.
		 *
		 *  Also check the sanity of the length to guard against rogue frames.
		 */
		if ((rxBytes != (frameLen + MRFI_LENGTH_FIELD_SIZE + MRFI_RX_METRICS_SIZE))
		||	((frameLen + MRFI_LENGTH_FIELD_SIZE) > MRFI_MAX_FRAME_SIZE)
		||	(frameLen < MRFI_MIN_SMPL_FRAME_SIZE)
			)
		{
			bspIState_t s;

			/* mismatch between bytes-in-FIFO and frame length */

			/*
			 *  Flush receive FIFO to reset receive.  Must go to IDLE state to do this.
			 *  The critical section guarantees a transmit does not occur while cleaning up.
			 */
			BSP_ENTER_CRITICAL_SECTION(s);
			MRFI_STROBE_IDLE_AND_WAIT();
			mrfiSpiCmdStrobe( SFRX );
			mrfiSpiCmdStrobe( SRX );
			BSP_EXIT_CRITICAL_SECTION(s);

			/* flush complete, skip to end */
		}
		else
		{
			/* bytes-in-FIFO and frame length match up - continue processing */

			/* ------------------------------------------------------------------
			 *    Get packet
			 *   ------------
			 */

			/* clean out buffer to help protect against spurious frames */
			memset(mrfiIncomingPacket.frame, 0x00, sizeof(mrfiIncomingPacket.frame));

			/* set length field */
			mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFS] = frameLen;

			/* get packet from FIFO */
			mrfiSpiReadRxFifo(&(mrfiIncomingPacket.frame[MRFI_FRAME_BODY_OFS]), frameLen);

			/* get receive metrics from FIFO */
			mrfiSpiReadRxFifo(&(mrfiIncomingPacket.rxMetrics[0]), MRFI_RX_METRICS_SIZE);

			MRFI_DISABLE_SYNC_PIN_INT( ); // disable radio sync interrupt so no more occur
			rx_isr_context = true;        // deactivate sync pin interrupt enable/disable macros
			BSP_ENABLE_INTERRUPTS( );     // enable interrupts so higher priority irqs can occur

			/* ------------------------------------------------------------------
			 *    CRC check
			 *   ------------
			 */
			/*
			 *  Note!  Automatic CRC check is not, and must not, be enabled.  This feature
			 *  flushes the *entire* receive FIFO when CRC fails.  If this feature is
			 *  enabled it is possible to be reading from the FIFO and have a second
			 *  receive occur that fails CRC and automatically flushes the receive FIFO.
			 *  This could cause reads from an empty receive FIFO which puts the radio
			 *  into an undefined state.
			 */

			/* determine if CRC failed */
			if (!(mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] & MRFI_RX_METRICS_CRC_OK_MASK))
			{
				/* CRC failed - do nothing, skip to end */
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
			BSP_DISABLE_INTERRUPTS( );    // disable interrupts so we can enable radio sync interrupt again
			rx_isr_context = false;       // activate sync pin interrupt enable/disable macros
			MRFI_ENABLE_SYNC_PIN_INT( );  // enable radio sync interrupt again
		}
	}

	/* ------------------------------------------------------------------
	 *    End of function
	 *   -------------------
	 */
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
  /* clear any residual receive interrupt */
  MRFI_CLEAR_SYNC_PIN_INT_FLAG();

  /* send strobe to enter receive mode */
  mrfiSpiCmdStrobe( SRX );

  /* enable receive interrupts */
  MRFI_ENABLE_SYNC_PIN_INT();

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
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

  /* if radio is off, turn it on */
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
  /* disable receive interrupts */
  MRFI_DISABLE_SYNC_PIN_INT();

  /* turn off radio */
  MRFI_STROBE_IDLE_AND_WAIT();

  /* flush the receive FIFO of any residual data */
  mrfiSpiCmdStrobe( SFRX );

  /* clear receive interrupt */
  MRFI_CLEAR_SYNC_PIN_INT_FLAG();

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
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
  bspIState_t s;

  /* Critical section necessary for watertight testing and
   * setting of state variables.
   */
  BSP_ENTER_CRITICAL_SECTION(s);

  /* If radio is not asleep, put it to sleep */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* go to idle so radio is in a known state before sleeping */
    MRFI_RxIdle();

    mrfiSpiCmdStrobe( SPWD );

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;
  }

  BSP_EXIT_CRITICAL_SECTION(s);
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
  /* if radio is already awake, just ignore wakeup request */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    return;
  }

  /* drive CSn low to initiate wakeup */
  MRFI_SPI_DRIVE_CSN_LOW();

  /* wait for MISO to go high indicating the oscillator is stable */
  while (MRFI_SPI_SO_IS_HIGH());

  /* wakeup is complete, drive CSn high and continue */
  MRFI_SPI_DRIVE_CSN_HIGH();

/*
 *  The test registers must be restored after sleep for the CC1100 and CC2500 radios.
 *  This is not required for the CC1101 radio.
 */
#ifndef MRFI_CC1101
  mrfiSpiWriteReg( TEST2, SMARTRF_SETTING_TEST2 );
  mrfiSpiWriteReg( TEST1, SMARTRF_SETTING_TEST1 );
  mrfiSpiWriteReg( TEST0, SMARTRF_SETTING_TEST0 );
#endif

  /* enter idle mode */
  mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  MRFI_STROBE_IDLE_AND_WAIT();

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  stx_active = false; // indicate we're not in transmit
#endif // MRFI_TIMER_ALWAYS_ACTIVE
}


/**************************************************************************************************
 * @fn          MRFI_GpioIsr
 *
 * @brief       Interrupt Service Routine for handling GPIO interrupts.  The sync pin interrupt
 *              comes in through GPIO.  This function is designed to be compatible with "ganged"
 *              interrupts.  If the GPIO interrupt services more than just a single pin (very
 *              common), this function just needs to be called from the higher level interrupt
 *              service routine.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_GpioIsr(void)
{
#ifdef NWK_PLL
	/*  If the network pll is running then we need to acquire
	 *  the receive time stamp (at least as close to the actual
	 *  time that the receive occurred)
	 */
	if( sRxTimeStampAddr != NULL )
		MRFI_GetLocalRawTime( sRxTimeStampAddr );
#endif

	/* see if sync pin interrupt is enabled and has fired */
	if (MRFI_SYNC_PIN_INT_IS_ENABLED() && MRFI_SYNC_PIN_INT_FLAG_IS_SET())
	{
		/*  clear the sync pin interrupt, run sync pin ISR */

		/*
		 *  NOTE!  The following macro clears the interrupt flag but it also *must*
		 *  reset the interrupt capture.  In other words, if a second interrupt
		 *  occurs after the flag is cleared it must be processed, i.e. this interrupt
		 *  exits then immediately starts again.  Most microcontrollers handle this
		 *  naturally but it must be verified for every target.
		 */
		MRFI_CLEAR_SYNC_PIN_INT_FLAG();
		Mrfi_SyncPinRxIsr();
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
	uint8_t regValue;
	
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

	/* Read the RSSI value */
	regValue = mrfiSpiReadReg( RSSI );

	/* convert and do offset compensation */
	return( Mrfi_CalculateRssi(regValue) );
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
		MRFI_DelayUsec( sBackoffHelper );
#else
		Mrfi_DelayUsec( sBackoffHelper );
#endif
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
		while( MRFI_PAPD_PIN_IS_HIGH() );

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
  return mrfiSpiReadReg(PKTSTATUS) & (MRFI_PKTSTATUS_CCA | MRFI_PKTSTATUS_CS);
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
  static int c = 0;
  c++;
  BSP_CRITICAL_STATEMENT( sReplyDelayContext = 1 );
  Mrfi_DelayUsecLong( sReplyDelayScalar, 0, Mrfi_CheckSem );
  BSP_CRITICAL_STATEMENT( sKillSem = sReplyDelayContext = 0 );
}

#else // MRFI_TIMER_ALWAYS_ACTIVE

/****************************************************************************************************
 * @fn          Mrfi_DelayUsec
 *
 * @brief       Execute a delay loop using HW timer. The macro actually used to do the delay
 *              is not thread-safe. This routine makes the delay execution thread-safe by breaking
 *              up the requested delay up into small chunks and executing each chunk as a critical
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
	uint16_t count = howLong / MRFI_MAX_DELAY_US;

	if (howLong)
	{
		do
		{
			BSP_ENTER_CRITICAL_SECTION(s);
			BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
			BSP_EXIT_CRITICAL_SECTION(s);
		} while (count--);
	}
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


#define MRFI_RADIO_TX_FIFO_SIZE     64  /* from datasheet */

/* verify largest possible packet fits within FIFO buffer */
#if ((MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE) > MRFI_RADIO_TX_FIFO_SIZE)
#error "ERROR:  Maximum possible packet length exceeds FIFO buffer.  Decrease value of maximum application payload."
#endif

/* verify that the SmartRF file supplied is compatible */
#if ((!defined SMARTRF_RADIO_CC2500) && \
     (!defined SMARTRF_RADIO_CC1100) && \
     (!defined SMARTRF_RADIO_CC1101) && \
     (!defined SMARTRF_RADIO_CC1100E))
#error "ERROR:  The SmartRF export file is not compatible."
#endif

/*
 *  These asserts happen if there is extraneous compiler padding of arrays.
 *  Modify compiler settings for no padding, or, if that is not possible,
 *  comment out the offending asserts.
 */
BSP_STATIC_ASSERT(sizeof(mrfiRadioCfg) == ((sizeof(mrfiRadioCfg)/sizeof(mrfiRadioCfg[0])) * sizeof(mrfiRadioCfg[0])));


/**************************************************************************************************
*/
