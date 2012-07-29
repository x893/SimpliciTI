#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_pll.h"

#ifndef APP_AUTO_ACK
	#error ERROR: Must define the macro APP_AUTO_ACK for this application.
#endif

void toggleLED(uint8_t);

/**************************** COMMENTS ON ASYNC LISTEN APPLICATION ***********************
Summary:
  This AP build includes implementation of an unknown number of end device peers in
  addition to AP functionality. In this scenario all End Devices establish a link to
  the AP and only to the AP. The AP acts as a data hub. All End Device peers are on
  the AP and not on other distinct ED platforms.

  There is still a limit to the number of peers supported on the AP that is defined
  by the macro NUM_CONNECTIONS. The AP will support NUM_CONNECTIONS or fewer peers
  but the exact number does not need to be known at build time.

  In this special but common scenario SimpliciTI restricts each End Device object to a
  single connection to the AP. If multiple logical connections are required these must
  be accommodated by supporting contexts in the application payload itself.

Solution overview:
  When a new peer connection is required the AP main loop must be notified. In essence
  the main loop polls a semaphore to know whether to begin listening for a peer Link
  request from a new End Device. There are two solutions: automatic notification and
  external notification. The only difference between the automatic notification
  solution and the external notification solution is how the listen semaphore is
  set. In the external notification solution the sempahore is set by the user when the
  AP is stimulated for example by a button press or a commend over a serial link. In the
  automatic scheme the notification is accomplished as a side effect of a new End Device
  joining.

  The Rx callback must be implemented. When the callback is invoked with a non-zero
  Link ID the handler could set a semaphore that alerts the main work loop that a
  SMPL_Receive() can be executed successfully on that Link ID.

  If the callback conveys an argument (LinkID) of 0 then a new device has joined the
  network. A SMPL_LinkListen() should be executed.

  Whether the joining device supports ED objects is indirectly inferred on the joining
  device from the setting of the NUM_CONNECTIONS macro. The value of this macro should
  be non-zero only if ED objects exist on the device. This macro is always non-zero
  for ED-only devices. But Range Extenders may or may not support ED objects. The macro
  should be be set to 0 for REs that do not also support ED objects. This prevents the
  Access Point from reserving resources for a joinng device that does not support any
  End Device Objects and it prevents the AP from executing a SMPL_LinkListen(). The
  Access Point will not ever see a Link frame if the joining device does not support
  any connections.

  Each joining device must execute a SMPL_Link() after receiving the join reply from the
  Access Point. The Access Point will be listening.

*************************** END COMMENTS ON ASYNC LISTEN APPLICATION ********************/

/************  THIS SOURCE FILE REPRESENTS THE AUTOMATIC NOTIFICATION SOLUTION ************/

/* reserve space for the maximum possible peer Link IDs */
static linkID_t sLID[NUM_CONNECTIONS] = {0};
static uint8_t  sNumCurrentPeers = 0;

/* callback handler */
static uint8_t sCB(linkID_t);

/* received message handler */
static void processMessage(linkID_t, uint8_t *, uint8_t);

/* Frequency Agility helper functions */
static void    checkChangeChannel(void);
static void    changeChannel(void);

/* work loop semaphores */
static volatile uint8_t sPeerFrameSem = 0;
static volatile uint8_t sJoinSem = 0;

#ifdef FREQUENCY_AGILITY
	/*	************** BEGIN interference detection support	*/
	#define INTERFERNCE_THRESHOLD_DBM (-70)
	#define SSIZE    25
	#define IN_A_ROW  3
	static int8_t  sSample[SSIZE];
	static uint8_t sChannel = 0;
#endif  /* FREQUENCY_AGILITY */

/* blink LEDs when channel changes... */
static volatile uint8_t sBlinky = 0;

	/*	************** END interference detection support	*/

#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

int main (void)
{
	bspIState_t intState;

#ifdef FREQUENCY_AGILITY
	memset(sSample, 0x0, sizeof(sSample));
#endif

	BSP_Init();

	SMPL_Init(sCB);

	/* green and red LEDs on solid to indicate waiting for a Join. */
	if (!BSP_LED2_IS_ON())	{	toggleLED(2);	}
	if (!BSP_LED1_IS_ON())	{	toggleLED(1);	}

	/* main work loop */
	while (1)
	{
		/* manage FHSS schedule if FHSS is active */
		FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
    
		/* Wait for the Join semaphore to be set by the receipt of a Join frame from a
		 * device that supports an End Device.
		 *
		 * An external method could be used as well. A button press could be connected
		 * to an ISR and the ISR could set a semaphore that is checked by a function
		 * call here, or a command shell running in support of a serial connection
		 * could set a semaphore that is checked by a function call.
		 */
		if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
		{
			/* listen for a new connection */
			while (1)
			{
				if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
					break;
				/* Implement fail-to-link policy here. otherwise, listen again. */
			}
			sNumCurrentPeers++;

			BSP_ENTER_CRITICAL_SECTION(intState);
			sJoinSem--;
			BSP_EXIT_CRITICAL_SECTION(intState);
		}

		/* Have we received a frame on one of the ED connections?
		 * No critical section -- it doesn't really matter much if we miss a poll
		 */
		if (sPeerFrameSem)
		{
			uint8_t msg[MAX_APP_PAYLOAD], len, i;

			/* process all frames waiting */
			for (i = 0; i < sNumCurrentPeers; ++i)
			{
				if (SMPL_SUCCESS == SMPL_Receive(sLID[i], msg, &len))
				{
					processMessage(sLID[i], msg, len);

					BSP_ENTER_CRITICAL_SECTION(intState);
					sPeerFrameSem--;
					BSP_EXIT_CRITICAL_SECTION(intState);
				}
			}
		}
		if (BSP_BUTTON1())
		{
			SPIN_ABOUT_A_QUARTER_SECOND;  /* debounce */
			changeChannel();
		}
		else
		{
			checkChangeChannel();
		}

		BSP_ENTER_CRITICAL_SECTION(intState);
		if (sBlinky)
		{
			if (++sBlinky >= 0xF)
			{
				sBlinky = 1;
				toggleLED(1);
				toggleLED(2);
			}
		}
		BSP_EXIT_CRITICAL_SECTION(intState);
	}
}

void toggleLED(uint8_t which)
{
	if (1 == which)			{	BSP_TOGGLE_LED1();	}
	else if (2 == which)	{	BSP_TOGGLE_LED2();	}
}

/* Runs in ISR context. Reading the frame should be done in the */
/* application thread not in the ISR thread. */
static uint8_t sCB(linkID_t lid)
{
	if (lid)
	{
		sPeerFrameSem++;
		sBlinky = 0;
	}
	else
	{
		sJoinSem++;
	}
	/* leave frame to be read by application. */
	return 0;
}

static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len)
{
	DEBUG_PRINT("LID:%d Packet:%d bytes\n", lid, len);
	/* do something useful */
	if (len)
		toggleLED(*msg);
	return;
}

static void changeChannel(void)
{
#ifdef FREQUENCY_AGILITY
	freqEntry_t freq;

	if (++sChannel >= NWK_FREQ_TBL_SIZE)
		sChannel = 0;

	DEBUG_PRINT("Channel %d\n", sChannel);

	freq.logicalChan = sChannel;
	SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
	BSP_TURN_OFF_LED1();
	BSP_TURN_OFF_LED2();
	sBlinky = 1;
#endif
}

/* implement auto-channel-change policy here... */
static void  checkChangeChannel(void)
{
#ifdef FREQUENCY_AGILITY
	int8_t dbm, inARow = 0;
	uint8_t i;

	memset(sSample, 0x0, SSIZE);
	for (i = 0; i < SSIZE; ++i)
	{
		/* quit if we need to service an app frame */
		if (sPeerFrameSem || sJoinSem)
			return;
		NWK_DELAY(1);
		SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, (void *)&dbm);
		sSample[i] = dbm;

		if (dbm > INTERFERNCE_THRESHOLD_DBM)
		{
			if (++inARow == IN_A_ROW)
			{
				changeChannel();
				break;
			}
		}
		else
			inARow = 0;
	}
#endif
}
