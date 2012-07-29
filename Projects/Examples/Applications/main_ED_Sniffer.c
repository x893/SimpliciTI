#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"

/*         SANITY CHECKS                   */
#if defined( BSP_BOARD_RFUSB )
#error ERROR: RFUSB does not support default channel sniffer build (has only 1 LED)
#endif

#if ( NUM_CONNECTIONS > 0 )
	#error ERROR: NUM_CONNECTIONS must be 0 when building channel sniffer
#endif

#ifndef FREQUENCY_AGILITY
#error ERROR: FREQUENCY_AGILITY must be defined when building channel sniffer
#endif
/*         END SANITY CHECKS                   */

#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)
#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

static void toggleLED(uint8_t);

int main (void)
{
	ioctlScanChan_t scan;
	freqEntry_t     freq[NWK_FREQ_TBL_SIZE];
	uint8_t         firstTimeThru = 1;

	BSP_Init();
	
	/* Keep trying to join (a side effect of successful initialization) until
	* successful. Toggle LEDS to indicate that joining has not occurred.
	*/
	while (SMPL_SUCCESS != SMPL_Init(0))
	{
		toggleLED(1);
		toggleLED(2);
		SPIN_ABOUT_A_SECOND;
	}

	scan.freq = freq;
	while (1)
	{
		SPIN_ABOUT_A_QUARTER_SECOND;

		SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SCAN, &scan);
		if (1 == scan.numChan)
		{
			if (firstTimeThru)
			{
				BSP_TURN_OFF_LED1();
				BSP_TURN_ON_LED2();
				{
					uint8_t i = 15;
					while (i--)
					{
						toggleLED(1);
						toggleLED(2);
						SPIN_ABOUT_A_QUARTER_SECOND;
					}
				}
				firstTimeThru = 0;
			}
			switch(freq[0].logicalChan)
			{
			case 0:
				/* GREEN OFF */
				/* RED   OFF */
				BSP_TURN_OFF_LED1();
				BSP_TURN_OFF_LED2();
				break;
	
			case 1:
				/* GREEN OFF */
				/* RED   ON */
				BSP_TURN_OFF_LED1();
				BSP_TURN_ON_LED2();
				break;
	
			case 2:
				/* GREEN ON */
				/* RED   OFF */
				BSP_TURN_ON_LED1();
				BSP_TURN_OFF_LED2();
				break;
	
			case 3:
				/* GREEN ON */
				/* RED   ON */
				BSP_TURN_ON_LED1();
				BSP_TURN_ON_LED2();
				break;
	
			case 4:
				/* blink them both... */
				BSP_TURN_OFF_LED1();
				BSP_TURN_OFF_LED2();
				SPIN_ABOUT_A_QUARTER_SECOND;
				BSP_TURN_ON_LED1();
				BSP_TURN_ON_LED2();
				SPIN_ABOUT_A_QUARTER_SECOND;
				BSP_TURN_OFF_LED1();
				BSP_TURN_OFF_LED2();
			}
		}
	}
}

static void toggleLED(uint8_t which)
{
	if (1 == which)			{	BSP_TOGGLE_LED1();	}
	else if (2 == which)	{	BSP_TOGGLE_LED2();	}
	return;
}
