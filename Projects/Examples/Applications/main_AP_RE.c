#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_pll.h"

/* For FHSS systems, calls to NWK_DELAY() will also call nwk_pllBackgrounder()
 * during the delay time so if you use the system delay mechanism in a loop,
 * you don't need to also call the nwk_pllBackgrounder() function.
 */
#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)

void toggleLED(uint8_t);

int main (void)
{
	BSP_Init();

	/* On FHSS systems the call to SMPL_Init will not return until we have
	 * locked onto a reference clock.  Also, on return the radio will always
	 * be on in receive mode (at least for now).
	 */

	while (SMPL_SUCCESS != SMPL_Init((uint8_t (*)(linkID_t))0))
	{
		toggleLED(1);
		toggleLED(2);
		SPIN_ABOUT_A_SECOND;
	}

	toggleLED(1);
	toggleLED(2);

#ifdef ACCESS_POINT
	/* This code example changes the Link token to be distributed to those who
	* Join. For the example here this should be done before anyone joins so
	* the Join context is defaulted to OFF for this scenario. See the
	* smpl_config.dat file. After the link token is set the Join context must
	* be enabled.
	*
	* NOTE that this is done after initialization. For APs the init sequence
	* consists of a step in which a link token is generated. The sequence here
	* overrides that setting. It can be used to distribute different link tokens
	* to different devices. The sequence here is a simple example of how to use
	* the IOCTL interface to set the Link token for subsequent Joiners.
	*
	* You might want to be careful about following this particular example if you
	* are restoring from NV unless you are setting a fixed value as is done here.
	* Unconditionally setting a random value will make it esentially impossible
	* for newly joining devices to link to devices that joined before the AP was
	* reset since they will have different link tokens.
	*/
	{
		ioctlToken_t t;

		t.tokenType       = TT_LINK;
		t.token.linkToken = 0x78563412;

		SMPL_Ioctl(IOCTL_OBJ_TOKEN, IOCTL_ACT_SET, &t);

		/* enable join context */
		SMPL_Ioctl(IOCTL_OBJ_AP_JOIN, IOCTL_ACT_ON, 0);
	}
#endif

	while (1)
		FHSS_ACTIVE( nwk_pllBackgrounder( false ) ); /* manage FHSS */
}

void toggleLED(uint8_t which)
{
	if (1 == which)			{	BSP_TOGGLE_LED1();	}
	else if (2 == which)	{	BSP_TOGGLE_LED2();	}
}
