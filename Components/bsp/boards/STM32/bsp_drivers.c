/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32
 *            "STM32 Board"
 *   Top-level driver file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_driver_defs.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "bsp_config.h"

/**************************************************************************************************
 * @fn          BSP_InitDrivers
 *
 * @brief       Initialize all enabled BSP drivers.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_InitDrivers(void)
{
#if (!defined BSP_NO_LEDS)
	BSP_InitLeds();
#endif

#if (!defined BSP_NO_BUTTONS)
	BSP_InitButtons();
#endif

}

/* ================================================================================================
 *                                        C Code Includes
 * ================================================================================================
 */
#ifndef BSP_NO_LEDS
#include "drivers/code/bsp_leds.c"
#endif

#ifndef BSP_NO_BUTTONS
#include "drivers/code/bsp_buttons.c"
#endif


/**************************************************************************************************
*/

   
   
