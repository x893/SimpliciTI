/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32
 *            "STM32 Board"
 *   Driver definition file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_DRIVER_DEFS_H
#define BSP_DRIVER_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                     Driver Initialization
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_DRIVERS_C               "bsp_drivers.c"
#define BSP_INIT_DRIVERS()          BSP_InitDrivers()

void BSP_InitDrivers(void);

#endif
