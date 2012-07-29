/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32
 *            "STM32 Board"
 *   LED definition file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_LED_DEFS_H
#define BSP_LED_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_board_defs.h"


/* ------------------------------------------------------------------------------------------------
 *                                      Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __bsp_NUM_LEDS__               0
#define __bsp_LED_BLINK_LOOP_COUNT__   0x34000


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #1 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  LED1
 *   Color     :  Green
 *   Polarity  :  Active High
 *   GPIO      :  PC8
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/*
#define __bsp_LED1_PIN__			PB0
#define __bsp_LED1_BIT__            (__bsp_LED1_PIN__ & 0xF)
#define __bsp_LED1_PORT__           GPIOB->ODR
#define __bsp_LED1_DDR__
#define __bsp_LED1_MODE__			((uint32_t)GPIO_Mode_Out_PP | (uint32_t)GPIO_Speed_2MHz)
#define __bsp_LED1_IS_ACTIVE_LOW__  0
*/

/* ------------------------------------------------------------------------------------------------
 *                                 Include Generic LED Macros
 * ------------------------------------------------------------------------------------------------
 */
#include "code/bsp_generic_leds.h"

/*
#undef  __bsp_LED1_CONFIG__
#undef  __bsp_LED2_CONFIG__
#define __bsp_LED1_CONFIG__()	__bsp_PIN_CONFIG__  ( __bsp_LED1_PIN__, __bsp_LED1_MODE__ )
#define __bsp_LED2_CONFIG__()	__bsp_PIN_CONFIG__  ( __bsp_LED2_PIN__, __bsp_LED2_MODE__ )
*/

#endif
