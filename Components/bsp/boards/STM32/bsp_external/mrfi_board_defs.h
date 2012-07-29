/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board definition file.
 *   Target : STM32
 *            "STM32 Board"
 *   Radios : CC2500, CC1100, CC1101
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_BOARD_DEFS_H
#define MRFI_BOARD_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp.h"
#include "pp_utils.h"
#include "bsp_config.h"
#include "bsp_driver_defs.h"
#include "nwk_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                        Radio Selection
 * ------------------------------------------------------------------------------------------------
 */
#if (!defined MRFI_CC1100)		&&	\
    (!defined MRFI_CC1101)		&&	\
    (!defined MRFI_CC1100E_470)	&&	\
    (!defined MRFI_CC1100E_950)	&&	\
    (!defined MRFI_CC2500)		&&	\
    (!defined MRFI_CC2420)
	#error "ERROR: A compatible radio must be specified for the STM32 board."
#endif

void createRandomAddress(addr_t * lAddr);

/* ------------------------------------------------------------------------------------------------
 *                                      GDO0 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_GDO0_PIN__						PB7
#define __mrfi_GDO0_BIT__						(__mrfi_GDO0_PIN__ & 0x0F)
#define __mrfi_GDO0_PORT__						GPIOB
#define __mrfi_GDO0_DDR__
#define __mrfi_GDO0_EXTI_LINE					EXTI_Line7

#define MRFI_CONFIG_GDO0_PIN_AS_INPUT()			__bsp_PIN_CONFIG__	( __mrfi_GDO0_PIN__, PIN_MODE_IN )
#define MRFI_GDO0_PIN_IS_HIGH()					( __bsp_PIN_IN__	( __mrfi_GDO0_PIN__ ) )

#define MRFI_GDO0_INT_VECTOR					EXTI9_5_IRQHandler

void __mrfi_ENABLE_GDO0_INT(void);
void __mrfi_DISABLE_GDO0_INT(void);
#define MRFI_ENABLE_GDO0_INT()					__mrfi_ENABLE_GDO0_INT()
#define MRFI_DISABLE_GDO0_INT()					__mrfi_DISABLE_GDO0_INT()

#define MRFI_GDO0_INT_IS_ENABLED()				(EXTI->IMR & __mrfi_GDO0_EXTI_LINE)

#define MRFI_CLEAR_GDO0_INT_FLAG()				EXTI_ClearITPendingBit(__mrfi_GDO0_EXTI_LINE)
#define MRFI_GDO0_INT_FLAG_IS_SET()				(EXTI_GetFlagStatus(__mrfi_GDO0_EXTI_LINE) != RESET)

void __mrfi_GDO0_FALLING_EDGE_INT(void);
#define MRFI_CONFIG_GDO0_FALLING_EDGE_INT()		__mrfi_GDO0_FALLING_EDGE_INT()
// #define MRFI_CONFIG_GDO0_RISING_EDGE_INT()		st ( /* Set Rising edge */; )

#define MRFI_GDO0_SYNC_INT_ENABLE(s)			st ( if (s != true) { MRFI_ENABLE_GDO0_INT();  } )
#define MRFI_GDO0_SYNC_INT_DISABLE(s)			st ( if (s != true) { MRFI_DISABLE_GDO0_INT(); } )

/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_SPI_PORT__						SPI1

/* CSn Pin Configuration */
#define __mrfi_SPI_CSN_PIN__					PA4
#define __mrfi_SPI_CSN_GPIO_BIT__				( __mrfi_SPI_CSN_PIN__ & 0x0F )
#define __mrfi_SPI_CSN_GPIO_PORT__				GPIOA
#define __mrfi_SPI_CSN_GPIO_DDR__
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()		__bsp_PIN_CONFIG__	( __mrfi_SPI_CSN_PIN__,  PIN_MODE_OUT_PP_50 )

void MRFI_SPI_DRIVE_CSN_HIGH(void);
void MRFI_SPI_DRIVE_CSN_LOW(void);
uint16_t MRFI_SPI_CSN_IS_HIGH(void);

/* SCLK Pin Configuration */
#define __mrfi_SPI_SCLK_PIN__					PA5
#define __mrfi_SPI_SCLK_GPIO_BIT__				5
#define __mrfi_SPI_SCLK_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SCLK_GPIO_DDR__
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()	__bsp_PIN_CONFIG__	( __mrfi_SPI_SCLK_PIN__, PIN_MODE_AF_50 )

/* SI Pin Configuration */
#define __mrfi_SPI_SI_PIN__						PA7
#define __mrfi_SPI_SI_GPIO_BIT__				( __mrfi_SPI_SI_PIN__ & 0x0F )
#define __mrfi_SPI_SI_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SI_GPIO_DDR__
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()		__bsp_PIN_CONFIG__	( __mrfi_SPI_SI_PIN__, PIN_MODE_AF_50 )

/* SO Pin Configuration */
#define __mrfi_SPI_SO_PIN__						PA6
#define __mrfi_SPI_SO_GPIO_BIT__				( __mrfi_SPI_SO_PIN__ & 0x0F )
#define __mrfi_SPI_SO_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SO_GPIO_DDR__
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()		__bsp_PIN_CONFIG__	( __mrfi_SPI_SO_PIN__, PIN_MODE_IN )
#define MRFI_SPI_SO_IS_HIGH()					( __bsp_PIN_IN__	( __mrfi_SPI_SO_PIN__ ))

/* read/write macros */
uint8_t	__mrfi_SPI_READ_BYTE(void);
void	__mrfi_SPI_WRITE_BYTE(uint8_t data);
void	__mrfi_SPI_WAIT_DONE(void);
#define MRFI_SPI_WRITE_BYTE(x)					__mrfi_SPI_WRITE_BYTE(x)
#define MRFI_SPI_READ_BYTE()					__mrfi_SPI_READ_BYTE()
#define MRFI_SPI_WAIT_DONE()					__mrfi_SPI_WAIT_DONE()

/* SPI critical section macros */
typedef bspIState_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)	    BSP_ENTER_CRITICAL_SECTION(x)
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)		BSP_EXIT_CRITICAL_SECTION(x)

/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  10 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */

/* initialization macro */
#define MRFI_SPI_INIT()				BSP_InitSPI( __mrfi_SPI_PORT__ );
#define MRFI_SPI_IS_INITIALIZED()	(__mrfi_SPI_PORT__->CR1 & SPI_CR1_SPE)

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#ifndef BSP_BOARD_STM32
	#error "ERROR: Mismatch between specified board and MRFI configuration."
#endif

#endif
