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
    (!defined MRFI_CC110L)		&&	\
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
#define __mrfi_GDO0_PORT__						GPIOB
#define __mrfi_GDO0_BIT__						(__mrfi_GDO0_PIN__ & 0xF)
#define __mrfi_GDO0_DDR__
#define __mrfi_GDO0_EXTI_LINE					EXTI_Line7
#define __mrfi_GDO0_EXTI_PIN					GPIO_PinSource7
#define __mrfi_GDO0_EXTI_PORT					GPIO_PortSourceGPIOB

#define MRFI_CONFIG_GDO0_PIN_AS_INPUT()			__bsp_PIN_CONFIG__	( __mrfi_GDO0_PIN__, PIN_MODE_IN )
#define MRFI_GDO0_PIN_IS_HIGH()					(__mrfi_GDO0_PORT__->IDR & BV(__mrfi_GDO0_BIT__) ? 1: 0)
#define MRFI_GDO0_PIN_IS_LOW()					(__mrfi_GDO0_PORT__->IDR & BV(__mrfi_GDO0_BIT__) ? 0: 1)

#define MRFI_GDO0_INT_VECTOR					EXTI9_5_IRQHandler
#define MRFI_GDO0_INT_HANDLER					EXTI9_5_IRQHandler

#define MRFI_GDO0_INT_DISABLE()					EXTI->IMR &= ~__mrfi_GDO0_EXTI_LINE
#define MRFI_GDO0_INT_ENABLE()					EXTI->IMR |=  __mrfi_GDO0_EXTI_LINE
#define MRFI_GDO0_SYNC_INT_ENABLE(s)			if (s != true) MRFI_GDO0_INT_ENABLE()
#define MRFI_GDO0_SYNC_INT_DISABLE(s)			if (s != true) MRFI_GDO0_INT_DISABLE()

#define MRFI_GDO0_INT_IS_ENABLED()				(EXTI->IMR & __mrfi_GDO0_EXTI_LINE)

#define MRFI_CLEAR_GDO0_INT_FLAG()				EXTI->PR = __mrfi_GDO0_EXTI_LINE
#define MRFI_GDO0_INT_FLAG_IS_SET()				(EXTI->PR & __mrfi_GDO0_EXTI_LINE)

void __mrfi_GDO0_INIT(void);
#define MRFI_CONFIG_GDO0_FALLING_EDGE_INT()		__mrfi_GDO0_INIT()

/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_SPI_PORT__						SPI1

/* CSn Pin Configuration */
void MRFI_SPI_DRIVE_CSN_HIGH(void);
void MRFI_SPI_DRIVE_CSN_LOW(void);
uint16_t MRFI_SPI_CSN_IS_HIGH(void);

#define __mrfi_SPI_CSN_PIN__					PA4
#define __mrfi_SPI_CSN_GPIO_BIT__				( __mrfi_SPI_CSN_PIN__ & 0x0F )
#define __mrfi_SPI_CSN_GPIO_PORT__				GPIOA
#define __mrfi_SPI_CSN_GPIO_DDR__
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()		MRFI_SPI_DRIVE_CSN_HIGH();	\
												__bsp_PIN_CONFIG__	( __mrfi_SPI_CSN_PIN__,  PIN_MODE_OUT_PP_50 )

/* SCLK Pin Configuration */
#define __mrfi_SPI_SCLK_PIN__					PA5
#define __mrfi_SPI_SCLK_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SCLK_GPIO_BIT__				( __mrfi_SPI_SCLK_PIN__ & 0x0F )
#define __mrfi_SPI_SCLK_GPIO_DDR__
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()	__bsp_PIN_CONFIG__	( __mrfi_SPI_SCLK_PIN__, PIN_MODE_AF_50 )

/* SO Pin Configuration */
#define __mrfi_SPI_SO_PIN__						PA6
#define __mrfi_SPI_SO_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SO_GPIO_BIT__				( __mrfi_SPI_SO_PIN__ & 0x0F )
#define __mrfi_SPI_SO_GPIO_DDR__
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()		__bsp_PIN_CONFIG__	( __mrfi_SPI_SO_PIN__, PIN_MODE_IN )
#define MRFI_SPI_SO_IS_HIGH()					( __bsp_PIN_IN__	( __mrfi_SPI_SO_PIN__ ))

/* SI Pin Configuration */
#define __mrfi_SPI_SI_PIN__						PA7
#define __mrfi_SPI_SI_GPIO_PORT__				GPIOA
#define __mrfi_SPI_SI_GPIO_BIT__				( __mrfi_SPI_SI_PIN__ & 0x0F )
#define __mrfi_SPI_SI_GPIO_DDR__
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()		__bsp_PIN_CONFIG__	( __mrfi_SPI_SI_PIN__, PIN_MODE_AF_50 )

/* read/write macros */
uint8_t	__mrfi_SPI_READ_BYTE(void);
void	__mrfi_SPI_WRITE_BYTE(uint8_t data);
void	__mrfi_SPI_WAIT_DONE(void);
uint8_t	__mrfi_SPI_IS_INITIALIZED(void);

#define MRFI_SPI_WRITE_BYTE(x)					__mrfi_SPI_WRITE_BYTE(x)
#define MRFI_SPI_READ_BYTE()					__mrfi_SPI_READ_BYTE()
#define MRFI_SPI_WAIT_DONE()					__mrfi_SPI_WAIT_DONE()
#define MRFI_SPI_IS_INITIALIZED()				__mrfi_SPI_IS_INITIALIZED()

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

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#ifndef BSP_BOARD_STM32
	#error "ERROR: Mismatch between specified board and MRFI configuration."
#endif

#endif
