/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   MCU : STM32 family
 *   Microcontroller definition file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_STM32_DEFS_H
#define BSP_STM32_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_MCU_STM32

#include "stm32f10x.h"
#undef READ_BIT

#define PA0		0x00
#define PA1		0x01
#define PA2		0x02
#define PA3		0x03
#define PA4		0x04
#define PA5		0x05
#define PA6		0x06
#define PA7		0x07
#define PA8		0x08
#define PA9		0x09
#define PA10	0x0A
#define PA11	0x0B
#define PA12	0x0C
#define PA13	0x0D
#define PA14	0x0E
#define PA15	0x0F

#define PB0		0x10
#define PB1		0x11
#define PB2		0x12
#define PB3		0x13
#define PB4		0x14
#define PB5		0x15
#define PB6		0x16
#define PB7		0x17
#define PB8		0x18
#define PB9		0x19
#define PB10	0x1A
#define PB11	0x1B
#define PB12	0x1C
#define PB13	0x1D
#define PB14	0x1E
#define PB15	0x1F

#define PC0		0x20
#define PC1		0x21
#define PC2		0x22
#define PC3		0x23
#define PC4		0x24
#define PC5		0x25
#define PC6		0x26
#define PC7		0x27
#define PC8		0x28
#define PC9		0x29
#define PC10	0x2A
#define PC11	0x2B
#define PC12	0x2C
#define PC13	0x2D
#define PC14	0x2E
#define PC15	0x2F

void			gpioPinHigh	(uint8_t pin);
void			gpioPinLow	(uint8_t pin);
uint8_t			gpioPinOut	(uint8_t pin);
uint8_t			gpioPinIn	(uint8_t pin);
void			gpioPinConfig(uint16_t mode_pin);

#define __bsp_PIN_LOW__(pin)			gpioPinLow	(pin)
#define __bsp_PIN_HIGH__(pin)			gpioPinHigh	(pin)
#define __bsp_PIN_OUT__(pin)			gpioPinOut	(pin)
#define __bsp_PIN_IN__(pin)				gpioPinIn	(pin)
#define __bsp_PIN_CONFIG__(pin,mode)	gpioPinConfig	((mode << 8) | pin)

#define PIN_MODE_AF_50		(((uint32_t)GPIO_Mode_AF_PP | (uint32_t)GPIO_Speed_50MHz))
#define PIN_MODE_IN			((uint32_t)GPIO_Mode_IN_FLOATING)
#define PIN_MODE_OUT_PP_50	(((uint32_t)GPIO_Mode_Out_PP | (uint32_t)GPIO_Speed_50MHz))
#define PIN_MODE_OUT_PP_10	(((uint32_t)GPIO_Mode_Out_PP | (uint32_t)GPIO_Speed_10MHz))
#define PIN_MODE_OUT_PP_2	(((uint32_t)GPIO_Mode_Out_PP | (uint32_t)GPIO_Speed_2MHz))

#define PORT_PIN_SET(p,b)	p->BSRR = BV(b)
#define PORT_PIN_CLR(p,b)	p->BRR  = BV(b)
#define PORT_PIN_IN(p,b)	( p->IDR & BV(b) )
#define PORT_PIN_STATE(p,b)	( p->ODR & BV(b) )

/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

#ifdef __IAR_SYSTEMS_ICC__
	/* ---------------------- IAR Compiler ---------------------- */
	#define BSP_COMPILER_IAR

	#define __bsp_ISTATE_T__			__istate_t
	#define __bsp_ISR_FUNCTION__(f,v)	void v(void)

	/* Initialization call provided in IAR environment before standard C-startup */
	#include <intrinsics.h>
	__intrinsic int __low_level_init(void);
	#define BSP_EARLY_INIT(void) __intrinsic int __low_level_init(void)

	#define __bsp_QUOTED_PRAGMA__(x)			_Pragma(#x)
	#define __bsp_GET_ISTATE__()				__get_interrupt_state()
	#define __bsp_RESTORE_ISTATE__(x)			__set_interrupt_state(x)
	#define __bsp_INTERRUPTS_ARE_ENABLED__()	__get_interrupt_state()

#elif (defined __CC_ARM )
	/* ---------------------- ARM--------------------------------- */
	#define BSP_COMPILER_ARM

	#include <stm32f10x.h>

	#define __interrupt
	#define __bsp_QUOTED_PRAGMA__(x)
	#define __bsp_ISTATE_T__	int
	#define __bsp_ISR_FUNCTION__(f,v)			__bsp_QUOTED_PRAGMA__(vector=v) __interrupt void f(void)
	#define BSP_EARLY_INIT(void)				int _system_pre_init(void)
	#define __bsp_GET_ISTATE__()				__disable_irq()
	#define __bsp_RESTORE_ISTATE__(x)			if (!x) __enable_irq();
	#define __bsp_INTERRUPTS_ARE_ENABLED__()	0

/* ------------------ Unrecognized Compiler ------------------ */
#else
	#error "ERROR: Unknown compiler."
#endif

#if (defined BSP_COMPILER_IAR) || (defined BSP_COMPILER_ARM)

	#define __bsp_ENABLE_INTERRUPTS__()			__enable_irq()
	#define __bsp_DISABLE_INTERRUPTS__()		__disable_irq()

#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Common
 * ------------------------------------------------------------------------------------------------
 */
#define __bsp_LITTLE_ENDIAN__   1
#define __bsp_CODE_MEMSPACE__   /* blank */
#define __bsp_XDATA_MEMSPACE__  /* blank */

#ifndef NULL
	#define NULL 0
#endif

#endif
