/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32F10X
 *            "STM32F10X board"
 *   Board definition file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_BOARD_DEFS_H
#define BSP_BOARD_DEFS_H

#define BSP_BOARD_STM32
#include "mcus/bsp_stm32f10x_defs.h"
#include <stdio.h>

/* ------------------------------------------------------------------------------------------------
 *                                          Clock
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_config.h"
#define __bsp_CLOCK_MHZ__         BSP_CONFIG_CLOCK_MHZ

/* ------------------------------------------------------------------------------------------------
 *                                          Timer
 * ------------------------------------------------------------------------------------------------
 */
#ifdef FREQUENCY_HOPPING
	#ifndef NWK_PLL
		#define NWK_PLL
	#endif
#endif

#if defined NWK_PLL && !defined MRFI_TIMER_ALWAYS_ACTIVE
	#define MRFI_TIMER_ALWAYS_ACTIVE
#endif

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
	#define BSP_TIMER_A3 0x4133 // 'A' and '3' characters in hex
	#define BSP_TIMER_B7 0x4237 // 'B' and '7' characters in hex

	#ifndef BSP_TIMER_USED
		#define BSP_TIMER_USED BSP_TIMER_A3
	#else // if BSP_TIMER_USED was user defined
		#if BSP_TIMER_USED != BSP_TIMER_A3 && BSP_TIMER_USED != BSP_TIMER_B7
			#error "ERROR: The selected timer is invalid, must be BSP_TIMER_A3 or BSP_TIMER_B7."
		#endif
	#endif

	#define BSP_MAX_MODULATION_MAGNITUDE ( BSP_TIMER_CLK_KHZ * 10 / 100 ) // ten percent

	#define BSP_TIMER_PRESCALE_VALUE 0
	#define BSP_TIMER_PRESCALE_DIVISOR ( 1 << ( ( BSP_TIMER_PRESCALE_VALUE * 2 )\
								   + ( ( BSP_TIMER_PRESCALE_VALUE == 0 ) ? 0 : 1 ) ) )
	//  #define BSP_TIMER_CLK_KHZ ((BSP_CLOCK_MHZ * 2000L / BSP_TIMER_PRESCALE_DIVISOR + 1)/2)
	//  #define BSP_TIMER_CLK_KHZ 6500
	#ifndef BSP_CONFIG_CLOCK_KHZ
		#define BSP_TIMER_CLK_KHZ	((BSP_CLOCK_MHZ * 2000L / BSP_TIMER_PRESCALE_DIVISOR + 1) / 2)
	#else
		#define BSP_TIMER_CLK_KHZ	((BSP_CONFIG_CLOCK_KHZ * 2L / BSP_TIMER_PRESCALE_DIVISOR + 1) / 2)
	#endif

//	#define BSP_CALC_LIMIT( ticks )
//	#define BSP_ROLLOVER_LIMIT						BSP_CALC_LIMIT( BSP_TIMER_CLK_KHZ )
//	#define BSP_TIMER_FREE_RUN_INIT( )
//	#define BSP_TIMER_CHECK_OVERFLOW_FLAG( )
//	#define BSP_TIMER_CLEAR_OVERFLOW_FLAG( )      /* the event is cleared automatically */
//	#define BSP_TIMER_MAN_CLEAR_OVERFLOW_FLAG( )
//	#define BSP_TIMER_GET_TIMER_COUNT_VALUE_LO( )
//	#define BSP_TIMER_GET_TIMER_COUNT_VALUE_HI( )
//	#define BSP_TIMER_GET_TIMER_COUNT( p )
//	#define BSP_TIMER_SET_OVERFLOW_VALUE( val )

#endif // MRFI_TIMER_ALWAYS_ACTIVE

/* ------------------------------------------------------------------------------------------------
 *                                     Board Initialization
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_BOARD_C			"bsp_board.c"
#define BSP_INIT_BOARD()	BSP_InitBoard()
#define BSP_DELAY_USECS(x)	BSP_Delay(x)

void BSP_InitBoard(void);
void BSP_Delay(uint16_t usec);

void BSP_InitUSART(USART_TypeDef * port, uint32_t speed);
void BSP_SendUSART(USART_TypeDef * port, uint8_t data);
uint8_t BSP_RecvUSART(USART_TypeDef * port);

void BSP_InitSPI(SPI_TypeDef * port);

#ifdef BSP_DEBUG_PORT

	#if   defined ( __GNUC__ )
		/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#elif defined ( __CC_ARM )
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#else
		#define PUTCHAR_PROTOTYPE int putchar(int ch)
	#endif /* __GNUC__ */

	char __bsp_debug(char data);
	void __bsp_debug_msg(const char * m);
	void __bsp_debug_msg_ln(const char * m);
	void __bsp_debug_clear(void);
	void __bsp_debug_dec(int num);

	#define DEBUG(m)			__bsp_debug_msg(m)
	#define DEBUG_LN(m)			__bsp_debug_msg_ln(m)
	#define DEBUG_PRINT(m...)	printf(m)
	#define DEBUG_CLEAR()		__bsp_debug_clear()
	#define DEBUG_DEC(n)			__bsp_debug_dec(n)

#else

	#define DEBUG(m)
	#define DEBUG_LN(m)
	#define DEBUG_PRINT(m...)
	#define DEBUG_CLEAR()

#endif

void __bsp_ASSERT(const char * file_name, int line, const char * func_name);
#define BSP_ASSERT_HANDLER()	__bsp_ASSERT(__FILE__, __LINE__, __func__)

#endif
