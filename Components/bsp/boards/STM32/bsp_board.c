/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32
 *            "STM32 Board"
 *   Top-level board code file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp.h"
#include "bsp_config.h"
#include <stddef.h>
#include "bsp_leds.h"

#define BSP_TIMER_CLK_MHZ   (BSP_CONFIG_CLOCK_MHZ_SELECT)
#ifdef STM32F_AUTO
	#warning "Use default STM32F10X_MD_VL processor. See bsp/boards/STM32/stm32f10x.h for details."
#endif


#if defined(SW_TIMER)
	static uint8_t sIterationsPerUsec = 0;
#endif

/**************************************************************************************************
 * @fn          BSP_EARLY_INIT
 */
void __bsp_ASSERT(const char * file_name, int line, const char * func_name)
{
	volatile uint32_t delay;
	volatile uint8_t blinks = 8;

	BSP_TURN_ON_LED2();
	BSP_TURN_OFF_LED1();

	DEBUG("ASSERT:");
	DEBUG(file_name);
	DEBUG(" (");
	DEBUG_DEC(line);
	DEBUG_LN(")");
	DEBUG_LN(func_name);
	DEBUG_LN("RESET ...");

	while(blinks--)
	{
		for(delay = 0; delay < SystemCoreClock / 100; delay++)
		{
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
		BSP_TOGGLE_LED1();
	}
	BSP_TURN_OFF_LED2();

	for(delay = 0; delay < SystemCoreClock / 10; delay++)
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}

	NVIC_SystemReset();
}

/**************************************************************************************************
 * @fn			BSP_EARLY_INIT
 *
 * @brief		This function is called by start-up code before doing the normal initialization
 *				of data segments. If the return value is zero, initialization is not performed.
 *				The global macro label "BSP_EARLY_INIT" gets #defined in the bsp_stm32_defs.h
 *				file, according to the specific compiler environment (ARM or IAR). In the IAR
 *				environment
 *              this macro invokes "__low_level_init()".
 *
 * @param       None
 *
 * @return      0 - don't intialize data segments / 1 - do initialization
 **************************************************************************************************
*/
BSP_EARLY_INIT(void)
{
	/* Return 1 - run seg_init */
	return (1);
}

/**************************************************************************************************
 * @fn          BSP_InitBoard
 *
 * @brief       Initialize the board.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_InitBoard(void)
{
	DBGMCU_Config( DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);

	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA	|
			RCC_APB2Periph_GPIOB	|
			RCC_APB2Periph_GPIOC	|
			RCC_APB2Periph_SPI1		|
			RCC_APB2Periph_AFIO, ENABLE);

#if (defined BSP_DEBUG_PORT)
	BSP_InitUSART(BSP_DEBUG_PORT, BSP_DEBUG_SPEED);
#endif

#if defined(SW_TIMER)
	#define MHZ_CLOCKS_PER_USEC			BSP_CLOCK_MHZ
	#define MHZ_CLOCKS_PER_ITERATION	1

	sIterationsPerUsec = (uint8_t)(((MHZ_CLOCKS_PER_USEC) / (MHZ_CLOCKS_PER_ITERATION)) + .5);

	if (!sIterationsPerUsec)
		sIterationsPerUsec = 1;
#endif   /* SW_TIMER */
}

/**************************************************************************************************
 * @fn          BSP_Delay
 *
 * @brief       Sleep for the requested amount of time.
 *
 * @param       # of microseconds to sleep.
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_Delay(uint16_t usec)
{
#if !defined(SW_TIMER)

	/* Start the timer in UP mode */
	/* Loop till compare interrupt flag is set */
	/* Stop the timer */
	/* Clear the interrupt flag */

#else   /* !SW_TIMER */

	/* Declared 'volatile' in case User optimizes for speed. This will
	 * prevent the optimizer from eliminating the loop completely. But
	 * it also generates more code...
	 */
	volatile uint16_t repeatCount = (sIterationsPerUsec * usec) / 2;
	while (repeatCount--);

#endif  /* !SW_TIMER */
}

#if defined ( BSP_DEBUG_PORT )
char __bsp_debug(char data)
{
	while(USART_GetFlagStatus(BSP_DEBUG_PORT, USART_FLAG_TXE) == RESET);
	USART_SendData(BSP_DEBUG_PORT, data);
	return data;
}

void __bsp_debug_msg(const char * m)
{
	while (*m != 0)
		__bsp_debug(*m++);
}
void __bsp_debug_msg_ln(const char * m)
{
	__bsp_debug_msg(m);
	__bsp_debug_msg("\r\n");
}

void __bsp_debug_hex_1(uint8_t num)
{
	char c;
	c = num & 0xF;
	if (c <= 9)
		c += '0';
	else
		c += ('A' - 10);
	__bsp_debug(c);
}

void __bsp_debug_hex(uint8_t num)
{
	__bsp_debug_hex_1(num >> 4);
	__bsp_debug_hex_1(num);
}

void __bsp_debug_hex_ln(uint8_t num)
{
	__bsp_debug_hex(num);
	__bsp_debug_msg("\r\n");
}

void __bsp_debug_dec(int num)
{
	int m = 10000;
	char ch = 0;

	if (num < 0)
	{
		__bsp_debug('-');
		num = -num;
	}
	if (num == 0)
		__bsp_debug('0');
	else
	{
		while (m != 0)
		{
			if (num < m)
			{
				if (ch != 0)
					__bsp_debug('0');
			}
			else
			{
				ch = (char)(num / m) + '0';
				__bsp_debug(ch);
				num %= m;
			}
			m /= 10;
		}
	}
}

#endif

void BSP_InitUSART(USART_TypeDef * port, uint32_t speed)
{
	USART_InitTypeDef	USART_InitStructure;

	if (port == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else if (port == USART2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	else if (port == USART3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	__bsp_PIN_CONFIG__ (BSP_DEBUG_TX_PIN, PIN_MODE_AF_50 );
	__bsp_PIN_CONFIG__ (BSP_DEBUG_RX_PIN, PIN_MODE_IN );

	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(port, &USART_InitStructure);
	USART_Cmd(port, ENABLE);
}

void BSP_SendUSART(USART_TypeDef * port, uint8_t data)
{
    /* Loop until USARTy DR register is empty */ 
    while(USART_GetFlagStatus(port, USART_FLAG_TXE) == RESET) { }
		USART_SendData(port, data);
}

uint8_t BSP_RecvUSART(USART_TypeDef * port)
{
    /* Loop until the USARTz Receive Data Register is not empty */
    while(USART_GetFlagStatus(port, USART_FLAG_RXNE) == RESET) { }
	return USART_ReceiveData(port);
}

/**
  * @brief  Initialize SPI channel
  * @param  None
  * @retval None
  */
void BSP_InitSPI(SPI_TypeDef * port)
{
	SPI_InitTypeDef   SPI_InitStructure;
	if (port == SPI1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	else if (port == SPI2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	else
		BSP_ASSERT(0);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0;
	SPI_Init(port, &SPI_InitStructure);
	SPI_Cmd(port, ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	return __bsp_debug(ch);
}

GPIO_TypeDef *	gpioPort	(uint8_t pin);
uint16_t		gpioPinMask	(uint8_t pin);

#if	defined (STM32F10X)

/*
 *	Return pointer to GPIO for pin
 *
 */
GPIO_TypeDef * gpioPort(uint8_t pin)
{
	return (GPIO_TypeDef * )(GPIOA_BASE + ((pin & 0xF0) << 6));
}
/*
 *	Return bit mask for pin
 *
 */
uint16_t gpioPinMask(uint8_t pin)
{
	return (uint16_t)(1 << (pin & 0x0F));
}
/*
 *	Set pin to HIGH
 *
 */
void gpioPinHigh(uint8_t pin)
{
	gpioPort(pin)->BSRR = gpioPinMask(pin);
}
/**********************************************************************
 *	Set pin to LOW
 *
 *********************************************************************/
void gpioPinLow(uint8_t pin)
{
	gpioPort(pin)->BRR  = gpioPinMask(pin);
}
/*
 *	Return output state for pin
 *
 */
uint8_t gpioPinOut(uint8_t pin)
{
	return (gpioPort(pin)->ODR & gpioPinMask(pin) ? 1 : 0);
}
/*
 *	Return input state for pin
 *
 */
uint8_t gpioPinIn(uint8_t pin)
{
	return (gpioPort(pin)->IDR & gpioPinMask(pin) ? 1 : 0);
}
/*
 *	Set pin mode
 *	low byte	pin
 *	high byte	mode for pin
 */
void gpioPinConfig(uint16_t mode_pin)
{
	uint32_t * port =
		((uint32_t *)gpioPort(mode_pin))
		+ (	(mode_pin & 0x08)
			? offsetof(GPIO_TypeDef,CRH) / sizeof(uint32_t)
			: offsetof(GPIO_TypeDef,CRL) / sizeof(uint32_t)
			);

	uint8_t m_pos = (mode_pin & 0x07) << 2;
	*port = (*port & ~(0x0Ful << m_pos)) | (((mode_pin & 0x0F00) >> 8) << m_pos);
}
#endif
