/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board code file.
 *   Target : STM32
 *            "STM32 Board"
 *   Radios : CC1100, CC1101, CC2500
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */
#include "bsp.h"
#include "mrfi_board_defs.h"
#include "nwk_types.h"
#include "nwk_api.h"

/* ------------------------------------------------------------------------------------------------
 *                                      External Prototypes
 * ------------------------------------------------------------------------------------------------
 */
extern void MRFI_GpioIsr(void);
uint8_t mrfi_ignore_interrupt = 0;

/**************************************************************************************************
 * @fn          GDO0 Interrupt handler
 **************************************************************************************************
 */
BSP_ISR_FUNCTION( MRFI_GDO0_INT_HANDLER, MRFI_GDO0_INT_VECTOR )
{
	if (mrfi_ignore_interrupt)
	{
		if (MRFI_SYNC_PIN_INT_IS_ENABLED() && MRFI_SYNC_PIN_INT_FLAG_IS_SET())
		{
			MRFI_CLEAR_SYNC_PIN_INT_FLAG();
			MRFI_GDO0_INT_DISABLE();
			mrfi_ignore_interrupt = 0;
		}
	}
	else
	{
		/* This ISR is easily replaced.  The new ISR must simply
		 * include the following function call.
		 */
		DEBUG_LN("-- INT --");
		MRFI_GpioIsr();
	}
}

/**************************************************************************************************
 * @fn	Set CS to high
 */
void MRFI_SPI_DRIVE_CSN_HIGH(void)
{
	__mrfi_SPI_CSN_GPIO_PORT__->BSRR = BV(__mrfi_SPI_CSN_GPIO_BIT__);
}

/**************************************************************************************************
 * @fn	Set CS to low
 */
void MRFI_SPI_DRIVE_CSN_LOW(void)
{
	__mrfi_SPI_CSN_GPIO_PORT__->BRR  = BV(__mrfi_SPI_CSN_GPIO_BIT__);
}

/**************************************************************************************************
 * @fn	Read CS state
 */
uint16_t MRFI_SPI_CSN_IS_HIGH(void)
{
	return (__mrfi_SPI_CSN_GPIO_PORT__->ODR & BV(__mrfi_SPI_CSN_GPIO_BIT__));
}

/**************************************************************************************************
 * @fn	Read data from SPI
 */
uint8_t __mrfi_SPI_READ_BYTE(void)
{
	// Wait for data ready
	while (!(__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_RXNE));
	return (uint8_t)(__mrfi_SPI_PORT__->DR);
}

/**************************************************************************************************
 * @fn	Send data to SPI
 */
void __mrfi_SPI_WRITE_BYTE(uint8_t data)
{
	// Check for data available from previous operation
	while (__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_RXNE)
		__mrfi_SPI_PORT__->DR;
	// Check for transmitte buffer empty
	while (!(__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_TXE));
	__mrfi_SPI_PORT__->DR = data;
}

/*
 * @fn	Check for SPI initialized
 */
uint8_t __mrfi_SPI_IS_INITIALIZED(void)
{
	return (__mrfi_SPI_PORT__->CR1 & SPI_CR1_SPE) ? 1 : 0;
}

/*
 * @fn	Wait for SPI complete operation (not a BUSY)
 */
void __mrfi_SPI_WAIT_DONE(void)
{
	while (__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_BSY);
}

/*
 * @fn	Wait for SPI complete operation (not a BUSY)
 */
void __mrfi_GDO0_INIT(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Set Falling edge */
	/* Connect EXTI7 Line to PB7 pin */
	GPIO_EXTILineConfig(__mrfi_GDO0_EXTI_PORT, __mrfi_GDO0_EXTI_PIN);

	/* Configure EXTI7 line */
	EXTI_InitStructure.EXTI_Line = __mrfi_GDO0_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	MRFI_GDO0_INT_DISABLE();

	/* Enable and set EXTI9_5 Interrupt to highest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************************
 * @fn          createRandomAddress
 **************************************************************************************************
 */
void createRandomAddress(addr_t * lAddr)
{
	uint16_t * id = (uint16_t *)0x1FFFF7E8;
	uint16_t unique_id;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	CRC_ResetDR();
	CRC_CalcCRC(*id++);
	CRC_CalcCRC(*id++);
	CRC_CalcCRC(*id++);
	CRC_CalcCRC(*id++);
	CRC_CalcCRC(*id++);
	CRC_CalcCRC(*id++);
	unique_id = (uint16_t)CRC_GetCRC();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);

	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_GET, lAddr);
	lAddr->addr[NET_ADDR_SIZE - 1] = (unique_id & 0x00FF);
	lAddr->addr[NET_ADDR_SIZE - 2] = (unique_id & 0xFF00) >> 8;
}
