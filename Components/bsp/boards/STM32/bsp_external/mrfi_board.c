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

/**************************************************************************************************
 * @fn          createRandomAddress
 **************************************************************************************************
 */
void createRandomAddress(addr_t * lAddr)
{
	uint32_t dev_id = DBGMCU_GetDEVID();
	uint32_t rev_id = DBGMCU_GetREVID();
	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_GET, lAddr);
	lAddr->addr[NET_ADDR_SIZE - 1] = dev_id & 0x00FF;
	lAddr->addr[NET_ADDR_SIZE - 2] = (lAddr->addr[NET_ADDR_SIZE - 2] & 0xF0) | ((dev_id & 0x0F00) >> 8);
}

/**************************************************************************************************
 * @fn          MRFI_GpioPort1Isr
 **************************************************************************************************
 */
void MRFI_GDO0_INT_VECTOR(void);
BSP_ISR_FUNCTION( BSP_GpioPort1Isr, MRFI_GDO0_INT_VECTOR )
{
	/* This ISR is easily replaced.  The new ISR must simply
	 * include the following function call.
	 */
	MRFI_GpioIsr();
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
	while ((__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_RXNE) == RESET);
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

/**************************************************************************************************
 * @fn	Wait for SPI complete operation (not a BUSY)
 */
void __mrfi_SPI_WAIT_DONE(void)
{
	while (__mrfi_SPI_PORT__->SR & SPI_I2S_FLAG_BSY);
}

void __mrfi_GDO0_FALLING_EDGE_INT(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Set Falling edge */
	/* Connect EXTI7 Line to PB7 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);

	/* Configure EXTI7 line */
	EXTI_InitStructure.EXTI_Line = __mrfi_GDO0_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI9_5 Interrupt to lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void __mrfi_ENABLE_GDO0_INT(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = __mrfi_GDO0_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void __mrfi_DISABLE_GDO0_INT(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = __mrfi_GDO0_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
}
