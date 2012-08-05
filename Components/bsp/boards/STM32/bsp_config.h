/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : STM32
 *            "STM32 board"
 *   Board configuration file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#define DEBUG_SPI

#define BSP_DEBUG_PORT			USART2
#define BSP_DEBUG_SPEED			115200
#define BSP_DEBUG_TX_PIN		PA2
#define BSP_DEBUG_RX_PIN		PA3

#include "Configuration/smpl_nwk_config.h"

#if   defined ( CFG_ACCESS_POINT )
	#include "Configuration/Access_Point/smpl_config.h"

#elif defined ( CFG_CHANNEL_SNIFFER )
	#include "Configuration/Channel_Sniffer/smpl_config.h"

#elif defined ( CFG_END_DEVICE )
	#include "Configuration/End_Device/smpl_config.h"

#elif defined ( CFG_RANGE_EXTENDER )
	#include "Configuration/Range_Extender/smpl_config.h"

#else
	#error "Configuration not define"
#endif

#endif
