#ifndef __STM32_CUSTOM_H__
#define __STM32_CUSTOM_H__

#define USE_STDPERIPH_DRIVER

#if defined(MapleRET6)

	#define STM32F10X_HD
	#define BSP_CONFIG_CLOCK_MHZ	72ul
	#define SYSCLK_FREQ_72MHz		(BSP_CONFIG_CLOCK_MHZ * 1000000ul)

#elif defined(STM32VLDISCOVERY)

	#define STM32F10X_MD_VL
	#define BSP_CONFIG_CLOCK_MHZ	24ul
	#define SYSCLK_FREQ_24MHz		(BSP_CONFIG_CLOCK_MHZ * 1000000ul)

#else
	#error "CPU type not define."
#endif

#if !defined (STM32F2XX) && \
	!defined (STM32F4XX)

	#define STM32F10X

	#if	!defined (STM32F10X_LD) 	&& \
		!defined (STM32F10X_LD_VL)	&& \
		!defined (STM32F10X_MD) 	&& \
		!defined (STM32F10X_MD_VL) 	&& \
		!defined (STM32F10X_HD) 	&& \
		!defined (STM32F10X_HD_VL) 	&& \
		!defined (STM32F10X_XL) 	&& \
		!defined (STM32F10X_CL)		&& \
		!defined (STM32F2XX)		&& \
		!defined (STM32F4XX)

		#define STM32F10X_HD

	#endif
#endif

#if		defined ( STM32F10X )
		#include "ST/STM32F10x/stm32f10x.h"

#elif	defined ( STM32F2XX )
		#include "ST/STM32F2xx/Include/stm32f10x.h"

#endif

#endif
