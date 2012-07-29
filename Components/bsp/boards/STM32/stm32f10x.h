#ifndef __STM32_CUSTOM_H__
#define __STM32_CUSTOM_H__

#define USE_STDPERIPH_DRIVER

#if	defined (STM32F10X_LD) 		|| \
	defined (STM32F10X_LD_VL)	|| \
	defined (STM32F10X_MD) 		|| \
	defined (STM32F10X_MD_VL) 	|| \
	defined (STM32F10X_HD) 		|| \
	defined (STM32F10X_HD_VL) 	|| \
	defined (STM32F10X_XL) 		|| \
	defined (STM32F10X_CL)

	#define STM32F10X
#else
	#define STM32F10X_MD_VL
	#define STM32F10X
	#define STM32F_AUTO
#endif

#if		defined ( STM32F10X )
		#include "ST/STM32F10x/stm32f10x.h"
#elif	defined ( STM32F2XX )
		#include "ST/STM32F2xx/Include/stm32f10x.h"
#endif

#endif
