/**
 ******************************************************************************
 * @file    stm32_misc.c
 * @author  iclm team
 * @brief   misc services
 ******************************************************************************
 */
#include "stm32_misc.h"
#include "utilities.h"

void CACHE_Enable(void) {
#if defined(STM32H750xx) || defined(STM32H743xx)
	SCB_EnableICache();
	SCB_EnableDCache();
#endif
}

void CACHE_InvalDCache(uint32_t *addr, int32_t dsize) {
#if defined(STM32H750xx) || defined(STM32H743xx)
	if (NULL == addr) {
		return;
	}
	SCB_InvalidateDCache_by_Addr(addr, dsize);
#endif
}

void CACHE_CleanDCache(uint32_t *addr, int32_t dsize) {
#if defined(STM32H750xx) || defined(STM32H743xx)
	if (NULL == addr) {
		return;
	}
	SCB_CleanDCache_by_Addr(addr, dsize);
#endif
}

void USB_Patch(void) {
#if defined(STM32H750xx) || defined(STM32H743xx)
	HAL_PWREx_EnableUSBVoltageDetector();
#endif
}
