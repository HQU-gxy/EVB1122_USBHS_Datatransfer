/**
  ******************************************************************************
  * @file    stm32_misc.h
  * @author  iclm team
  * @brief   misc header file
  ******************************************************************************
  */
#ifndef __STM32_MISC_H__
#define __STM32_MISC_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx) || defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))

void CACHE_Enable(void);
void CACHE_InvalDCache(uint32_t *addr, int32_t dsize);
void CACHE_CleanDCache(uint32_t *addr, int32_t dsize);
void USB_Patch(void);

#ifdef __cplusplus
}
#endif

#endif


