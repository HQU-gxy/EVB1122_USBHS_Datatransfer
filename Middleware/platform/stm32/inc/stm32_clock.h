/**
 ******************************************************************************
 * @file    stm32_clock.h
 * @author  iclm team
 * @brief   clock header file
 ******************************************************************************
 */
#ifndef __STM32_CLOCK_H__
#define __STM32_CLOCK_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx) || defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif
