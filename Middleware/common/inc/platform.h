/**
  ******************************************************************************
  * @file    platform.h
  * @author  ting.gao@iclegend.com
  * @brief   platform dependent header file
  ******************************************************************************
  */
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>

#ifdef STM32_PLATFORM
#include "stm32_clock.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"
#include "stm32_i2c.h"
#include "stm32_spi.h"
#include "stm32_flash.h"
#include "stm32_misc.h"
#endif


#define INDICATOR_RECV_THRESHOLD            (1000)
#define INDICATOR_RECV_THD_DFFT_SHIFT       (3)
#define INDICATOR_RECV_THD_DPEAK_SHIFT      (6)
#define INDICATOR_SEND_OF_THRESHOLD         (10)

void Platform_Init(void);
void Delay(uint32_t Delay);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void Indicator_RadarDataReceived(uint16_t threshold);
void Indicator_RadarDataIndexError(void);
void Indicator_RadarDataRecvOverFlow(void);
void Indicator_RadarDataSendOverFlow(void);
void Indicator_CmdDataRecvOverFlow(void);

#ifdef __cplusplus
}
#endif

#endif

