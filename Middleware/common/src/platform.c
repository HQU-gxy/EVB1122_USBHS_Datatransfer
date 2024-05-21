/**
 ******************************************************************************
 * @file    platform.c
 * @author  ting.gao@iclegend.com
 * @brief   platform dependent services
 ******************************************************************************
 */
#include "platform.h"
#include "stdio.h"


static uint8_t fac_us  = 0;
static uint16_t fac_ms = 0;

void delay_init(uint8_t SYSCLK) {
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
	fac_us = SYSCLK / 8;
	fac_ms = (uint16_t)fac_us * 1000;
}


void delay_us(uint32_t nus) {
	uint32_t temp;
	SysTick->LOAD = nus * fac_us;
	SysTick->VAL  = 0x00;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	do {
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16)));
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0X00;
}

// nms<=798ms
void delay_xms(uint16_t nms) {
	uint32_t temp;
	SysTick->LOAD = (uint32_t)nms * fac_ms;
	SysTick->VAL  = 0x00;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	do {
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16)));
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0X00;
}

// nms:0~65535
void delay_ms(uint16_t nms) {
	uint8_t repeat = nms / 540;

	uint16_t remain = nms % 540;
	while (repeat) {
		delay_xms(540);
		repeat--;
	}
	if (remain) delay_xms(remain);
}


void Delay(uint32_t time) {
#ifdef STM32_PLATFORM
	HAL_Delay(time);
#endif
}

void Indicator_RadarDataReceived(uint16_t threshold) {
	static uint16_t recvCnt = 0;

	recvCnt++;
	if (recvCnt % threshold == 0) {
		LED_Toggle(RADAR_DATA_RECV_LED);
	}
}

void Indicator_RadarDataIndexError(void) {
	LED_Toggle(RADAR_DATA_INDEX_ERR_LED);
}

void Indicator_RadarDataRecvOverFlow(void) {
	LED_Toggle(RADAR_DATA_RECV_OVER_FLOW_LED);
}

void Indicator_RadarDataSendOverFlow(void) {
	static uint8_t recvCnt = 0;

	recvCnt++;
	if (recvCnt % INDICATOR_SEND_OF_THRESHOLD == 0) {
		LED_Toggle(RADAR_DATA_SEND_OVER_FLOW_LED);
	}
}

void Indicator_CmdDataRecvOverFlow(void) {
	LED_Toggle(CMD_DATA_RECV_OVER_FLOW_LED);
}

void Platform_Init(void) {
#ifdef STM32_PLATFORM
	HAL_Init();
	SystemClock_Config();
	delay_init(168);
#endif
	LED_Init();
	UART_Init();

	SPI4_Config_Init();

	//    I2C_Init(I2C_Speed_400k);
}
