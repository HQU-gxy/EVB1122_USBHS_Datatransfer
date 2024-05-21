/**
 ******************************************************************************
 * @file    stm32_gpio.h
 * @author  iclm team
 * @brief   gpio header file
 ******************************************************************************
 */
#ifndef __STM32_GPIO_H__
#define __STM32_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx) || defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

typedef enum {
	LED0 = 0,
	LED1 = 1,
	LED2 = 2,
	LED3 = 3,
	LED4 = 4,
	LED_MAX
} led_typedef_enum;

#if defined(STM32F429xx)
#define LED0_PIN       GPIO_PIN_2
#define LED0_GPIO_PORT GPIOD
#define LED1_PIN       GPIO_PIN_1
#define LED1_GPIO_PORT GPIOD
#define LED2_PIN       GPIO_PIN_0
#define LED2_GPIO_PORT GPIOD
#define LED3_PIN       GPIO_PIN_3
#define LED3_GPIO_PORT GPIOD
#define LED4_PIN       GPIO_PIN_4
#define LED4_GPIO_PORT GPIOD
#endif

#if defined(STM32H750xx)
#define LED0_PIN       GPIO_PIN_2
#define LED0_GPIO_PORT GPIOD
#define LED1_PIN       GPIO_PIN_3
#define LED1_GPIO_PORT GPIOD
#define LED2_PIN       GPIO_PIN_4
#define LED2_GPIO_PORT GPIOD
#define LED3_PIN       GPIO_PIN_5
#define LED3_GPIO_PORT GPIOD
#define LED4_PIN       GPIO_PIN_6
#define LED4_GPIO_PORT GPIOD
#endif

#if defined(STM32H743xx)
#define LED0_PIN       GPIO_PIN_1
#define LED0_GPIO_PORT GPIOD
#define LED1_PIN       GPIO_PIN_2
#define LED1_GPIO_PORT GPIOD
#define LED2_PIN       GPIO_PIN_3
#define LED2_GPIO_PORT GPIOD
#define LED3_PIN       GPIO_PIN_4
#define LED3_GPIO_PORT GPIOD
#define LED4_PIN       GPIO_PIN_5
#define LED4_GPIO_PORT GPIOD
#endif

#define RADAR_DATA_RECV_LED           LED0
#define RADAR_DATA_INDEX_ERR_LED      LED1
#define RADAR_DATA_RECV_OVER_FLOW_LED LED2
#define RADAR_DATA_SEND_OVER_FLOW_LED LED3
#define CMD_DATA_RECV_OVER_FLOW_LED   LED4

void LED_On(led_typedef_enum lednum);
void LED_Off(led_typedef_enum lednum);
void LED_Toggle(led_typedef_enum lednum);
void LED_Init(void);

#ifdef __cplusplus
}
#endif

#endif
