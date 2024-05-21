/**
 ******************************************************************************
 * @file    stm32_uart.h
 * @author  iclm team
 * @brief   uart header file
 ******************************************************************************
 */
#ifndef __STM32_UART_H__
#define __STM32_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx) || defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

#define USARTx                      USART1
#define USARTx_CLK_ENABLE()         __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()        __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()      __HAL_RCC_USART1_RELEASE_RESET()

#define USARTx_TX_PIN               GPIO_PIN_9
#define USARTx_TX_GPIO_PORT         GPIOA
#define USARTx_TX_AF                GPIO_AF7_USART1
#define USARTx_RX_PIN               GPIO_PIN_10
#define USARTx_RX_GPIO_PORT         GPIOA
#define USARTx_RX_AF                GPIO_AF7_USART1

#define USARTx_BAUDRATE             (256000U)

void UART_Init(void);
void UART_Send(uint8_t *buf, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
