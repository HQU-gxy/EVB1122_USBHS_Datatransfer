/**
  ******************************************************************************
  * @file    stm32_gpio.c
  * @author  iclm team
  * @brief   gpio services
  ******************************************************************************
  */
#include "stm32_gpio.h"

static GPIO_TypeDef* GPIO_PORT[LED_MAX] = 
{
    LED0_GPIO_PORT,
    LED1_GPIO_PORT,
    LED2_GPIO_PORT,
    LED3_GPIO_PORT,
    LED4_GPIO_PORT
};

static uint16_t GPIO_PIN[LED_MAX] = 
{
    LED0_PIN,
    LED1_PIN,
    LED2_PIN, 
    LED3_PIN, 
    LED4_PIN
};


void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

 #if defined(STM32F429xx)
    __HAL_RCC_GPIOE_CLK_ENABLE();
 #elif defined(STM32H750xx) || defined(STM32H743xx)
    __HAL_RCC_GPIOD_CLK_ENABLE();
 #endif
	__HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIO_PORT[LED0], LED0_PIN|LED1_PIN|LED2_PIN|LED3_PIN 
                            |LED4_PIN, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = LED0_PIN|LED1_PIN|LED2_PIN|LED3_PIN 
                            |LED4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_PORT[LED0], &GPIO_InitStruct);

    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void LED_On(led_typedef_enum lednum)
{
    HAL_GPIO_WritePin(GPIO_PORT[lednum], GPIO_PIN[lednum], GPIO_PIN_SET);
}

void LED_Off(led_typedef_enum lednum)
{
    HAL_GPIO_WritePin(GPIO_PORT[lednum], GPIO_PIN[lednum], GPIO_PIN_RESET);
}

void LED_Toggle(led_typedef_enum lednum)
{
    HAL_GPIO_TogglePin(GPIO_PORT[lednum], GPIO_PIN[lednum]);
}


