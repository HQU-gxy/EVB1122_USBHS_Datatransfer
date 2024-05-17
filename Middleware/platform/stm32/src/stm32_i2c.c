/**
  ******************************************************************************
  * @file    stm32_i2c.c
  * @author  iclm team
  * @brief   i2c services
  ******************************************************************************
  */
#include "stm32_i2c.h"
#include "stm32_misc.h"
#include "utilities.h"

static I2C_HandleTypeDef hi2c;

uint32_t I2C_GetConfigBySpeed(uint8_t I2CSpeed)
{
	uint32_t Config = stm32_I2C_Speed_Config_400k;
	switch(I2CSpeed)
	{
		case I2C_Speed_100k:
		{
			Config = stm32_I2C_Speed_Config_100k;
			break;
		}
		case I2C_Speed_200k:
		{
			Config = stm32_I2C_Speed_Config_200k;
			break;
		}
		case I2C_Speed_300k:
		{
			Config = stm32_I2C_Speed_Config_300k;
			break;
		}
		case I2C_Speed_400k:
		{
			Config = stm32_I2C_Speed_Config_400k;
			break;
		}
		case I2C_Speed_1M:
		{
			Config = stm32_I2C_Speed_Config_1M;
			break;
		}
		default:
		{
			break;
		}
	}
	return Config;
}

HAL_StatusTypeDef I2C_Write(uint16_t devAddr, uint8_t regAddr, uint16_t regVal)
{
    regVal = LE16_TO_BIG(regVal);

    return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regVal, sizeof(regVal), I2C_TIMEOUT);
}


HAL_StatusTypeDef I2C_Read(uint16_t devAddr, uint8_t regAddr, uint16_t* regVal)
{  
	uint16_t val = 0;
	uint8_t retryCount = 3;
	HAL_StatusTypeDef result = HAL_OK;
	while(retryCount--)
	{
		result = HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&val,  sizeof(val), I2C_TIMEOUT);
		if( result == HAL_OK)
		{
			break;
		}
		else
		{
			HAL_Delay(1);
		}
	}
	*regVal = BIG16_TO_LE(val);
  return result;
}

void I2C_Init(uint8_t I2CSpeed)
{
	  uint32_t ConfigSpeed = I2C_GetConfigBySpeed(I2CSpeed);
    hi2c.Instance = I2Cx;
    hi2c.Init.ClockSpeed = ConfigSpeed;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c) != HAL_OK)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c, 0) != HAL_OK)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(i2cHandle->Instance==I2C1)
    {  
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C1 GPIO Configuration
					PB6     ------> I2C1_SCL
					PB7     ------> I2C1_SDA
				*/
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
    if(i2cHandle->Instance==I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
    }
} 

void I2C_ResetSpeed(uint8_t I2CSpeed)
{
	HAL_I2C_DeInit(&hi2c);
	I2C_Init(I2CSpeed);
}
