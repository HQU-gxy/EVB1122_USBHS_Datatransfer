/**
  ******************************************************************************
  * @file    stm32_spi.h
  * @author  iclm team
  * @brief   spi header file
  ******************************************************************************
  */
#ifndef __STM32_SPI_H__
#define __STM32_SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx) || defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

#include "global_conf.h"

typedef enum
{
    DMA_RECV_FLAG_MEM_0        = 0,
    DMA_RECV_FLAG_MEM_1        = 1,
    DMA_RECV_FLAG_MAX
}dmaRecvFlagEnum;

#define SPI_FRAME_DLEN_MAX      RADAR_DATA_MAX_LEN
#define SPI_FRAME_HLEN          (4)
#define SPI_FRAME_TLEN          (4)

#define SPI_FRAME_WRAPPER           (12)
#define SPI_FRAME_PADDING       (24)
#define SPI_FRAME_WRAPPER_HEAD (0x484C4349)
#define SPI_FRAME_WRAPPER_TAIL (0x544C4349)
#define SPI_FRAME_WRAPPER_SHIFT 8
#define SPI_FRAME_LEN_MAX       (SPI_FRAME_WRAPPER + SPI_FRAME_DLEN_MAX + SPI_FRAME_HLEN + SPI_FRAME_TLEN + SPI_FRAME_PADDING)

#define DATA_RECV_BUF_SIZE           (SPI_FRAME_LEN_MAX * 2) /*ping-pong buffer*/


extern uint8_t g_dataRecvBuf[CHANNEL_MAX][DATA_RECV_BUF_SIZE];
extern volatile uint8_t g_dataRecvFlag[CHANNEL_MAX][DMA_RECV_FLAG_MAX];

extern uint8_t isSingleSPI;
extern uint8_t SpiMethod;

void SPI4_Init(void);
void SPI4_Config_Init(void);
void SPI4_DeInit(void);

void SPI_Init(uint16_t dmaBufLen);
void SPI_DeInit(void);
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle);

HAL_StatusTypeDef SPI4_Write(uint16_t devAddr, uint8_t regAddr, uint16_t regVal);
HAL_StatusTypeDef SPI4_Read(uint16_t devAddr, uint8_t regAddr, uint16_t* regVal);

uint8_t IsSingleSpiInit(void);

#ifdef __cplusplus
}
#endif

#endif

