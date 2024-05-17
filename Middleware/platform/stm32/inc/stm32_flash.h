/**
  ******************************************************************************
  * @file    stm32_flash.h
  * @author  ting.gao@iclegend.com
  * @brief   flash header file
  ******************************************************************************
  */
#ifndef __STM32_FLASH_H__
#define __STM32_FLASH_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#if defined(STM32F429xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32H750xx)
#include "stm32h7xx_hal.h"
#include "w25qxx.h"
#endif

#if defined(STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */

#if defined(STM32F429xx)
#define FMC_START_ADDR                (0x08000000U)
#define FMC_END_ADDR                  (0x08080000U)

#define FMC_PAGE_SIZE                 (0x20000U)
#define FMC_SW_SETTING_ADDR           ADDR_FLASH_SECTOR_7
//#define FMC_HW_SETTING_ADDR           ADDR_FLASH_SECTOR_6

#define BOOT_LOAD_SIZE                (1024 * 48)       /* 48K */
#define APP_SIZE                      (1024 * 128)      /* 128K */

#define BOOT_LOAD_ADDR                ADDR_FLASH_SECTOR_0
#define APP0_ADDR                     ADDR_FLASH_SECTOR_5
#define APP1_ADDR                     ADDR_FLASH_SECTOR_6
#define APP2_ADDR                     ADDR_FLASH_SECTOR_7
#define APP3_ADDR                     ADDR_FLASH_SECTOR_4

#define APP_RUN_STORE_ADDR            ADDR_FLASH_SECTOR_3
#define APP_NEED_UPDATE_ADDR          (ADDR_FLASH_SECTOR_3 + 4)
#endif

#if defined(STM32H750xx) 
#define FMC_START_ADDR                (0x00000000U)
#define FMC_END_ADDR                  (0x00800000U)

#define FMC_PAGE_SIZE                 (0x1000U)
#define FMC_SW_SETTING_ADDR           (FMC_END_ADDR - FMC_PAGE_SIZE)
#define FMC_HW_SETTING_ADDR           (FMC_END_ADDR - FMC_PAGE_SIZE * 2)
#endif

#if defined(STM32H743xx)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define FMC_START_ADDR                (ADDR_FLASH_SECTOR_0_BANK2)
#define FMC_END_ADDR                  (ADDR_FLASH_SECTOR_7_BANK2 + FMC_PAGE_SIZE)

#define FMC_PAGE_SIZE                 (0x20000U)
#define FMC_SW_SETTING_ADDR           ADDR_FLASH_SECTOR_7_BANK2
#define FMC_HW_SETTING_ADDR           ADDR_FLASH_SECTOR_6_BANK2
#endif

int8_t Flash_ErasePage(uint32_t pageAddr);
int8_t Flash_Program(uint32_t flashStartAddr, uint32_t *data, uint16_t len);
uint32_t Flash_ReadWord(uint32_t addr);
int8_t Flash_WriteWord(uint32_t WriteAddr,uint32_t WriteData);

#ifdef __cplusplus
}
#endif

#endif

