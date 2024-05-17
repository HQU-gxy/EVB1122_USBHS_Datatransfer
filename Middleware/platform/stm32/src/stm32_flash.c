/**
  ******************************************************************************
  * @file    stm32_flash.c
  * @author  ting.gao@iclegend.com
  * @brief   flash services
  ******************************************************************************
  */
#include <string.h>
#include "stm32_flash.h"
#include "utilities.h"
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
#if defined(STM32F429xx)
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else
  {
    sector = FLASH_SECTOR_7;
  }  
#elif defined(STM32H743xx)
  if(((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))    
  {
    sector = FLASH_SECTOR_0;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))    
  {
    sector = FLASH_SECTOR_1;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))    
  {
    sector = FLASH_SECTOR_2;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))    
  {
    sector = FLASH_SECTOR_3;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))    
  {
    sector = FLASH_SECTOR_4;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))    
  {
    sector = FLASH_SECTOR_5;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))    
  {
    sector = FLASH_SECTOR_6;  
  }
  else if(((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((Address < FMC_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
  {
     sector = FLASH_SECTOR_7;  
  }
  else
  {
    sector = FLASH_SECTOR_7;
  }
#endif
  return sector;
}

int8_t Flash_ErasePage(uint32_t pageAddr)
{
#if defined(STM32F429xx) || defined(STM32H743xx)
    uint32_t firstSector = 0;
    uint32_t SECTORError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
#endif

//    if (pageAddr % FMC_PAGE_SIZE)
//    {
//        return FAIL;
//    }

#if defined(STM32F429xx) || defined(STM32H743xx)    
    HAL_FLASH_Unlock();
    
    firstSector = GetSector(pageAddr);
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
#if defined(STM32H743xx)
	EraseInitStruct.Banks         = FLASH_BANK_2;
#endif	
    EraseInitStruct.Sector        = firstSector;
    EraseInitStruct.NbSectors     = 1;
    
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return FAIL;
    }

    HAL_FLASH_Lock();
#elif defined(STM32H750xx)
    W25QXX_Erase_Sector(pageAddr/FMC_PAGE_SIZE);
#endif

    return OK;
}

int8_t Flash_Program(uint32_t flashStartAddr, uint32_t *data, uint16_t len)
{
    uint16_t loop = 0;

    if ((flashStartAddr + (len * sizeof(uint32_t))) > FMC_END_ADDR)
    {
        return FAIL;
    }

    if (flashStartAddr % sizeof(uint32_t))
    {
        return FAIL;
    }

#if defined(STM32F429xx)
    HAL_FLASH_Unlock();

    for (loop = 0; loop < len; loop++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashStartAddr, data[loop]);
        flashStartAddr += sizeof(uint32_t);
    }
    
    HAL_FLASH_Lock();
	
#elif defined(STM32H743xx)
	uint32_t *pData = data;
	uint32_t tempData[8] = {0};
	uint32_t programStartPos = 32 * (flashStartAddr / 32);
	uint16_t programOffPosDword = (flashStartAddr % 32) / 4;
	uint16_t programRemainDword = 8 - programOffPosDword;	  
	if(len <= programRemainDword)
	{
		programRemainDword = len;
	}
	
	HAL_FLASH_Unlock();
	while(1)
	{
		memset(tempData, 0xFFFFFFFF, sizeof(tempData));
		for(loop = 0; loop < programOffPosDword; loop++)
		{
			tempData[loop] = REG32(programStartPos + 4 * loop);
		}
		
		for(loop = 0; loop < programRemainDword; loop++) 
		{
			tempData[programOffPosDword + loop] = pData[loop];
		}
		
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, programStartPos, (uint32_t)tempData);
		if(len == programRemainDword)
		{
			break;
		}
		else
		{
			programOffPosDword = 0;
			pData += programRemainDword;
			programStartPos += 32;
			len -= programRemainDword;
			if(len > 8)
			{
				programRemainDword = 8;
			}
			else
			{
				programRemainDword = len;
			}
		}
	}
	HAL_FLASH_Lock();
#elif defined(STM32H750xx)
    (void)loop;
    W25QXX_Write((uint8_t*)data, flashStartAddr, (len * sizeof(uint32_t))); 
#endif

    return OK;
}

/********************************************
 @名称；Flash_ReadWord
 @功能：读取指定地址的字(32位数据) 
 @参数：addr, 读地址 
 @返回：读到的数据
*********************************************/
uint32_t Flash_ReadWord(uint32_t addr)
{
		return *(__IO uint32_t *)addr; 
}


int8_t Flash_WriteWord(uint32_t WriteAddr,uint32_t WriteData)
{
		int8_t ret = 0;
		ret = Flash_Program(WriteAddr, &WriteData,1);
		return ret;
}

