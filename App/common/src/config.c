/**
  ******************************************************************************
  * @file           : config.c
  * @author         : ting.gao@iclegend.com
  * @brief          : config module
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "utilities.h"
#include "banyan.h"
#include "dataprocess.h"
#include "system.h"


static FLASH_DATA_T flashData;
static uint8_t needFlashWrite = 0;

/********************************************
 @���ƣ�Config_ReloadAlgoRegister
 @���ܣ����¸�ֵ����
 @������none
 @���أ�none
*********************************************/
void Config_ReloadAlgoRegister(uint16_t systemMode)
{
#ifdef SUPPORT_DYNAMIC_SYS_MODE
	  uint8_t reload = 1;
    uint16_t loop = 0;
    uint16_t readNum = 0;
    uint32_t *readAddr = NULL;
    uint32_t flashAddr = FLASH_DATA_PAGE_ADDR;
    switch(systemMode)
    {
       default:
            reload = 0;
        break;
    }

    if(reload)
    {
        if(Radar_GetChipRegListConfigParaLen(0) < readNum)
        {
            return;
        }
    
		readAddr = (uint32_t *)Radar_GetChipRegListConfigParaAddr(0);
		for (loop = 0; loop < readNum - MAX_REG_FIX_NUM; loop++)
		{
			readAddr[loop] = REG32(flashAddr);
			flashAddr += FLASH_WORD_LEN;
		}	
        readAddr[loop] = 0x00000000;
        
        readAddr = (uint32_t *)Radar_GetChipRegListStartParaAddr(0);
        for (loop = 0; loop < REG_FIX_NUM; loop++)
		{
			readAddr[loop] = REG32(flashAddr);
			flashAddr += FLASH_WORD_LEN;
		}	
        readAddr[loop] = 0x00000000;
    }
#endif	
}

/********************************************
 @���ƣ�Config_WriteData2Flash
 @���ܣ����������õ�flashDataд��flash
 @������none
 @���أ�none
*********************************************/
void Config_WriteData2Flash(void)
{
    uint32_t flashAddr = FLASH_DATA_PAGE_ADDR;
    uint16_t idx = 0;

    Flash_ErasePage(FLASH_DATA_PAGE_ADDR);

    flashData.magic = FLASH_MAGIC_NUM;
    Flash_Program(flashAddr, &flashData.magic, 1);
    flashAddr += FLASH_WORD_LEN;

    for (idx = 0; idx < FLASH_ELEM_MAX; idx++)
    {
        Flash_Program(flashAddr, &flashData.elem[idx].elemLen, 1);
        flashAddr += FLASH_WORD_LEN;
        Flash_Program(flashAddr, flashData.elem[idx].elemAddr, flashData.elem[idx].elemLen/FLASH_WORD_LEN);
        flashAddr += flashData.elem[idx].elemLen;
    }
}

/********************************************
 @���ƣ�Config_RetrieveFlashData
 @���ܣ�Retrieve Flash Data
 @������none
 @���أ�none
*********************************************/
void Config_RetrieveFlashData(void)
{
    uint16_t idx = 0;
    uint16_t loop = 0;
    uint16_t readNum = 0;
    uint32_t *readAddr = NULL;
    uint32_t flashAddr = FLASH_DATA_PAGE_ADDR;


    flashData.magic = REG32(flashAddr);
    if (FLASH_MAGIC_NUM != flashData.magic)
    {
        printf("flash setting is empty!\r\n");
        Config_WriteData2Flash();
    }

    flashData.magic = REG32(flashAddr);
    if (FLASH_MAGIC_NUM != flashData.magic)
    {
        printf("Error: flash work abnormal!\r\n");
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
    
    flashAddr += FLASH_WORD_LEN;
    for (idx = 0; idx < FLASH_ELEM_MAX; idx++)
    {
        flashData.elem[idx].elemLen = REG32(flashAddr);
        flashAddr += FLASH_WORD_LEN;
        
        readNum = flashData.elem[idx].elemLen/FLASH_WORD_LEN;
        readAddr = (uint32_t *)flashData.elem[idx].elemAddr;
        for (loop = 0; loop < readNum; loop++)
        {
            readAddr[loop] = REG32(flashAddr);
            flashAddr += FLASH_WORD_LEN;
        }
    }
    Config_ReloadAlgoRegister(System_GetSysMode());
}

/********************************************
 @���ƣ�Config_SavePara2Flash
 @���ܣ�����д��flash����ں���
 @������none
 @���أ�none
*********************************************/
void Config_SavePara2Flash(void)
{    
    if (!needFlashWrite)
    {
        return;
    }

    Config_WriteData2Flash();
    needFlashWrite = 0;
}

/********************************************
 @���ƣ�Config_EarseFlashData
 @���ܣ�����flashҳ
 @������none
 @���أ�none
*********************************************/
void Config_EarseFlashData(void)
{    
    Flash_ErasePage(FLASH_DATA_PAGE_ADDR);
}

/********************************************
 @���ƣ�Config_NeedFlashWrite
 @���ܣ���λflashд��flag
 @������none
 @���أ�none
*********************************************/
void Config_NeedFlashWrite(void)
{
    needFlashWrite = 1;
}

/********************************************
 @���ƣ�Config_GetSN
 @���ܣ��ض�SN
 @������none
 @���أ�SN�ŵ�ַ
*********************************************/
//uint32_t Config_GetSN(void)
//{
//    uint32_t flashAddr = FLASH_SN_ADDR;

//    return REG32(flashAddr);
//}

/********************************************
 @���ƣ�Config_Init
 @���ܣ��������ó�ʼ��
 @������none
 @���أ�none
*********************************************/
void Config_Init(void)
{
    uint32_t totalLen = 0;

    totalLen += sizeof(flashData.magic);
    
    flashData.elem[FLASH_ELEM_CHIP_CONFIG_REG0].elemAddr = Radar_GetChipRegListConfigParaAddr(0); 
    flashData.elem[FLASH_ELEM_CHIP_CONFIG_REG0].elemLen = Radar_GetChipRegListConfigParaLen(0);
    totalLen += flashData.elem[FLASH_ELEM_CHIP_CONFIG_REG0].elemLen + sizeof(flashData.elem[FLASH_ELEM_CHIP_CONFIG_REG0].elemLen);
      
    flashData.elem[FLASH_ELEM_CHIP_START_REG0].elemAddr = Radar_GetChipRegListStartParaAddr(0); 
    flashData.elem[FLASH_ELEM_CHIP_START_REG0].elemLen = Radar_GetChipRegListStartParaLen(0);
    totalLen += flashData.elem[FLASH_ELEM_CHIP_START_REG0].elemLen + sizeof(flashData.elem[FLASH_ELEM_CHIP_START_REG0].elemLen);

    System_ParaInit();
    flashData.elem[FLASH_ELEM_SYS].elemAddr = System_GetSysParaAddr(); 
    flashData.elem[FLASH_ELEM_SYS].elemLen = System_GetSysParaLen();
    totalLen += flashData.elem[FLASH_ELEM_SYS].elemLen + sizeof(flashData.elem[FLASH_ELEM_SYS].elemLen);
    
    uint8_t defaultSn[8] = "12345678";
    System_SetSysSnInfo(8, defaultSn);
    flashData.elem[FLASH_ELEM_SN].elemAddr = System_GetSysSnInfoAddr(); 
    flashData.elem[FLASH_ELEM_SN].elemLen = System_GetSysSnInfoLen();
    totalLen += flashData.elem[FLASH_ELEM_SN].elemLen + sizeof(flashData.elem[FLASH_ELEM_SN].elemLen);    
    PD_ParaInit();
    flashData.elem[FLASH_ELEM_PD_CALI_PARA].elemAddr = System_GetPDCaliParaAddr(); 
    flashData.elem[FLASH_ELEM_PD_CALI_PARA].elemLen = System_GetPDCaliParaLen();
    totalLen += flashData.elem[FLASH_ELEM_PD_CALI_PARA].elemLen + sizeof(flashData.elem[FLASH_ELEM_PD_CALI_PARA].elemLen);
    
    if (totalLen > FLASH_PAGE_SIZE)
    {
        printf("Error: flashDataLen is more than FLASH_PAGE_SIZE!\r\n");
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }

    Config_RetrieveFlashData();
    
    whichDemo = System_GetSysMode();  /* ensure demo which is applied */
    //printf("sn: 0x%x\r\n", Config_GetSN());
}
