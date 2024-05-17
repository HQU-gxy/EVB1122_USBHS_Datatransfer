/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2022, ����΢
 * All rights reserved.
 *
 * ���̼����������ݰ�Ȩ��������΢����
 * @file           main
 * @company        ����΢
 * @author         RSS
 * @version        v1.0.0
 * @Target core    S5KM1122
 * @date           2023-11-14
 ******************************************************************
 */
#include <stdio.h>
#include "global_conf.h"
#include "platform.h"
#include "banyan.h"
#include "dataprocess.h"
#include "cmdprocess.h"
#include "config.h"
#ifdef STM32_PLATFORM
#include "rtos.h"
#include "cmsis_os.h"
#endif

int main(void)
{   
//#if defined RUN_APP0
//    SCB->VTOR = FLASH_BASE | 0x20000;//����ƫ����
//#elif defined RUN_APP1
//    SCB->VTOR = FLASH_BASE | 0x40000;//����ƫ����
//#endif
    Platform_Init();
    delay_ms(8);
    Config_Init();
//    Radar_Init();
    Radar_Init_Stop_BanyanE();
    Radar_Init_Start_BanyanE();
	
	
	
	
#ifdef STM32_PLATFORM
    RTOS_Init();
#endif
    
    DataProc_Init();

    CmdProc_Init();

    osKernelStart();
    
    while (1);
}

