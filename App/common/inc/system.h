/**
  ******************************************************************************
  * @file           : system.h
  * @author         : iclm team
  * @brief          : system header file
  ******************************************************************************
  */
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#ifdef __cplusplus
 extern "C" {
#endif 
#include "global_def.h"
#include "stdint.h"
#include "cmdprocess.h"




typedef struct SYS_PARA
{
    int16_t systemMode;
    int16_t uploadSampleRate;
    int16_t debugMode;
    int16_t pad;
    int32_t dataUploadMode;
    uint8_t chipNum;
    uint8_t chipAddr[4];
    uint16_t useMcuPackWrapper;
}SYS_PARA_T;


typedef struct _mcu_special_func
{
    uint8_t isTempCheckEnable;
    uint8_t isPowerCheckEnable;
    uint32_t tempReportGap; // Unit: ms
    uint32_t powerReportGap; // Unit: ms
}mcu_special_func_t;

enum _mcu_func_para
{
    TEMP_CHECK_ENABLE = 0,    /* 使能温度检测，值为检测间隔（毫秒），值为0表示关闭 */
    POWER_CHECK_ENABLE,       /* 使能功率检测，值为检测间隔（毫秒），值为0表示关闭 */
};

extern mcu_special_func_t mcuSpecialFunc;
extern SYS_PARA_T sysPara;
void MCU_FuncParaUpdate(uint16_t type, uint32_t val);

 typedef enum  
{
	SYS_SYSTEM_MODE = 0,
    SYS_UPLOAD_SP_RATE,
    SYS_DEBUG_MODE,
    SYS_MCU_PACK_WRAPPER,
	SYS_PRE_START,
    SYS_CHIP_NUM,
    SYS_CHIP_ADDR
}sysParaEnum;

#pragma pack(4)
typedef struct SN_INFO
{
    int16_t snLen;
    uint8_t cSn[SN_LEN];
}SN_INFO_T;
#pragma pack()


uint16_t System_GetSysMode(void);
uint16_t System_GetUploadSampleRate(void);
uint16_t System_GetDebugMode(void);
void* System_GetSysParaAddr(void);
uint32_t System_GetSysParaLen(void);
uint32_t System_GetDataUploadMode(void);
uint16_t System_UseMcuPackWrapper(void);
void System_ParaInit(void);
int8_t System_ParaUpdate(uint16_t type, int32_t val);
int32_t System_ParaRead(uint16_t type);
void System_Reconfig(uint8_t dataType);
void System_Reset(void);
void System_SetSysSnInfo(uint16_t nLen, uint8_t* buf);
void* System_GetSysSnInfoAddr(void);
uint32_t System_GetSysSnInfoLen(void);

#ifdef __cplusplus
}
#endif

#endif


