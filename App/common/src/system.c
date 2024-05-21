/**
 ******************************************************************************
 * @file           : system.c
 * @author         : iclm team
 * @brief          : system module
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "global_conf.h"
#include "utilities.h"
#include "system.h"
#include "platform.h"
#include "dataprocess.h"
#include "usb_transfer.h"
#include "banyan.h"
#include "rtos.h"

SYS_PARA_T sysPara __ALIGN(4);
mcu_special_func_t mcuSpecialFunc __ALIGN(4);
SN_INFO_T snInfo;

/************************************************************************
 @名称；System_GetSysParaAddr
 @功能：返回sysPara结构体指针
 @参数：none
 @返回：sysPara结构体指针
*************************************************************************/
void *System_GetSysParaAddr(void) {
	return (void *)&sysPara;
}

/************************************************************************
 @名称；System_GetSysParaLen
 @功能：返回sysPara结构体长度
 @参数：none
 @返回：sysPara结构体长度
*************************************************************************/
uint32_t System_GetSysParaLen(void) {
	return sizeof(sysPara);
}

/************************************************************************
 @名称；System_GetSysMode
 @功能：返回systemMode
 @参数：none
 @返回：systemMode
*************************************************************************/
uint16_t System_GetSysMode(void) {
	return sysPara.systemMode;
}

/************************************************************************
 @名称；System_GetUploadSampleRate
 @功能：返回uploadSampleRate
 @参数：none
 @返回：uploadSampleRate
*************************************************************************/
uint16_t System_GetUploadSampleRate(void) {
	return sysPara.uploadSampleRate;
}

/************************************************************************
 @名称；System_GetDebugMode
 @功能：返回debugMode
 @参数：none
 @返回：debugMode
*************************************************************************/
uint16_t System_GetDebugMode(void) {
	return sysPara.debugMode;
}

/************************************************************************
 @名称；System_GetDataUploadMode
 @功能：返回dataUploadMode
 @参数：none
 @返回：dataUlloadMode
*************************************************************************/
uint32_t System_GetDataUploadMode(void) {
	return sysPara.dataUploadMode;
}

/************************************************************************
 @名称；System_UseMcuPackWrapper
 @功能：返回是否使用PackWrapper标志位
 @参数：none
 @返回：useMcuPackWrapper
*************************************************************************/
uint16_t System_UseMcuPackWrapper(void) {
	return sysPara.useMcuPackWrapper;
}

/************************************************************************
 @名称；MCU_FuncParaUpdate
 @功能：设置MCU特殊函数
 @参数：type，类型
		val，值
 @返回：none
*************************************************************************/
void MCU_FuncParaUpdate(uint16_t type, uint32_t val) {
	switch (type) {
	case TEMP_CHECK_ENABLE:
		if (val != 0) {
			mcuSpecialFunc.isTempCheckEnable = 1;
			mcuSpecialFunc.tempReportGap     = val;
		} else {
			mcuSpecialFunc.isTempCheckEnable = 0;
		}
		break;

	case POWER_CHECK_ENABLE:
		if (val != 0) {
			mcuSpecialFunc.isPowerCheckEnable = 1;
			mcuSpecialFunc.powerReportGap     = val;
		} else {
			mcuSpecialFunc.isPowerCheckEnable = 0;
		}
		break;

	default:
		break;
	}
}


/************************************************************************
 @名称；System_ParaInit
 @功能：sysPara结构体初始化
 @参数：none
 @返回：none
*************************************************************************/
void System_ParaInit(void) {
#ifdef SUPPORT_DYNAMIC_SYS_MODE
	sysPara.systemMode = SYS_MODE_DEFAULT;
#endif
	sysPara.uploadSampleRate  = UPLOAD_SAMPLE_RATE;
	sysPara.debugMode         = DEBUG_MODE_DEFAULT;
	sysPara.dataUploadMode    = 0;
	sysPara.useMcuPackWrapper = MCU_PACK_WRAPPER_DEFAULT;
}

/************************************************************************
 @名称；System_ParaUpdate
 @功能：sysPara更新
 @参数：type，数据类型
		val，值
 @返回：none
*************************************************************************/
int8_t System_ParaUpdate(uint16_t type, int32_t val) {
	switch (type) {
#ifdef SUPPORT_DYNAMIC_SYS_MODE
	case SYS_SYSTEM_MODE:
		sysPara.systemMode = (int16_t)val;
		if ((sysPara.systemMode == SYS_MODE_ABD) || (sysPara.systemMode == SYS_MODE_MTT)) {
			Radar_Write_ABD_MTT_Para(sysPara.systemMode);
			mcuSpecialFunc.isPowerCheckEnable = 0; /* 调用两个demo时，禁止McuSepcialFuncTask线程读取温度和功率 */
			mcuSpecialFunc.isTempCheckEnable  = 0;
		}
		break;
#endif

#if defined(SUPPORT_DATA_PASSTHROUGH) || defined(SUPPORT_DYNAMIC_SYS_MODE)
	case SYS_UPLOAD_SP_RATE:
		sysPara.uploadSampleRate = (int16_t)val;
		DataProc_ResetRecvCnt();
		break;
#endif

	case SYS_DEBUG_MODE:
		sysPara.debugMode = (int16_t)val;
		break;

	case SYS_MCU_PACK_WRAPPER:
		sysPara.useMcuPackWrapper = (uint16_t)val;
		if (!sysPara.useMcuPackWrapper) {
			mcuSpecialFunc.isPowerCheckEnable = 0; /* 不使用MCU数据封装时，禁止McuSepcialFuncTask线程读取温度和功率 */
			mcuSpecialFunc.isTempCheckEnable  = 0;
		} else {
			mcuSpecialFunc.isPowerCheckEnable = 1; /* 不使用MCU数据封装时，禁止McuSepcialFuncTask线程读取温度和功率 */
			mcuSpecialFunc.isTempCheckEnable  = 1;
		}
		break;

	case SYS_CHIP_NUM:
		sysPara.chipNum = val;
		break;
	case SYS_CHIP_ADDR:
		*(volatile uint32_t *)&sysPara.chipAddr[0] = val;
		break;

	default:
		return -1;
	}

	return 0;
}

/************************************************************************
 @名称；System_ParaRead
 @功能：回读sysPara
 @参数：type，数据类型
 @返回：sysPara
*************************************************************************/
int32_t System_ParaRead(uint16_t type) {
	switch (type) {
	case SYS_SYSTEM_MODE:
		return sysPara.systemMode;

	case SYS_UPLOAD_SP_RATE:
		return sysPara.uploadSampleRate;

	case SYS_DEBUG_MODE:
		return sysPara.debugMode;

	default:
		return 0x7fffffff; /*invalid value*/
	}
}

/************************************************************************
 @名称；System_Reset
 @功能：系统复位
 @参数：none
 @返回：none
*************************************************************************/
void System_Reset(void) {
	SPI_DeInit();
#ifdef STM32_PLATFORM
	UsbTransfer_ResetUsbBuffer();
	DataProc_ResetRadarDataQueue();
#endif
}

/************************************************************************
 @名称；System_Reconfig
 @功能：系统重配置
 @参数：dataType，数据类型
 @返回：none
*************************************************************************/
void System_Reconfig(uint8_t dataType) {
	uint16_t dmaBufLen = 0;

#if defined(CONFIG_DEBUG)
	printf("system reconfig check...\r\n");
#endif

	DataProc_NeedReconfig(dataType);

#if defined(CONFIG_DEBUG)
	printf("system now do reconfig!\r\n");
#endif
	dmaBufLen = Radar_GetIsDataMerge() ? DataProc_GetRadarDataLen() * 4 : DataProc_GetRadarDataLen() * 2;

	RADAR_DATA_T radarData;
	radarData.buf     = NULL;
	radarData.len     = 0;
	radarData.dmaFlag = DMA_RECV_FLAG_MAX;

	for (uint8_t channel = 0; channel < CHANNEL_MAX; channel++) {
		DataProc_Send2RadarDataQueue(channel, &radarData);
	}
	SPI_DeInit();
#ifdef STM32_PLATFORM
	UsbTransfer_ResetUsbBuffer();
	DataProc_ResetRadarDataQueue();
#endif
	SPI_Init(dmaBufLen);
	whichDemo = System_GetSysMode();
}

void System_SetSysSnInfo(uint16_t nLen, uint8_t *buf) {
	if ((nLen == 0) || (buf == NULL)) {
		return;
	}
	snInfo.snLen = nLen;
	memcpy(snInfo.cSn, buf, nLen);
}

void *System_GetSysSnInfoAddr(void) {
	return (void *)&snInfo;
}

uint32_t System_GetSysSnInfoLen(void) {
	return sizeof(snInfo);
}
