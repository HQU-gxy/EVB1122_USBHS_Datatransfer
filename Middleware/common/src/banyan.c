/**
 ******************************************************************************
 * @file           : banyan.c
 * @author         : iclm team
 * @brief          : banyan driver
 ******************************************************************************
 */
#include <stdio.h>
#include "global_conf.h"
#include "platform.h"
#include "banyan.h"
#include "config.h"
#include "stm32_spi.h"
#include "banyan_param.h"

uint8_t g_TxCount = (CHANNEL_MAX + 1) / 2;

uint8_t g_ChipCount = (CHANNEL_MAX + 1) / 2;
uint8_t g_RxCount   = CHANNEL_MAX;

static uint16_t rawPointMap[RAW_MAP_NUM] =
	{
		RAW_POINT_64,
		RAW_POINT_128,
		RAW_POINT_256,
		RAW_POINT_512,
		RAW_POINT_1024};


static const RADAR_REG_T RegList_LowPWR[] =
	{
		{0x70, 0x1020},
		{0x6C, 0x8880},
		{0x6D, 0x8800},
		{0x72, 0x0650},
		{0x67, 0x0000},
		{0x66, 0xF0F0},
		{0x6E, 0x03FC},
		{0x41, 0x4804},
		{0x00, 0x0000}};

static const RADAR_REG_T RegList_NormalPWR[] =
	{
		{0x41, 0xc864},
		{0x72, 0x0653},
		{0x6C, 0x9990},
		{0x6D, 0x9940},
		{0x70, 0x32a0},
		{0x6E, 0xabFC},
		{0x66, 0x0a00},
		{0x67, 0x1840},
		{0x00, 0x0000}};

const static RADAR_REG_T InitChipRegListStop_BanyanE[9] __attribute__((aligned(4))) =
	{

		{0x40, 0x4207},
		{0x41, 0x0000},
		{0x09, 0xE901},
		{0x01, 0x0000},
		{0x67, 0x0000},
		{0x72, 0x0650},
		{0x3A, 0x8410},
		{0x77, 0x3200},
		{0xFF, 0xFFFF} /*must be last, do not delete!!!*/
};


void Radar_Init_Stop_BanyanE(void) {
	uint16_t loop = 0;

	loop = 0;
	while (((InitChipRegListStop_BanyanE[loop].addr != 0xFF) || (InitChipRegListStop_BanyanE[loop].val != 0xFFFF))) {
		if (SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitChipRegListStop_BanyanE[loop].addr), InitChipRegListStop_BanyanE[loop].val) != HAL_OK) {
		}
		loop++;
	}

	loop = 0;
	while (((InitChipRegListConfig0[loop].addr != 0xFF) || (InitChipRegListConfig0[loop].val != 0xFFFF))) {
		SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitChipRegListConfig0[loop].addr), InitChipRegListConfig0[loop].val);
		loop++;
	}
}

void Radar_Init_Start_BanyanE(void) {
	uint16_t loop = 0;
	while (((InitChipRegListStart0[loop].addr != 0xFF) || (InitChipRegListStart0[loop].val != 0xFFFF))) {
		SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitChipRegListStart0[loop].addr), InitChipRegListStart0[loop].val);
		loop++;
	}
}

uint8_t Radar_GetRxGainType(void) {
	uint8_t valRet     = 0;
	uint16_t valLpf    = 0;
	uint16_t valLna    = 0;
	uint16_t bAutoMode = 0;

	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_FUN_SWITCH, &bAutoMode);
	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_LPF_CHANNEL_ENABLE, &valLpf);
	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_LNA_CHANNEL_ENABLE, &valLna);

	if ((bAutoMode & 0x180) == 0x100) {
		valRet |= 1;
	} else if ((bAutoMode & 0x180) == 0x180) {
		valRet |= 3;
	} else {
		if ((((valLpf >> 8) & 0xa) | ((valLna >> 14) & 0x2)) != 0) {
			valRet |= 1;
		}

		if ((((valLpf >> 8) & 0x5) | ((valLna >> 14) & 0x1)) != 0) {
			valRet |= 2;
		}
	}

	return (valRet == 0) ? 3 : valRet;
}

uint16_t Radar_GetFftPoint(void) {
	uint16_t regVal = 64;
	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_FFT_NUM, &regVal);
	return regVal;
}

uint16_t Radar_GetRawPoint(void) {
	uint16_t val    = 256;
	uint16_t rawVal = 0;

	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_RAW_PEAK_NUM, &val);
	rawVal = (val >> BANYAN_RAW_POS) & BANYAN_RAW_MASK;

	if (rawVal < RAW_MAP_NUM) {
		return rawPointMap[rawVal];
	} else {
		return RAW_POINT_64;
	}
}

uint16_t Radar_GetOneFrameChirpNum(void) {
	uint16_t val = 32;
	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_PAT_CHIRP_NUM, &val);
	val &= 0x1FF;
	return val;
}

uint16_t Radar_GetDfftRoiEnable(void) {
	uint16_t roiValue = 0;
	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_DFFT_ROI, &roiValue);
	if (roiValue & BANYAN_DIG_DFFT_ROI) {
		return 0;
	} else {
		return 1;
	}
}
uint16_t Radar_GetDfftDataNum(void) {
	uint16_t val = 0x2000;

	uint16_t valRet = 0;
	if (Radar_GetDfftRoiEnable()) {
		SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_DFFT_ROI_DATA_NUM, &val);
		valRet = (val >> BANYAN_DFFT_DATA_NUM_POS) + 1;
	} else {
		SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_DFFT_DATA_NUM, &val);
		valRet = val & 0xFF;
	}

	return valRet;
}

uint16_t Radar_GetDfftPeakSize(void) {
	uint16_t val = 32;

	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_RAW_PEAK_NUM, &val);

	return ((val & BANYAN_PEAK_MASK) * 4); /*4--word length*/
}

uint16_t Radar_GetDfftChirpNum(void) {
	uint16_t val = 0x2000;

	uint16_t valRet = 0;
	if (Radar_GetDfftRoiEnable()) {
		SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_DFFT_ROI_CHIRP_NUM, &val);
		valRet = (val >> BANYAN_DFFT_DATA_NUM_POS) + 1;
	} else {
		SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_DFFT_CHIRP_NUM, &val);
		valRet = val & 0xFF;
	}

	return valRet;
}

uint8_t Radar_GetDataType(void) {
	uint8_t dataType = DATA_TYPE_FFT;
	uint16_t val     = BANYAN_FFT_DATA;

	SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_FUN_SWITCH, &val);

	if (val & BANYAN_DFFT_DATA) {
		dataType = DATA_TYPE_DFFT;
	} else if (val & BANYAN_FFT_DATA) {
		dataType = DATA_TYPE_FFT;
	} else if (val & BANYAN_DFFT_PEAK_DATA) {
		dataType = DATA_TYPE_DFFT_PEAK;
	} else if (val & BANYAN_DSRAW_DATA) {
		dataType = DATA_TYPE_DSRAW;
	} else {
		dataType = DATA_TYPE_MAX;
	}

	return dataType;
}

uint8_t Radar_GetDataTypeByRegisterList(void) {
	uint8_t dataType = DATA_TYPE_FFT;
	uint16_t val     = BANYAN_FFT_DATA;

	uint16_t loop = 0;
	while (((InitChipRegListStart0[loop].addr != 0xFF) || (InitChipRegListStart0[loop].val != 0xFFFF))) {
		if ((uint8_t)InitChipRegListStart0[loop].addr == BANYAN_DIG_FUN_SWITCH) {
			val = InitChipRegListStart0[loop].val;
		}
		loop++;
	}

	if (val & BANYAN_DFFT_DATA) {
		dataType = DATA_TYPE_DFFT;
	} else if (val & BANYAN_FFT_DATA) {
		dataType = DATA_TYPE_FFT;
	} else if (val & BANYAN_DFFT_PEAK_DATA) {
		dataType = DATA_TYPE_DFFT_PEAK;
	} else if (val & BANYAN_DSRAW_DATA) {
		dataType = DATA_TYPE_DSRAW;
	} else {
		dataType = DATA_TYPE_MAX;
	}

	return dataType;
}

void Radar_Enter_LowPWR(uint16_t devAddr) {
	uint16_t loop = 0;
	while ((RegList_LowPWR[loop].addr) || (RegList_LowPWR[loop].val != 0x0000)) {
		SPI4_Write(devAddr, (uint8_t)(RegList_LowPWR[loop].addr), RegList_LowPWR[loop].val);
		loop++;
	}
}

void Radar_Exit_LowPWR(uint16_t devAddr) {
	uint16_t loop = 0;
	while ((RegList_NormalPWR[loop].addr) || (RegList_NormalPWR[loop].val != 0x0000)) {
		SPI4_Write(devAddr, (uint8_t)(RegList_NormalPWR[loop].addr), RegList_NormalPWR[loop].val);
		loop++;
	}
}

void Radar_Write_ABD_MTT_Para(int16_t sysMode) {
	uint16_t loop = 0;

	if (sysMode == SYS_MODE_ABD) {
		while (((InitRegList_ABD[loop].addr != 0xFF) || (InitRegList_ABD[loop].val != 0xFFFF))) {
			SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitRegList_ABD[loop].addr), InitRegList_ABD[loop].val);
			loop++;
		}
	} else if (sysMode == SYS_MODE_MTT) {
		while (((InitRegList_MTT[loop].addr != 0xFF) || (InitRegList_MTT[loop].val != 0xFFFF))) {
			SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitRegList_MTT[loop].addr), InitRegList_MTT[loop].val);
			loop++;
		}
	}
}

// void Radar_Init(void)
//{
//     uint16_t loop = 0;

// #ifdef CONFIG_DEBUG
//     printf("radar flash value:\r\n");
//     while(InitRegList[loop].addr)
//     {
//         printf("%02X=%04X\r\n", InitRegList[loop].addr, InitRegList[loop].val);
//         loop++;
//     }
//     loop = 0;
// #endif

//
//    while((InitRegList[loop].addr || ((InitRegList[loop].addr == 0x00) && (InitRegList[loop+1].addr == 0x01))))
//    {
//        SPI4_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(InitRegList[loop].addr), InitRegList[loop].val);
//        loop++;
//
//    }

// #ifdef CONFIG_DEBUG
//     loop = 0;
//     printf("radar ic value:\r\n");
//	  uint16_t val = 0;
//     while(InitRegList[loop].addr)
//     {
//		    SPI4_Read(I2C_ADDR_BanYan_Chip0, InitRegList[loop].addr, &val);
//         printf("%02X=%04X\r\n", InitRegList[loop].addr, val);
//         loop++;
//     }
// #endif
// }


void Radar_UpdateReg(uint16_t devAddr, uint16_t addr, uint16_t val) /*currently only update existing reg*/
{
	uint16_t loop           = 0;
	RADAR_REG_T *pRegConfig = NULL;
	RADAR_REG_T *pRegStart  = NULL;

	if (devAddr == I2C_ADDR_BanYan_Chip0) {
		pRegConfig = InitChipRegListConfig0;
		pRegStart  = InitChipRegListStart0;
	} else {
		return;
	}

	while (!((pRegConfig[loop].addr == 0xFF) && (pRegConfig[loop].val == 0xFFFF))) {
		if (pRegConfig[loop].addr == addr) {
			pRegConfig[loop].val = val;
			Config_NeedFlashWrite();
			return;
		}
		loop++;
	}

	loop = 0;
	while (!((pRegStart[loop].addr == 0xFF) && (pRegStart[loop].val == 0xFFFF))) {
		if (pRegStart[loop].addr == addr) {
			pRegStart[loop].val = val;
			Config_NeedFlashWrite();
			return;
		}
		loop++;
	}
}

void *Radar_GetChipRegListConfigParaAddr(uint8_t chip) {
	return (void *)&InitChipRegListConfig0;
}

uint32_t Radar_GetChipRegListConfigParaLen(uint8_t chip) {
	return sizeof(InitChipRegListConfig0);
}

void *Radar_GetChipRegListStartParaAddr(uint8_t chip) {
	return (void *)&InitChipRegListStart0;
}

uint32_t Radar_GetChipRegListStartParaLen(uint8_t chip) {
	return sizeof(InitChipRegListStart0);
}
