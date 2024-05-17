/**
  ******************************************************************************
  * @file           : gaincalibration.c
  * @author         : iclm team
  * @brief          : gaincalibration
  ******************************************************************************
  */
#include <stdio.h>
#include "gaincalibration.h"
#include "banyan.h"
#include "banyan_para.c"
#include "platform.h"
#include "system.h"

int16_t RawValueMax[CHANNEL_MAX];
int16_t RawValueMin[CHANNEL_MAX];
int16_t RawValueMaxAfterCalibraion[CHANNEL_MAX];

GAIN_CALIBRATION_STATE GainCalibrationState = GAIN_CALIBRATION_STATE_NONE;
IVCM_CALIBRATION_STATE IVCMCalibrationState = IVCM_CALIBRATION_STATE_NONE;
int CalibraitionFrameCount = 0;

uint8_t gainCalibration = 0;
//For setting up ivcm calibration
uint8_t IVCMSel_N_I_CH1 = 0x0F;
uint8_t IVCMSel_N_I_CH2 = 0x0F;
uint8_t IVCMSel_N_Q_CH1 = 0x0F;
uint8_t IVCMSel_N_Q_CH2 = 0x0F;
uint8_t IVCMSel_P_I_CH1 = 0x0F;
uint8_t IVCMSel_P_I_CH2 = 0x0F;
uint8_t IVCMSel_P_Q_CH1 = 0x0F;
uint8_t IVCMSel_P_Q_CH2 = 0x0F;

//For setting up GAIN calibration
uint8_t TXCalibrationSuccess = 0;//successful,the registers of 0x72, 0x6c and 0x6d will be updated
uint16_t RX1CalibrationGainCode = 0x07;// successful,the registers of 0x6e will be updated
uint16_t RX2CalibrationGainCode = 0x07;// successful,the registers of 0x6e will be updated
RADAR_REG_T regList[MAX_REG_NUM] = {0};

RADAR_REG_T UpdateRegList[23] =
{
    {0x42, 0x0010},
    {0x43, 0x8290},
    {0x44, 0x0020},
    {0x47, 0x1001},
    {0x48, 0x4050},
	{0x4A, 0x5013},
    {0x4B, 0x000E},
    {0x4C, 0xE28D},
    {0x5B, 0x0022},
    {0x5C, 0x0022},
    {0x6D, 0x99C0},
	{0x6E, 0xC3FC},
    {0x04, 0x020C},
    {0x09, 0x6901},
    {0x0A, 0xC200},
    {0x0B, 0xC000},
	{0x02, 0x2000},
    {0x05, 0x0100},
    {0x0D, 0x0000},
    {0x0E, 0x2000},
    {0x01, 0x8222},
	{0x41, 0xC864},
	{0x00, 0x0000}
};

void CreateNewRegList()
{
	uint16_t loop = 0;
	uint32_t f_T_FSM01_US = 0;
	uint32_t f_Step01_Hz = 0;
	uint32_t f_Step10_Hz = 0;
	uint16_t reg55Val = 0;
	uint16_t reg56Val = 0;
	uint16_t reg57Val = 0;
	uint16_t reg58Val = 0;
	
	uint16_t reg55Index = 0;
	uint16_t reg56Index = 0;
	uint16_t reg57Index = 0;
	uint16_t reg58Index = 0;
	
    while(InitRegList[loop].addr) 
    {   
		regList[loop] = InitRegList[loop];
        if (loop < REG_FIX_NUM)
        {
            loop++;
            continue;
        }
        
		if(InitRegList[loop].addr == 0x47)
		{
			f_T_FSM01_US |= (InitRegList[loop].val & 0x0FFF) << 16;
		}
		
		if(InitRegList[loop].addr == 0x48)
		{
			f_T_FSM01_US |= InitRegList[loop].val;
		}
		
		if(InitRegList[loop].addr == 0x55)
		{
			reg55Index = loop;
			f_Step01_Hz |= InitRegList[loop].val << 16;
		}
		
		if(InitRegList[loop].addr == 0x56)
		{
			reg56Index = loop;
			f_Step01_Hz |= InitRegList[loop].val;
			f_Step01_Hz = f_Step01_Hz * (f_T_FSM01_US - 1) / (410 * 200 - 1);
			reg55Val = (uint16_t)((f_Step01_Hz & 0xFFFF0000) >> 16);
			reg56Val = (uint16_t)(f_Step01_Hz & 0x0000FFFF);
		}
		
		if(InitRegList[loop].addr == 0x57)
		{
			reg57Index = loop;
			f_Step10_Hz |= InitRegList[loop].val << 16;
		}
		
		if(InitRegList[loop].addr == 0x58)
		{
			reg58Index = loop;
			f_Step10_Hz |= InitRegList[loop].val;
			f_Step10_Hz = f_Step10_Hz * (f_T_FSM01_US - 1) / (410 * 200 - 1);
			reg57Val = (uint16_t)((f_Step10_Hz & 0xFFFF0000) >> 16);
			reg58Val = (uint16_t)(f_Step10_Hz & 0x0000FFFF);
		}
			
		uint16_t checkloop = 0;
		while(UpdateRegList[checkloop].addr) 
		{
			if (regList[loop].addr == UpdateRegList[checkloop].addr)
			{
				regList[loop].val = UpdateRegList[checkloop].val;
				break;
			}
		}

        loop++;
    }
	regList[reg55Index].val = reg55Val;
	regList[reg56Index].val = reg56Val;
	regList[reg57Index].val = reg57Val;
	regList[reg58Index].val = reg58Val;
	
}

uint8_t IsGainCalibration(void)
{
	return gainCalibration;
}

void SetArrayValue(int16_t *a, uint16_t n, int16_t v)
{
	for (uint16_t i = 0; i < n; i++)
	{
		a[i] = v;
	}
}

void StartGainCalibration(void)
{
	uint16_t loop = 0;
	IVCMSel_N_I_CH1 = 0x0F;
	IVCMSel_N_I_CH2 = 0x0F;
	IVCMSel_N_Q_CH1 = 0x0F;
	IVCMSel_N_Q_CH2 = 0x0F;
	IVCMSel_P_I_CH1 = 0x0F;
	IVCMSel_P_I_CH2 = 0x0F;
	IVCMSel_P_Q_CH1 = 0x0F;
	IVCMSel_P_Q_CH2 = 0x0F;
	
	TXCalibrationSuccess = 0;
	RX1CalibrationGainCode = 0x07;
	RX2CalibrationGainCode = 0x07;
	
	CreateNewRegList();
	while(regList[loop].addr) 
    {
        I2C_Write(I2C_ADDR_BanYan_Chip0, (uint8_t)(regList[loop].addr), regList[loop].val);
        loop++;
    }
	System_Reconfig();
	
	gainCalibration = 1;
	GainCalibrationState = GAIN_CALIBRATION_STATE_IVCM;
	IVCMCalibrationState = IVCM_CALIBRATION_STATE_CALIBRATION;
	CalibraitionFrameCount = -2;
	
	SetArrayValue(RawValueMax, CHANNEL_MAX, (int16_t)SHORT_MIN);
	SetArrayValue(RawValueMin, CHANNEL_MAX, (int16_t)SHORT_MAX);
	SetArrayValue(RawValueMaxAfterCalibraion, CHANNEL_MAX, (int16_t)SHORT_MIN);
}

void StopGainCalibraion(void)
{
	if (gainCalibration)
	{

	}

	gainCalibration = 0;
}
