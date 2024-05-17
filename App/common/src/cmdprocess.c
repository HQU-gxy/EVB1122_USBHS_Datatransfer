/**
  ******************************************************************************
  * @file           : cmdprocess.c
  * @author         : iclm team
  * @brief          : command process module
  ******************************************************************************
  */
#include "stm32_spi.h"
#include <stdio.h>
#include <string.h>
#include "global_conf.h"
#include "platform.h"
#include "cmdprocess.h"
#include "banyan.h"
#include "config.h"
#include "utilities.h"
#include "system.h"
#ifdef STM32_PLATFORM
#include "I2CStressTest.h"
#include "usbd_cdc_if.h"
#include "usb_transfer.h"
#include "rtos.h"
#include "cmsis_os.h"
#endif

SemaphoreHandle_t g_McuSpecialFuncSig;
PD_CALIBRATION_PARAM_T PD_CalibrationPara = {0};


static uint8_t enableResetAdc = 0;
static uint8_t PreSystemCofig = 0;
static uint16_t resetDelayMs = RESET_DELAY_TIME;
uint8_t gDataprocessFlag = 0;

static CMD_PARSE_T CmdDataParse;
static uint8_t CmdModeFlag = 0;
static uint8_t DebugModeFlag = 0;

static uint8_t CmdHead[CMD_HEAD_MAX] = {0xFD, 0xFC, 0xFB, 0xFA};
static uint8_t CmdTail[CMD_TAIL_MAX] = {0x04, 0x03, 0x02, 0x01};
static uint8_t CmdAck[CMD_LEN_MAX] __ALIGN(4) = {0};

static uint16_t masterProtocolVer = 0;

static uint8_t enableNopConfig = 0;
static uint8_t nopConfigStartRegisterIndex = 0;
static uint8_t nopConfigStopRegisterIndex = 0;
static uint16_t nopConfigStartMs = RESET_DELAY_TIME;
static uint16_t nopConfigStopMs = RESET_DELAY_TIME + 2;
int32_t nopConfigStartRegister[16] = {0};
int32_t nopConfigStopRegister[16] = {0};

#ifdef STM32_PLATFORM
static uint8_t* pI2CTestData = NULL;
static uint16_t I2CTestDataCurIndex = 0;
static uint8_t stressTestReady = 1;
#endif

/* 目标追踪部分变量定义 */
uint8_t TrackMode = 2;    //默认多目标模式

void* System_GetPDCaliParaAddr(void)
{
    return (void*)&PD_CalibrationPara;
}

uint32_t System_GetPDCaliParaLen(void)
{
    return sizeof(PD_CalibrationPara);
}


void PD_ParaInit(void)
{
    PD_CalibrationPara.calibrated_PhaseDiff = DEFAULT_PDPARA_PHASEDIFF;
    PD_CalibrationPara.PDCalib1_Temper = DEFAULT_PDPARA1_TEMPER;
    PD_CalibrationPara.PDCalib1_Power = DEFAULT_PDPARA1_POWER; 
    PD_CalibrationPara.PDCalib1_Refval = DEFAULT_PDPARA1_REFVAL;
    PD_CalibrationPara.PDCalib1_SlopeVal = DEFAULT_PDPARA1_SLOPE; 
    PD_CalibrationPara.PDCalib2_Temper =   DEFAULT_PDPARA2_TEMPER;
    PD_CalibrationPara.PDCalib2_Power =    DEFAULT_PDPARA2_POWER; 
    PD_CalibrationPara.PDCalib2_Refval =   DEFAULT_PDPARA2_REFVAL;
    PD_CalibrationPara.PDCalib2_SlopeVal = DEFAULT_PDPARA2_SLOPE;
}

/********************************************
 @名称；CmdProc_AdcReset
 @功能：Nop期间重置ADC
 @参数：none
 @返回：none
*********************************************/
void CmdProc_AdcReset(void)
{
    if (enableResetAdc && CmdModeFlag == 0)
    {                       
        Delay(resetDelayMs);
        uint16_t val = 0;
        SPI4_Write(I2C_ADDR_BanYan_Chip0, 0x67, 0x1E00);   
        SPI4_Read(I2C_ADDR_BanYan_Chip0, 0x67, &val);    
        SPI4_Write(I2C_ADDR_BanYan_Chip0, 0x67, 0x1E40);   
        if(g_TxCount > 1)
        {
            SPI4_Write(I2C_ADDR_BanYan_Chip1, 0x67, 0x1E00);   
            SPI4_Read(I2C_ADDR_BanYan_Chip1, 0x67, &val);    
            SPI4_Write(I2C_ADDR_BanYan_Chip1, 0x67, 0x1E40);   
        }
    }
}


/********************************************
 @名称；CmdProc_NopConfig
 @功能：Frame Nop期间相关配置
 @参数：none
 @返回：none
*********************************************/
void CmdProc_NopConfig(void)
{
    if (enableNopConfig && CmdModeFlag == 0)
    {     
        uint16_t loop = 0;
        Delay(nopConfigStartMs);
        for(loop = 0; loop < nopConfigStartRegisterIndex; loop++)
        {
            int32_t val = nopConfigStartRegister[loop];
            CMD_REG_T *cmdReg = (CMD_REG_T *)(&val);
            SPI4_Write(I2C_ADDR_BanYan_Chip0, cmdReg->addr, cmdReg->val); 
            //printf("CmdProc_NopConfig chip0: g_ChannelCount = %d  addr=0x%02x val=0x%02x\r\n", g_ChannelCount, cmdReg->addr, cmdReg->val);
            if(g_TxCount > 1)
            {
                SPI4_Write(I2C_ADDR_BanYan_Chip1, cmdReg->addr, cmdReg->val); 
                //printf("CmdProc_NopConfig chip1: addr=0x%02x val=0x%02x\r\n", cmdReg->addr, cmdReg->val);
            }
            
        }
        Delay(nopConfigStopMs - nopConfigStartMs);
        for(loop = 0; loop < nopConfigStopRegisterIndex; loop++)
        {
            int32_t val = nopConfigStopRegister[loop];
            CMD_REG_T *cmdReg = (CMD_REG_T *)(&val);
            SPI4_Write(I2C_ADDR_BanYan_Chip0, cmdReg->addr, cmdReg->val); 
            //printf("CmdProc_NopConfig chip0: addr=0x%02x val=0x%02x\r\n", cmdReg->addr, cmdReg->val);
            if(g_TxCount > 1)
            {
                SPI4_Write(I2C_ADDR_BanYan_Chip1, cmdReg->addr, cmdReg->val); 
                //printf("CmdProc_NopConfig chip1: addr=0x%02x val=0x%02x\r\n", cmdReg->addr, cmdReg->val);
            }
        }
    }
}

/********************************************
 @名称；FillCmdAck
 @功能：命令行ack填充
 @参数：data，ack数据
        dataLen，数据长度
        cmdType，命令类型
        status，cmd上报状态
@返回：index，待发送数据长度
*********************************************/
uint16_t FillCmdAck(uint16_t *data, uint16_t dataLen, uint16_t cmdType, uint16_t status)
{
    uint16_t index = 0;

    if (dataLen * sizeof(uint16_t) > (CMD_LEN_MAX - CMD_OVERHEAD_LEN))
    {
        return 0;
    }
    
    memcpy(&CmdAck[index], CmdHead, sizeof(CmdHead));
    index += sizeof(CmdHead);
    
    if (data == NULL)
    {
        *((uint16_t*)(&CmdAck[index])) = CMD_TYPE_LEN + CMD_STATUS_LEN;
    }
    else if(masterProtocolVer != 0)
    {
        *((uint16_t*)(&CmdAck[index])) = CMD_TYPE_LEN + CMD_STATUS_LEN + (dataLen * sizeof(uint16_t));
    }
    else
    {
        *((uint16_t*)(&CmdAck[index])) = CMD_TYPE_LEN + (dataLen * sizeof(uint16_t));
    }
    index += sizeof(uint16_t);

    *((uint16_t*)(&CmdAck[index])) = cmdType | CMD_ACK_TYPE;
    index += sizeof(uint16_t);

    if(masterProtocolVer != 0 || data == NULL)
    {
        if(masterProtocolVer == 0)
        {
            status = (status == ACK_OK) ? ACK_OK_OLD : ACK_FAIL_OLD;
        }
        *((uint16_t*)(&CmdAck[index])) = status;
        index += sizeof(uint16_t);
    }
    
    if (data != NULL)
    {
        memcpy(&CmdAck[index], data, dataLen*sizeof(uint16_t));
        index += dataLen * sizeof(uint16_t);   
    }

    memcpy(&CmdAck[index], CmdTail, sizeof(CmdTail));
    index += sizeof(CmdTail);
    
    return index;
}

/********************************************
 @名称；IAP_FillCmdACK
 @功能：IAP 命令ACK
 @参数：CmdType，命令类型
        pData，数据
        dataLen，数据长度
        ACK_Status，ACK状态
@返回：pos，待发送数据长度
*********************************************/
static uint16_t IAP_FillCmdACK(uint16_t CmdType, uint8_t* pData, uint16_t dataLen, uint16_t ACK_Status)
{
    uint16_t pos = 0;
    uint16_t len = 4;
    uint32_t i= 0;

    /* 填充头部 */
    CmdAck[pos++] = 0xFD;
    CmdAck[pos++] = 0xFC;
    CmdAck[pos++] = 0xFB;
    CmdAck[pos++] = 0xFA;

    if(pData != NULL)
    {
        len += dataLen;
    }

    CmdAck[pos++] = (len & 0x00FF) >> 0;
    CmdAck[pos++] = (len & 0xFF00) >> 8;

    CmdAck[pos++] = ((CmdType | CMD_ACK_TYPE) & 0x00FF) >> 0;
    CmdAck[pos++] = ((CmdType | CMD_ACK_TYPE) & 0xFF00) >> 8;

    CmdAck[pos++] = (ACK_Status & 0x00FF) >> 0;
    CmdAck[pos++] = (ACK_Status & 0xFF00) >> 8;

    if(pData != NULL)
    {
        for(i = 0; i < dataLen; i++)
        {
            CmdAck[pos+i] = pData[i];
        }
    }
    pos += dataLen;

    CmdAck[pos++] = 0x04;
    CmdAck[pos++] = 0x03;
    CmdAck[pos++] = 0x02;
    CmdAck[pos++] = 0x01;

    return pos;
}

/********************************************
 @名称；DoAdcReset
 @功能：重置ADC
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，待发送数据长度
*********************************************/
//static uint16_t DoAdcReset(CMD_T *cmd, uint32_t cmdLen)
//{
//    uint16_t ackLen = 0;
//    uint16_t status = ACK_OK;

//    if (NULL == cmd || 0 == cmdLen)
//    {
//        return 0;
//    }
//        
//    if (CmdModeFlag == 1)
//    {
//        enableResetAdc = (uint8_t)cmd->cmdData[0];
//        resetDelayMs = cmd->cmdData[1];
//#if defined(CONFIG_DEBUG) && defined(STM32_PLATFORM)
//        printf("enableResetAdc:%d resetDelayMs:%d\r\n", enableResetAdc, resetDelayMs);
//#endif
//    } 
//    else 
//    {
//        status = ACK_FAIL;
//    }
//    
//    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);

//    return ackLen;
//}

/********************************************
 @名称；FillReadSnCmdAck
 @功能：返回SN命令
 @参数：data，命令数据
        dataLen，命令数据长度
        cmdType， 命令类型
        status， 回复状态
 @返回：ackLen，待发送数据长度
*********************************************/
static uint16_t FillReadSnCmdAck(uint16_t *data, uint16_t dataLen, uint16_t cmdType, uint16_t status)
{
    uint16_t index = 0;

    if (dataLen * sizeof(uint16_t) > (CMD_LEN_MAX - CMD_OVERHEAD_LEN))
    {
        return 0;
    }
    
    //head: 0xFD 0xFC 0xFB 0xFA
    memcpy(&CmdAck[index], CmdHead, sizeof(CmdHead));
    index += sizeof(CmdHead);
    
    //data Len
    *((uint16_t*)(&CmdAck[index])) = CMD_TYPE_LEN + CMD_STATUS_LEN + CMD_STR_LEN + dataLen;
    index += sizeof(uint16_t);
    
    //cmd type
    *((uint16_t*)(&CmdAck[index])) = cmdType | CMD_ACK_TYPE;
    index += sizeof(uint16_t);
    
    //status: fail or ok
    *((uint16_t*)(&CmdAck[index])) = status;
    index += sizeof(uint16_t);
    
    //data
    if (data != NULL && dataLen > 0)
    {
        *((uint16_t*)(&CmdAck[index])) = dataLen;
        index += sizeof(uint16_t);
        memcpy(&CmdAck[index], data, dataLen);
        index += dataLen;
    }
    
    //tail
    memcpy(&CmdAck[index], CmdTail, sizeof(CmdTail));
    index += sizeof(CmdTail);
    
    return index;
}

/********************************************
 @名称；DoWriteReg
 @功能：向SoC写入寄存器
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，待发送数据长度
*********************************************/
static uint16_t DoWriteReg(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    uint16_t regNum = 0;
    uint16_t loop = 0;
    uint16_t regVal = 0;
    uint16_t devAddress = I2C_ADDR_BanYan_Chip0;
    CMD_REG_T *cmdReg = NULL;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {
        devAddress = cmd->cmdData[0] << 1;
        regNum = (cmdLen - CMD_TYPE_LEN - CMD_DEV_ADDR_LEN) / sizeof(CMD_REG_T);
        for (loop = 0; loop < regNum; loop++) 
        {
            cmdReg = (CMD_REG_T*)((uint8_t*)(cmd->cmdData) + CMD_DEV_ADDR_LEN + (loop * sizeof(CMD_REG_T)));
            SPI4_Write(devAddress, (uint8_t)(cmdReg->addr), cmdReg->val);
            if(devAddress == I2C_ADDR_BanYan_Chip0)
            {//update master register
                Radar_UpdateReg(devAddress, cmdReg->addr, cmdReg->val);
            }
#ifdef STM32_PLATFORM
            HAL_Delay(10); 
#endif
            
            SPI4_Read(devAddress, (uint8_t)(cmdReg->addr), &regVal);
            if (regVal != cmdReg->val) 
            {
                status = ACK_FAIL;
            }
        }
    } 
    else 
    {
        status = ACK_FAIL;
    }
    
    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
    
    return ackLen;
}

/********************************************
 @名称；DoReadReg
 @功能：回读SoC寄存器
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，待发送数据长度
*********************************************/
static uint16_t DoReadReg(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    uint16_t regNum = 0;
    uint16_t loop = 0;
    uint16_t *readBuf = NULL;
    uint16_t devAddress = I2C_ADDR_BanYan_Chip0;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {
        devAddress = cmd->cmdData[0] << 1;
        regNum = (cmdLen - CMD_TYPE_LEN - CMD_DEV_ADDR_LEN) / CMD_REG_ADDR_LEN;
        
        if (regNum > CMD_REG_MAX)
        {
            regNum = 0;
            status = ACK_FAIL;
        }
        else
        {
            uint16_t dataIndex = (masterProtocolVer != 0) ? CMD_DATA_POS : CMD_DATA_POS_OLD;
            readBuf = (uint16_t*)(&CmdAck[dataIndex]);
            for (loop = 0; loop < regNum; loop++) 
            {
                uint16_t regVal = 0;
                SPI4_Read(devAddress, (uint8_t)(cmd->cmdData[loop + 1]), &regVal);
                readBuf[loop] = regVal;
            }
        }   
    } 
    else 
    {
        status = ACK_FAIL;
    }
    
    ackLen = FillCmdAck(readBuf, regNum, cmd->cmdType, status);

    return ackLen;
}

__weak int8_t MTT_ParaUpdate(uint16_t type, int32_t val)
{
    return -1;
}

__weak int8_t fftzerofill_ParaUpdate(uint16_t type, int32_t val)
{
    return -1;
}

/********************************************
 @名称；NopCofig_ParaUpdate
 @功能：Nop期间配置相关参数
 @参数：type，类型
        val，数值
 @返回：0/-1，指示配置状态
*********************************************/
int8_t NopCofig_ParaUpdate(uint16_t type, int32_t val)
{
    switch (type)  
    {
        case NOPCONFIG_ENABLE:
            enableNopConfig = (int8_t)val;
            nopConfigStartRegisterIndex = 0;
            nopConfigStopRegisterIndex = 0;
            //printf("enablenopreg:%d\r\n", enableNopConfig);
            break;
        case NOPCONFIG_STARTTIME:
            nopConfigStartMs = (uint16_t)val;
            //printf("starttime:%d\r\n", nopConfigStartMs);
            break;
        case NOPCONFIG_STOPTIME:
            nopConfigStopMs = (uint16_t)val;
            //printf("stoptime:%d\r\n", nopConfigStopMs);
            break;
        case NOPCONFIG_STARTREGISTER:
            nopConfigStartRegister[nopConfigStartRegisterIndex++] = val;
            break;
        case NOPCONFIG_STOPREGISTER:
            nopConfigStopRegister[nopConfigStopRegisterIndex++] = val;
            break;        
        default:
            return -1;
    }
    return 0;
}

/********************************************
 @名称；DoTrackOneMod
 @功能：切换多目标追踪demo目标个数
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoTrackOneMod(CMD_T* cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
    
    TrackMode = 1;
    
    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
    return ackLen;
}

/********************************************
 @名称；DoTrackTwoMod
 @功能：切换多目标追踪demo目标个数
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoTrackTwoMod(CMD_T* cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    
    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
    
    TrackMode = 2;
    
    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
    return ackLen;
}

/********************************************
 @名称；DoWritePara
 @功能：更新SysPara
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoWritePara(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    uint16_t paraNum = 0;
    uint16_t loop = 0;
    CMD_PARA_T *cmdPara = NULL;
    int8_t ret = 0;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {
        paraNum = (cmdLen - CMD_TYPE_LEN) / sizeof(CMD_PARA_T);
        for (loop = 0; loop < paraNum; loop++) 
        {
            cmdPara = (CMD_PARA_T*)((uint8_t*)(cmd->cmdData) + (loop * sizeof(CMD_PARA_T)));
            switch (cmd->cmdType)
            {
                case WRITE_MTT_CMD: 
                    ret = MTT_ParaUpdate(cmdPara->type, cmdPara->val);
                    break;

                case WRITE_SYS_CMD: 
#ifdef SUPPORT_DYNAMIC_SYS_MODE
//                    if (cmdPara->type == SYS_SYSTEM_MODE &&
//                        System_GetSysMode() != (int16_t)cmdPara->val)
//                    {
//                        Config_ReloadAlgoRegister((int16_t)cmdPara->val);
//                        Radar_Init();
//                    }
#endif
                    if(cmdPara->type == SYS_PRE_START)
                    {
                        PreSystemCofig = 1;
                        System_Reconfig((uint8_t)(cmdPara->val)); 
                        ret = 0;
                    }
                    else
                    {	
                        ret = System_ParaUpdate(cmdPara->type, cmdPara->val);
                    }
                    break;

                case MCU_SPECIAL_FUNC_CMD:
                    MCU_FuncParaUpdate(cmdPara->type, cmdPara->val);
                    break;

                case FFT_ZEROFILL_CMD:
                    ret = fftzerofill_ParaUpdate(cmdPara->type, cmdPara->val);
                    break;
//                case NOP_CONFIG_CMD:
//                    ret = NopCofig_ParaUpdate(cmdPara->type, cmdPara->val);
//                    break;
                default:
                    ret = -1;
                    break;
            }
            if (ret)
            {
                status = ACK_FAIL;
            }
            else
            {
                Config_NeedFlashWrite();     /* 注释掉这行，因为我不需要在修改完sysPara后继续将修改后的配置写入flash */  
            }
        }
    } 
    else 
    {
        status = ACK_FAIL;
    }
    if(sysPara.useMcuPackWrapper)
    {
        if(cmd->cmdType == MCU_SPECIAL_FUNC_CMD)
        {
            xSemaphoreGive(g_McuSpecialFuncSig);
        }
        
        if(CmdModeFlag)
        {
            ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
        }
        else
        {
            ackLen = 0; // For MCU_SPECIAL_FUNC_CMD, don't need to give the response in non command mode
        }
    }
    else
    {
        ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
    }

    
    return ackLen;
}

__weak int32_t MTT_ParaRead(uint16_t type)
{
    return 0x7fffffff; /*invalid value*/
}

/********************************************
 @名称；DoReadPara
 @功能：回读SysPara
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoReadPara(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    uint16_t paraNum = 0;
    uint16_t loop = 0;
    uint32_t *readBuf = NULL;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {
        paraNum = (cmdLen - CMD_TYPE_LEN) / CMD_PARA_NAME_LEN;
        
        if (paraNum > CMD_PARA_MAX)
        {
            paraNum = 0;
            status = ACK_FAIL;
        }
        else
        {
            uint16_t dataIndex = (masterProtocolVer != 0) ? CMD_DATA_POS : CMD_DATA_POS_OLD;
            readBuf = (uint32_t*)(&CmdAck[dataIndex]);
            for (loop = 0; loop < paraNum; loop++) 
            {
                switch (cmd->cmdType)
                {
                    case READ_MTT_CMD:
                        readBuf[loop] = MTT_ParaRead(cmd->cmdData[loop]);
                        break;
                    
                    case READ_SYS_CMD:
                        readBuf[loop] = System_ParaRead(cmd->cmdData[loop]);
                        break;
                    
                    default:
                        break;
                }
            }
        }   
    } 
    else 
    {
        status = ACK_FAIL;
    }
    
    ackLen = FillCmdAck((uint16_t*)readBuf, paraNum*2, cmd->cmdType, status);
    
    return ackLen;
}

/********************************************
 @名称；DoWriteSn
 @功能：写入SN号
 @参数：cmd，命令数据
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoWriteSn(CMD_T *cmd)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
	
    if (NULL == cmd)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {   
        uint16_t nLen = cmd->cmdData[0];
        if(nLen > SN_LEN)
        {
            status = ACK_FAIL;
        }
        else
        {
            System_SetSysSnInfo(nLen, (uint8_t*)(cmd->cmdData + 1));
            Config_NeedFlashWrite();
        }  
    } 
    else 
    {
        status = ACK_FAIL;
    }
    
    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
    return ackLen;
}

/********************************************
 @名称；DoReadSN
 @功能：回读SN号
 @参数：cmd，命令数据
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoReadSn(CMD_T *cmd)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;
    SN_INFO_T sn;
	sn.snLen = 0;
	
    if (NULL == cmd)
    {
        return 0;
    }
        
    if (CmdModeFlag) 
    {   
		memcpy(&sn, System_GetSysSnInfoAddr(), sizeof(SN_INFO_T));	
    } 
    else 
    {
        status = ACK_FAIL;
    }
    
    ackLen = FillReadSnCmdAck((uint16_t*)sn.cSn, sn.snLen, cmd->cmdType, status);
    return ackLen;
}

/********************************************
 @名称；DoCascadingMode
 @功能：级联模式设置
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
static uint16_t DoCascadingMode(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t status = ACK_OK;

    if (NULL == cmd || 0 == cmdLen)
    {
        return 0;
    }
#ifdef STM32_PLATFORM      
    if (CmdModeFlag == 1)
    {
        //uint8_t cascadingMode = (uint8_t)cmd->cmdData[0];
        g_ChipCount = (uint8_t)cmd->cmdData[0];
        g_RxCount = cmd->cmdData[1]; 
        g_TxCount = (g_RxCount + 1) / 2;
        
        if((cmdLen - CMD_TYPE_LEN) == 6)
        {
            g_TxCount = cmd->cmdData[2];
        }
    } 
    else 
    {
        status = ACK_FAIL;
    }
#endif  
    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);

    return ackLen;
}

#ifdef STM32_PLATFORM 
/********************************************
 @名称；ResetI2CTestDataBuf
 @功能：重置I2C压测数据buffer
 @参数：none
 @返回：none
*********************************************/
void ResetI2CTestDataBuf()
{
    if(pI2CTestData != NULL)
    {
        free(pI2CTestData);
        pI2CTestData = NULL;
    }
}
#endif 

/********************************************
 @名称；StartI2CTest
 @功能：开始I2C压测
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
//static uint16_t StartI2CTest(CMD_T *cmd, uint32_t cmdLen)
//{
//    uint16_t ackLen = 0;
//    uint16_t status = ACK_OK;

//    if (NULL == cmd || 0 == cmdLen)
//    {
//        return 0;
//    }
//#ifdef STM32_PLATFORM 	
//    uint16_t dataLen = 0;
//    uint16_t packIndex = 0;
//    dataLen = cmd->cmdData[0];
//    packIndex = cmd->cmdData[1];
//    
//    if(packIndex == 0)
//    {
//        if(I2CStressTest_IsStressTestProcessRunning())
//        {
//            stressTestReady = (I2CStressTest_ForceStopI2CStressTestProcess() != 0) ? 1 : 0;
//        }
//        if(stressTestReady)
//        {
//            ResetI2CTestDataBuf();
//            pI2CTestData = malloc(dataLen);
//            I2CTestDataCurIndex = 0;	
//        }
//    }
//    if(stressTestReady && pI2CTestData != NULL)
//    {
//        memcpy(pI2CTestData + I2CTestDataCurIndex, (uint8_t*)(cmd->cmdData) + 4, cmdLen - CMD_TYPE_LEN - 4);
//        I2CTestDataCurIndex += cmdLen - CMD_TYPE_LEN - 4;
//            
//        if(I2CTestDataCurIndex == dataLen)
//        {
//            I2CStressTest_ResetRunningState();
//            I2CTEST_DATA_T I2CTestData = {0};
//            I2CTestData.buf = pI2CTestData;
//            I2CTestData.len = I2CTestDataCurIndex;
//            I2CStressTest_Send2I2CTestDataQueue(&I2CTestData);
//        }
//    }
//    
//    if(!stressTestReady)
//    {
//        status = ACK_FAIL;
//    }
//#endif     
//    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
//    return ackLen;
//}

/********************************************
 @名称；StopI2CTest
 @功能：停止I2C压测
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
//static uint16_t StopI2CTest(CMD_T *cmd, uint32_t cmdLen)
//{
//    uint16_t ackLen = 0;
//    uint16_t status = ACK_OK;
//#ifdef STM32_PLATFORM 
//    if(I2CStressTest_IsStressTestProcessRunning())
//    {
//        status = (I2CStressTest_ForceStopI2CStressTestProcess() != 0) ? ACK_OK : ACK_FAIL;
//    }
//#endif   
//    ackLen = FillCmdAck(NULL, 0, cmd->cmdType, status);
//    return ackLen;	
//}

/********************************************
 @名称；GetI2CTestResult
 @功能：取得I2C压测结果
 @参数：cmd，命令数据
        cmdLen，命令数据长度
 @返回：ackLen，上报数据长度
*********************************************/
//static uint16_t GetI2CTestResult(CMD_T *cmd, uint32_t cmdLen)
//{
//    uint16_t ackLen = 0;
//    uint16_t status = ACK_OK;
//    uint8_t buf[10] = {0};
//#ifdef STM32_PLATFORM
//    I2CTEST_RESULT_T testResult = {0};
//    testResult = I2CStressTest_GetI2CTestResult();
//    
//    *((uint16_t*)buf) = testResult.running;
//    *((uint32_t*)(buf + 2)) = testResult.curTestCount;
//    *((uint32_t*)(buf + 6)) = testResult.curTestErrCount;
//    
//    //printf("GetI2CTestResult curTestCount=%d  curTestCount=%d\r\n", testResult.curTestCount, testResult.curTestCount);
//#endif 
//    ackLen = FillCmdAck((uint16_t*)buf, 5, cmd->cmdType, status);
//    return ackLen;
//}

/************************************************************************
 @名称；DoGetAppRun
 @功能：获取当前雷达运行的APP固件
 @参数：cmd：命令 
        cmdLen：命令长度
 @返回：无
*************************************************************************/
static uint16_t DoGetAppRun(CMD_T* cmd, uint32_t cmdLen)
{
    uint32_t ack_data = 0;
    uint16_t status = ACK_OK;

    if(CmdProc_InCmdMode())
    {
        #if defined RUN_BOOTLOADER
            ack_data = MCU_RUN_BOOTLOADER;
        #elif defined RUN_APP0
            ack_data = MCU_RUN_APP0;
        #elif defined RUN_APP1
            ack_data = MCU_RUN_APP1;
		#elif defined RUN_APP2
            ack_data = MCU_RUN_APP2;
		#elif defined RUN_APP3
			ack_data = MCU_RUN_APP3;
        #else
        #endif
    }
    else
    {
        status = ACK_FAIL;
    }
    return IAP_FillCmdACK(cmd->cmdType, (uint8_t*)&ack_data, sizeof(ack_data), status);
}


/************************************************************************
 @名称；DoSoftReset
 @功能：软件复位命令
 @参数：cmd：命令 
        cmdLen：命令长度
 @返回：无
*************************************************************************/
static uint16_t DoSoftReset(CMD_T* cmd, uint32_t cmdLen)
{
    uint16_t status = ACK_FAIL;
	
    if(CmdProc_InCmdMode())
    {
        Flash_WriteWord(APP_NEED_UPDATE_ADDR, 0xFFFFFFFE); 
        HAL_NVIC_SystemReset();
    }
    else
    {
        return IAP_FillCmdACK(cmd->cmdType, NULL, 0, status);;
    }
    return 0;
}

static uint16_t DoGetWhichDemo(CMD_T* cmd, uint32_t cmdLen)
{
	uint32_t ack_data = 0;
    uint32_t app_use  = 15;
    uint16_t ack_status = ACK_OK;
	uint8_t i = 0;
	MCU_RUN_APP_T current_app = MCU_RUN_NONE;
	if(CmdProc_InCmdMode())
    {
		#if defined RUN_BOOTLOADER
            current_app = MCU_RUN_BOOTLOADER;
        #elif defined RUN_APP0
            current_app = MCU_RUN_APP0;
        #elif defined RUN_APP1
            current_app = MCU_RUN_APP1;
		#elif defined RUN_APP2
            current_app = MCU_RUN_APP2;
		#elif defined RUN_APP3
            current_app = MCU_RUN_APP3;
        #else
        #endif
		
		ack_data = cmd->cmdData[0];
		for (i = 0; i < 4; i++)
		{
			if(((ack_data&0x0F) & (1<<i)) == (1<<i))
			{
				ack_data = i;
				break;
			}
		}
		app_use = Flash_ReadWord(APP_RUN_STORE_ADDR);
		if((app_use & (1 << (ack_data*4))) != (0x00000001<<(ack_data*4)))
		{
			 ack_status = ACK_FAIL;
		}
		else if((current_app >> ((ack_data&0x0F) + 1)) == 1)
		{
			
		}
		else
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
			Flash_ErasePage(ADDR_FLASH_SECTOR_3);
			Flash_WriteWord(ADDR_FLASH_SECTOR_3 + 8, ack_data); 
			Flash_WriteWord(APP_RUN_STORE_ADDR, 0x00001111);
			HAL_NVIC_SystemReset();
		}
	}
	else
    {
        ack_status = ACK_FAIL;
    }

    return IAP_FillCmdACK(cmd->cmdType, (uint8_t*)&ack_data, sizeof(ack_data), ack_status);
}


/********************************************
 @名称；DoGetAppUse
 @功能：获取MCU可编程的APP分区
 @参数：cmd，命令参数
 @返回：none
 @作者：HDN
*********************************************/
static uint16_t DoGetAppUse(CMD_T* cmd, uint32_t cmdLen)
{
    uint32_t ack_data = 0;
    uint32_t app_use  = 15;
    uint16_t ack_status = ACK_OK;

    if(CmdProc_InCmdMode())
    {
        app_use = Flash_ReadWord(APP_RUN_STORE_ADDR);
        ack_data = app_use;
        ack_data &= 0x0F;
    }
    else
    {
        ack_status = ACK_FAIL;
    }

    return IAP_FillCmdACK(cmd->cmdType, (uint8_t*)&ack_data, sizeof(ack_data), ack_status);
}


/********************************************
 @名称；CmdGetDeviceID
 @功能：获取处理器96位的序列号的后32位
 @参数：cmd，命令参数; cmdLen, 参数长度
 @返回：none
*********************************************/
static uint16_t DoGetDeviceID(CMD_T* cmd, uint32_t cmdLen)
{
    uint32_t id = 0;
    uint32_t ack_data = 0;
    uint16_t ack_status = ACK_OK;

    if(CmdProc_InCmdMode())
    {
        id = HAL_GetUIDw0();   
        ack_data = id;
    }
    else
    {
        ack_status = ACK_FAIL;
    }
    return IAP_FillCmdACK(cmd->cmdType, (uint8_t*)&ack_data, sizeof(ack_data), ack_status);
}

static uint16_t Set_PDCalibrationPara(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t lowPart = 0;
    uint16_t highPart = 0;
    
    if(0 == CmdModeFlag)
    {
        goto ACK_FAIL_RETURN; 
    }
    
    /* config RangeBin threshold cmd */
    if(WRITE_PD_CALIBRATION_CMD == cmd->cmdType)
    {
        /* check paras ID */
        if(cmdLen < 8)
        {
            goto ACK_FAIL_RETURN;
        }
        for(uint8_t i=0;i<((cmdLen-2)/ 6);i++)
        {
            if(0x0000 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.calibrated_PhaseDiff = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0010 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib1_Temper = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0011 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib1_Power = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0012 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib1_Refval = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0013 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib1_SlopeVal = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0020 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib2_Temper = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0021 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib2_Power = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0022 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib2_Refval = (uint32_t)((highPart<<16)|lowPart);
            }
            else if(0x0023 == cmd->cmdData[3*i])
            {
                lowPart = cmd->cmdData[3*i+1];
                highPart = cmd->cmdData[3*i+2];
                PD_CalibrationPara.PDCalib2_SlopeVal = (uint32_t)((highPart<<16)|lowPart);
            }
        }

    }

    Config_NeedFlashWrite();

    ackLen = FillCmdAck(NULL, 2, cmd->cmdType, ACK_OK); 
    return ackLen;

    ACK_FAIL_RETURN:
    ackLen = FillCmdAck(NULL, 2, cmd->cmdType, ACK_FAIL);
    return ackLen;
}

static uint16_t Read_PDCalibPara(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint8_t pos = 0;
    uint16_t  tx_buf_u16[18] = {0};
    
    if(0 == CmdModeFlag)
    {
        ackLen = FillCmdAck(NULL, 2, cmd->cmdType, ACK_FAIL);
        return ackLen;
    }
    
    if(READ_PD_CALIBRATION_CMD == cmd->cmdType)
    {
        if(cmdLen < 4)
        {
            ackLen = FillCmdAck(NULL, 2, cmd->cmdType, ACK_FAIL);
            return ackLen;
        }
        
        for(uint8_t i=0;i<((cmdLen-2)/ 2);i++)
        {
            if(0x0000 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.calibrated_PhaseDiff, 4);
            }
            else if(0x0010 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib1_Temper, 4);
            }
            else if(0x0011 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib1_Power, 4);
            }
            else if(0x0012 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib1_Refval, 4);
            }
            else if(0x0013 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib1_SlopeVal, 4);
            }
            else if(0x0020 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib2_Temper, 4);
            }
            else if(0x0021 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib2_Power, 4);
            }
            else if(0x0022 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib2_Refval, 4);
            }
            else if(0x0023 == cmd->cmdData[i])
            {
                memcpy(&(tx_buf_u16[pos]), &PD_CalibrationPara.PDCalib2_SlopeVal, 4);
            }
            pos += 2;
        }
        
    }
    ackLen = FillCmdAck(tx_buf_u16, pos, cmd->cmdType, ACK_OK); 
    return ackLen;
}

extern QueueHandle_t usbDataQueue;
/********************************************
 @名称；CmdExec
 @功能：获取处理器96位的序列号的后32位
 @参数：cmd，命令参数
 @返回：none
*********************************************/
static void CmdExec(CMD_T *cmd, uint32_t cmdLen)
{
    uint16_t ackLen = 0;
    uint16_t devVersion[] = {RADAR_DEV_TYPE, RADAR_DEV_FIRMWARE_TYPE, RADAR_DEV_VER_MAJAR, RADAR_DEV_VER_MINOR, RADAR_DEV_VER_PATCH};

    uint16_t boardInfomation[] = {RADAR_PROTOCOL_VERSION, CMD_LEN_MAX};
#ifdef STM32_PLATFORM
    USB_DATA_T usbDataQ = {0};
#endif

    if (NULL == cmd || 0 == cmdLen)
    {
        return;
    }

    switch(cmd->cmdType)
    {

        case START_CFG_CMD:
            CmdModeFlag = 1;
            PreSystemCofig = 0;
            while(gDataprocessFlag)
            {
                vTaskDelay(1);
            }  
            System_Reset();
						
            SPI4_DeInit();
            SPI4_Config_Init();
						
            if(cmdLen > CMD_TYPE_LEN)
            {
                masterProtocolVer = cmd->cmdData[0];
                ackLen = FillCmdAck(boardInfomation, ARRAY_SIZE(boardInfomation), cmd->cmdType, ACK_OK);
            }
            else
            {
                masterProtocolVer = 0;
                ackLen = FillCmdAck(NULL, 0, cmd->cmdType, ACK_OK);
            }
            break;
        
        case FINISH_CFG_CMD:
//            Config_SavePara2Flash();
//            System_Reconfig();
            ackLen = FillCmdAck(NULL, 0, cmd->cmdType, ACK_OK);
				PreSystemCofig = 1;
				isSingleSPI = IsSingleSpiInit();
				System_Reconfig(0xFF);
            break;

        case WRITE_REG_CMD:
            ackLen = DoWriteReg(cmd, cmdLen);
            break;
            
        case READ_REG_CMD:
            ackLen = DoReadReg(cmd, cmdLen);
            break;
            
        case READ_VER_CMD:
            if(masterProtocolVer != 0)
            {
                ackLen = FillCmdAck(devVersion, ARRAY_SIZE(devVersion), cmd->cmdType, ACK_OK);
            }
            else
            {
                ackLen = FillCmdAck(&devVersion[2], ARRAY_SIZE(devVersion) - 2 , cmd->cmdType, ACK_OK);
            }
            
            break;
        
        case WRITE_MTT_CMD:
        case WRITE_SYS_CMD:
        case FFT_ZEROFILL_CMD:
//        case NOP_CONFIG_CMD:
        case MCU_SPECIAL_FUNC_CMD:
            ackLen = DoWritePara(cmd, cmdLen);
            break;
    
        case READ_MTT_CMD:
        case READ_SYS_CMD:
            ackLen = DoReadPara(cmd, cmdLen);
            break;

            /* config para, specially for ABD */
//        case PARAM_CFG_CMD:
//        case THRESHOLD_SET_CMD:
//            ackLen = BodySensing_ParaUpdate(cmd, cmdLen, CmdModeFlag);
//            break;
//        case PARAM_READ_CMD: 
//            ackLen = DoReadBodySensingPara(cmd, cmdLen, CmdModeFlag);
//            break;

        case WRITE_PD_CALIBRATION_CMD:
            ackLen = Set_PDCalibrationPara(cmd, cmdLen);
            break;

        case READ_PD_CALIBRATION_CMD:
            ackLen = Read_PDCalibPara(cmd, cmdLen);
            break;

        case TRACK_ONE_CMD:
            ackLen = DoTrackOneMod(cmd, cmdLen);
            break;
        
        case TRACK_TWO_CMD:
            ackLen = DoTrackTwoMod(cmd, cmdLen);
            break;
        
        case WRITE_SN_CMD:
            ackLen = DoWriteSn(cmd);
            break;

        case READ_SN_CMD:
            ackLen = DoReadSn(cmd);
            break;
        
        case CASCADING_MODE_CMD:
            ackLen = DoCascadingMode(cmd, cmdLen);
            break;
        
//        case START_I2C_TEST_CMD:
//            ackLen = StartI2CTest(cmd, cmdLen);
//            break;
        
//        case STOP_I2C_TEST_CMD:
//            ackLen = StopI2CTest(cmd, cmdLen);
//            break;
        
//        case GET_I2C_TEST_RESULT_CMD:
//            ackLen = GetI2CTestResult(cmd, cmdLen);
//            break;
        
        case GET_APP_RUN_CMD:
            ackLen = DoGetAppRun(cmd, cmdLen);
            break;

        case GET_MCU_ID_CMD:
            ackLen = DoGetDeviceID(cmd, cmdLen);
            break;

        case SOFT_GET_APP_USE_CMD:
            ackLen = DoGetAppUse(cmd, cmdLen);
            break;

        case SOFT_RESET_CMD:
            ackLen = DoSoftReset(cmd, cmdLen);
            break;

		case GET_WHICH_DEMO:
			ackLen = DoGetWhichDemo(cmd, cmdLen);
			break;
        
        default:
            break;
    }
    
    if (ackLen > 0)
    {

#ifdef STM32_PLATFORM
        usbDataQ.buf = CmdAck;
        usbDataQ.len = ackLen;
        if(uxQueueSpacesAvailable(usbDataQueue))
        {
            UsbTransfer_Send2UsbDataQueue(&usbDataQ);
        }
#else
        UART0_DmaSend(CmdAck, ackLen);
#endif
    }
        
    if(cmd->cmdType == FINISH_CFG_CMD)
    {
        if(CmdModeFlag)
        {
            CmdModeFlag = 0;
            Config_SavePara2Flash();
            if(!PreSystemCofig)
            {
                System_Reconfig(0xFF);  
            }         
        }      
    }
}

/********************************************
 @名称；CmdProc_InCmdMode
 @功能：返回CmdModeFlag
 @参数：none
 @返回：CmdModeFlag，cmd模式flag
*********************************************/
uint8_t CmdProc_InCmdMode(void)
{
    return CmdModeFlag;
}

/********************************************
 @名称；CmdProcess
 @功能：命令行模式入口函数
 @参数：buf，串口数据buf
        len，数据长度
 @返回：none
*********************************************/
static void CmdProcess(uint8_t* buf, uint16_t len)
{
    uint16_t loop = 0;

    if (NULL == buf || 0 == len)
    {
        return;
    }
    
#if defined(CONFIG_DEBUG) && defined(STM32_PLATFORM)
    printf("recvLen:%d\r\n", len);
    for (int i = 0; i < len; i++)
    {
        printf("0x%02x ", buf[i]);
    }
    printf("\r\n");
#endif

    for (loop = 0; loop < len; loop++) 
    {
        switch(CmdDataParse.state)
        {                    
            case CMD_STATE_HEAD0:
                if (buf[loop] == CmdHead[CMD_HEAD_0]) 
                {
                    CmdDataParse.curIndex = 0;
                    CmdDataParse.state = CMD_STATE_HEAD1;
                }
                break;
                
            case CMD_STATE_HEAD1:
                if (buf[loop] == CmdHead[CMD_HEAD_1])
                {
                    CmdDataParse.state = CMD_STATE_HEAD2;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
    
            case CMD_STATE_HEAD2:
                if (buf[loop] == CmdHead[CMD_HEAD_2])
                {
                    CmdDataParse.state = CMD_STATE_HEAD3;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
    
            case CMD_STATE_HEAD3:
                if (buf[loop] == CmdHead[CMD_HEAD_3])
                {
                    CmdDataParse.state = CMD_STATE_LEN0;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
    
            case CMD_STATE_LEN0:
                CmdDataParse.len = buf[loop];
                CmdDataParse.state = CMD_STATE_LEN1;
                break;
    
            case CMD_STATE_LEN1:
                CmdDataParse.len += buf[loop] << CMD_LEN_HIGH_POS;
                if (CmdDataParse.len <= CMD_BUF_LEN)
                {
                    CmdDataParse.state = CMD_STATE_DATA;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
                
            case CMD_STATE_DATA:
                CmdDataParse.buf[CmdDataParse.curIndex++] = buf[loop];
                if (CmdDataParse.curIndex == CmdDataParse.len)
                {
                    CmdDataParse.state = CMD_STATE_TAIL0;
                }
                break;
                
            case CMD_STATE_TAIL0:
                if (buf[loop] == CmdTail[CMD_TAIL_0])
                {
                    CmdDataParse.state = CMD_STATE_TAIL1;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
    
            case CMD_STATE_TAIL1:
                if (buf[loop] == CmdTail[CMD_TAIL_1])
                {
                    CmdDataParse.state = CMD_STATE_TAIL2;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
                
            case CMD_STATE_TAIL2:
                if (buf[loop] == CmdTail[CMD_TAIL_2])
                {
                    CmdDataParse.state = CMD_STATE_TAIL3;
                }
                else
                {
                    CmdDataParse.state = CMD_STATE_HEAD0;
                }
                break;
    
            case CMD_STATE_TAIL3:
                if (buf[loop] == CmdTail[CMD_TAIL_3])
                {
                    CmdExec((CMD_T*)(CmdDataParse.buf), CmdDataParse.len);   
                }
                CmdDataParse.state = CMD_STATE_HEAD0;
                break; 
            
            default:
                CmdDataParse.state = CMD_STATE_HEAD0;
                break;
    
        }
    }

}

/********************************************
 @名称；CmdProc_Init
 @功能：命令行初始化
 @参数：none
 @返回：none
*********************************************/
void CmdProc_Init(void)
{
    memset(&g_cmdRecv, 0, sizeof(g_cmdRecv));

}

/********************************************
 @名称；CmdProc_IsInDebugMode
 @功能：返回调试模式flag
 @参数：none
 @返回：DebugModeFlag，调试模式flag
*********************************************/
uint8_t CmdProc_IsInDebugMode(void)
{
    return DebugModeFlag;
}


#ifdef STM32_PLATFORM
osThreadId cmdProcTaskHandle;
QueueHandle_t cmdDataQueue;

/********************************************
 @名称；CmdProc_Send2CmdDataQueue
 @功能：将命令行数据传输至队列
 @参数：cmdData，命令行数据指针
 @返回：none
*********************************************/
void CmdProc_Send2CmdDataQueue(void *cmdData)
{
    BaseType_t xHigherPriorityTaskWoken = 0;
    
    if (NULL == cmdData)
    {
        return;
    }
    
    if(xQueueSendFromISR(cmdDataQueue, cmdData, &xHigherPriorityTaskWoken)!= pdPASS) 
    {
        Indicator_CmdDataRecvOverFlow();
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

/********************************************
 @名称；CmdProcTask
 @功能：命令行任务函数
 @参数：none
 @返回：none
*********************************************/
void CmdProcTask(void const * argument)
{
    CMD_DATA_T cmdData = {0};
        
    while(1)
    { 
        if (xQueueReceive(cmdDataQueue, &cmdData, portMAX_DELAY))
        {
            CmdProcess(cmdData.buf, cmdData.len);
        }
    } 
}

/********************************************
 @名称；CmdProc_TaskInit
 @功能：命令行任务初始化
 @参数：none
 @返回：none
*********************************************/
void CmdProc_TaskInit(void)
{
    cmdDataQueue = xQueueCreate(CMD_DATA_QUEUE_SIZE, sizeof(CMD_DATA_T));
    
    osThreadDef(cmdProcTask, CmdProcTask, osPriorityHigh, 0, CMD_PROC_STACK_SIZE);
    cmdProcTaskHandle = osThreadCreate(osThread(cmdProcTask), NULL);

    g_McuSpecialFuncSig = xSemaphoreCreateBinary();

    if (NULL == cmdProcTaskHandle)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
}
#endif

