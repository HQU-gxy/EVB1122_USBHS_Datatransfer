/**
  ******************************************************************************
  * @file           : dataprocess.c
  * @author         : iclm team
  * @brief          : data process module
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "global_conf.h"
#include "utilities.h"
#include "dataprocess.h"
#include "banyan.h"
#include "cmdprocess.h"
#include "system.h"
#ifdef STM32_PLATFORM
#include "rtos.h"
#include "cmsis_os.h"
#include "usb_transfer.h"
#endif
#include "platform.h"
#include "stm32_spi.h"

static uint8_t flag_read_PD = 0;
static uint8_t flag_read_TS = 0;

static RADAR_DATA_PARSE_T RadarDataParse[CHANNEL_MAX];
static int16_t nRecvCntMax = 256;
static uint8_t nChannelIndex[CHANNEL_MAX];
static uint8_t needCheckIndex[CHANNEL_MAX];
static RADAR_PARA_T RadarPara;
//static uint8_t adcResetChannelIndex;
uint16_t whichDemo = 0xFFFF;        /* 0x00: pass through */
                                    /* 0x01: motion(multi) target tracking */
                                    /* 0x04: Active Body Detection */

uint8_t g_ChipTempDataPack[MCU_SPECIAL_PACK_SIZE] __ALIGN(4) = {0};
uint8_t g_ChipPowerDataPack[MCU_SPECIAL_PACK_SIZE] __ALIGN(4) = {0};

/********************************************
 @名称；CheckChirpIndex
 @功能：检查chirp index
 @参数：channel，接收通道
        chirpIndex，chirp索引
 @返回：none
*********************************************/
static void CheckChirpIndex(uint8_t channel, uint8_t chirpIndex)
{
    static uint8_t curIndex[CHANNEL_MAX] = {0};
    static uint8_t oldCurIndex[CHANNEL_MAX] = {0};
    static uint8_t skipNum = SKIP_NUM;

    if (channel >= CHANNEL_MAX)
    {
        return;
    }

    if (skipNum) 
    {
        skipNum--;
        curIndex[channel] = oldCurIndex[channel] = chirpIndex % RadarPara.chirpNum;
    } 
    else 
    {
        curIndex[channel] = chirpIndex % RadarPara.chirpNum;
        if (curIndex[channel] != ((oldCurIndex[channel] + 1) % RadarPara.chirpNum))
        {
            Indicator_RadarDataIndexError();
        }
        oldCurIndex[channel] = curIndex[channel];
    }
}

/********************************************
 @名称；CheckFrameCnt
 @功能：检查frame索引
 @参数：channel，接收通道
        frameCnt，frame索引
 @返回：none
*********************************************/
static void CheckFrameCnt(uint8_t channel, uint16_t frameCnt)
{
    static uint16_t oldFrameCnt[CHANNEL_MAX] = {0};
    static uint8_t skipNum = SKIP_NUM;

    if (channel >= CHANNEL_MAX)
    {
        return;
    }
    
    if (skipNum) 
    {
        skipNum--;
        oldFrameCnt[channel] = frameCnt;
    } 
    else if (frameCnt != oldFrameCnt[channel] + 1)
    {
        Indicator_RadarDataIndexError();
    }
    oldFrameCnt[channel] = frameCnt;
}

/********************************************
 @名称；DataStateIdParse
 @功能：状态ID解析
 @参数：data，数据
        channel，接收通道
 @返回：none
*********************************************/
static void DataStateIdParse(uint8_t data, uint8_t channel)
{
    uint8_t flag = 0;
    
    if(RadarPara.newDataFormat)
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_FFT:
                if ((data & ID_MASK) == FFT0_ID_NEW || (data & ID_MASK) == FFT1_ID_NEW)
                {
                    RadarDataParse[channel].chirpIndex = (data & CHIRP_INDEX_NEW_MASK0) << CHIRP_INDEX_NEW_POS0;
                    flag = 1;
                }
                break;
            case DATA_TYPE_DSRAW:
                if ((data & ID_MASK) == DSRAW0_ID_NEW || (data & ID_MASK) == DSRAW1_ID_NEW)
                {
                    RadarDataParse[channel].chirpIndex = (data & CHIRP_INDEX_NEW_MASK0) << CHIRP_INDEX_NEW_POS0;
                    flag = 1;
                }
                break;
            case DATA_TYPE_DFFT:
                if ((data & ID_MASK) == DFFT0_ID_NEW || (data & ID_MASK) == DFFT1_ID_NEW)
                {
                    RadarDataParse[channel].frameCnt = (data & CHIRP_INDEX_NEW_MASK0) << CHIRP_INDEX_NEW_POS0;
                    flag = 1;
                }
                break;
            
            case DATA_TYPE_DFFT_PEAK:
                if ((data & ID_MASK) == DFFT_PEAK_ID_NEW)
                {
                    flag = 1;
                }
                break;
            
            default:
                break;
        }        
    }
    else
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_FFT:
                if ((data & ID_MASK) == FFT0_ID || (data & ID_MASK) == FFT1_ID)
                {
                                if(channel > 1)
                                {
                                    data += 0x20;
                                }
                  RadarDataParse[channel].chirpIndex = (data & CHIRP_INDEX_MASK) << CHIRP_INDEX_POS0;
                  flag = 1;
                }
                break;
            case DATA_TYPE_DSRAW:
                if ((data & ID_MASK) == DSRAW0_ID || (data & ID_MASK) == DSRAW1_ID)
                {
                                if(channel > 1)
                                {
                                    data += 0x20;
                                }
                  RadarDataParse[channel].chirpIndex = (data & CHIRP_INDEX_MASK) << CHIRP_INDEX_POS0;
                  flag = 1;
                }
                break;
            case DATA_TYPE_DFFT:
                            if ((data & ID_MASK) == DFFT0_ID || (data & ID_MASK) == DFFT1_ID)
                {
                                if(channel > 1)
                                {
                                    data -= 0x80;
                                }
                  flag = 1;
                      }
                break;
            
            case DATA_TYPE_DFFT_PEAK:
                if ((data & ID_MASK) == DFFT_PEAK_ID)
                {
                    flag = 1;
                      }
                break;
            
            default:
                break;
        }
    }

    if (flag)
    {
        RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
        RadarDataParse[channel].state = DATA_STATE_INDEX1;
    }
    else
    {
        RadarDataParse[channel].state = DATA_STATE_HEAD;
    }
}

static void DataStateIndex1Parse(uint8_t data, uint8_t channel)
{
    if(RadarPara.newDataFormat)
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_DSRAW:
            case DATA_TYPE_FFT:
                RadarDataParse[channel].chirpIndex += (((data & CHIRP_INDEX_NEW_MASK2) << CHIRP_INDEX_NEW_POS2) | 
                                                        ((data & CHIRP_INDEX_NEW_MASK1) >> CHIRP_INDEX_NEW_POS1));
                break;
            
            case DATA_TYPE_DFFT:
                RadarDataParse[channel].frameCnt += (((data & CHIRP_INDEX_NEW_MASK2) << CHIRP_INDEX_NEW_POS2) | 
                                                        ((data & CHIRP_INDEX_NEW_MASK1) >> CHIRP_INDEX_NEW_POS1));                
                break;   
            
            case DATA_TYPE_DFFT_PEAK:
                break;
            
            default:
                break;
        }         
    }
    else
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_DSRAW:
            case DATA_TYPE_FFT:
                RadarDataParse[channel].chirpIndex += data >> CHIRP_INDEX_POS1;
                break;
            
            case DATA_TYPE_DFFT:
                RadarDataParse[channel].frameCnt = data << FRAME_CNT_POS;
                break;
            
            case DATA_TYPE_DFFT_PEAK:
                break;
            
            default:
                break;
        }    
    }

    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    RadarDataParse[channel].state = DATA_STATE_INDEX2;
}

/********************************************
 @名称；DataStateIndex2Parse
 @功能：索引解析
 @参数：data，数据
        channel，接收通道
 @返回：none
*********************************************/
static void DataStateIndex2Parse(uint8_t data, uint8_t channel)
{
    if(RadarPara.newDataFormat == 0)
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_FFT:
                break;
            
            case DATA_TYPE_DFFT:
                RadarDataParse[channel].frameCnt += data;
                break;
            
            case DATA_TYPE_DFFT_PEAK:
                break;
            
            default:
                break;
        }
    }
    
    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    RadarDataParse[channel].state = DATA_STATE_DATA;
}

/********************************************
 @名称；DataStateTail2Parse
 @功能：帧尾解析
 @参数：data，数据
        channel，接收通道
 @返回：none
*********************************************/
static uint8_t DataStateTail2Parse(uint8_t data, uint8_t channel)
{
    if(RadarPara.newDataFormat == 0)
    {
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_FFT:
                        if(channel > 1)
                        {
                            data += 0x20;
                        }
              break;
                    
            case DATA_TYPE_DSRAW:
                        if(channel > 1)
                        {
                            data += 0x20;
                        }  
              break;
            
            default:
              break;
        }
    }

    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    RadarDataParse[channel].state = DATA_STATE_TAIL3;
    return 1;
}

/********************************************
 @名称；DataStateTail3Parse
 @功能：帧尾解析
 @参数：data，数据
        channel，接收通道
 @返回：none
*********************************************/
static uint8_t DataStateTail3Parse(uint8_t data, uint8_t channel)
{
    RadarDataParse[channel].state = DATA_STATE_HEAD;
    if (data != DATA_TAIL) 
    {
        return 0;
    }
    
    switch (RadarPara.dataType)
    {
        case DATA_TYPE_DSRAW:
        case DATA_TYPE_FFT:
            if ((RadarDataParse[channel].chirpIndex == RadarPara.chirpNum - 1) && (channel == CHANNEL_MAX - 1))
            {
                CmdProc_AdcReset();
                CmdProc_NopConfig();
            }
            break;
        
        case DATA_TYPE_DFFT:             
            if (channel == CHANNEL_MAX - 1)
            {
                CmdProc_AdcReset();
                CmdProc_NopConfig();
            }
            break;
        
        case DATA_TYPE_DFFT_PEAK:
            break;
        
        default:
            break;
    }
    
    RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = data;
    
    return 1;
}

/********************************************
 @名称；DataCopy
 @功能：复制数据至解析buffer
 @参数：buf，数据指针
        len，数据长度
        channel，接收通道
        i，当前数据位置
 @返回：none
*********************************************/
static void DataCopy(uint8_t* buf, uint16_t len, uint8_t channel, uint16_t *i)
{
    uint16_t copyLen = 0;

    if (NULL == buf || NULL == i)
    {
        return;
    }
    
    copyLen = (RadarDataParse[channel].needCopyLen > (len - *i))?
            (len - *i) : RadarDataParse[channel].needCopyLen;
    memcpy(&RadarDataParse[channel].buf[RadarDataParse[channel].curIndex], &buf[*i], copyLen);

    RadarDataParse[channel].curIndex += copyLen;
    *i += (copyLen - 1);
    RadarDataParse[channel].needCopyLen -= copyLen;

    if (!RadarDataParse[channel].needCopyLen)
    {
        RadarDataParse[channel].state = DATA_STATE_TAIL0;
        RadarDataParse[channel].needCopyLen = RadarPara.dataLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
    }
}

/********************************************
 @名称；DataParse
 @功能：复制数据至解析buffer
 @参数：buf，数据指针
        len，数据长度
        channel，接收通道
        left，剩余数据
 @返回：none
*********************************************/
static uint8_t DataParse(uint8_t* buf, uint16_t len, uint8_t channel, uint16_t* left)
{
    uint16_t i = 0;
    uint8_t parseFinish = 0;
    *left = len;
    uint8_t startPos = sysPara.useMcuPackWrapper? SPI_FRAME_WRAPPER_SHIFT:0;;
    
    if (NULL == buf || 0 == len || channel >= CHANNEL_MAX)
    {
        return 0;
    }
    
    for (i = 0; (i < len) && (parseFinish == 0); i++) 
    {
            switch(RadarDataParse[channel].state)
            {                    
                case DATA_STATE_HEAD:
                    if (buf[i] == DATA_HEAD) 
                    {
                        RadarDataParse[channel].curIndex = startPos;
                        RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
                        RadarDataParse[channel].state = DATA_STATE_ID;
                    }
                    break;
                
                case DATA_STATE_ID:
        DataStateIdParse(buf[i], channel);
                    break;
                
                case DATA_STATE_INDEX1:
        DataStateIndex1Parse(buf[i], channel);
                    break;
                
                case DATA_STATE_INDEX2:
        DataStateIndex2Parse(buf[i], channel);
                    break;
            
                case DATA_STATE_DATA:
        DataCopy(buf, len, channel, &i);
                    break;
    
        case DATA_STATE_TAIL0:
        RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
                    RadarDataParse[channel].state = DATA_STATE_TAIL1;
                    break;
    
        case DATA_STATE_TAIL1:
        RadarDataParse[channel].buf[RadarDataParse[channel].curIndex++] = buf[i];
                    RadarDataParse[channel].state = DATA_STATE_TAIL2;
                    break;
    
        case DATA_STATE_TAIL2:
                    DataStateTail2Parse(buf[i], channel);
                    break;
                
        case DATA_STATE_TAIL3:
        parseFinish = DataStateTail3Parse(buf[i], channel);
                break;
            
                default:
                    RadarDataParse[channel].state = DATA_STATE_HEAD;
                    break;
        }
    }
    
    *left -= i;
    
  return parseFinish;
}

#if defined(SUPPORT_DATA_PASSTHROUGH) || defined(SUPPORT_DYNAMIC_SYS_MODE)
static int16_t nRecvCnt[CHANNEL_MAX] = {0};

/********************************************
 @名称；DataProc_ResetRecvCnt
 @功能：复位接收cnt
 @参数：none
 @返回：none
*********************************************/
void DataProc_ResetRecvCnt(void)
{
    memset(nRecvCnt, 0, sizeof(nRecvCnt));
}

/************************************************************************
 @名称：ChirpOrFrameIndexCheck
 @功能：索引判断，检查索引是否正确
 @参数：channel：通道
        index：索引
 @返回：TRUE，索引真确；FALSE：索引错误
*************************************************************************/
static uint8_t ChirpOrFrameIndexCheck(uint8_t channel, uint8_t index)
{
    static uint8_t exp_idx[CHANNEL_MAX] = {0};
    uint16_t cycleNum = 0;
    
    if(RadarPara.dataType == DATA_TYPE_DFFT)
    {
        cycleNum = 512;    /* 2dfft数据的frame index索引最大为511，9bit保存，不过实际用到2dfft数据时这个函数可以注释掉不进行校验  */
    }
    else
    {
        cycleNum = RadarPara.chirpNum;
    }

    if(index != exp_idx[channel])
    {
        exp_idx[channel] = 0;

        return 0;
    }

    exp_idx[channel] = (exp_idx[channel] + 1) % cycleNum;

    return 1;
}

static uint8_t IsFrameReady()
{
    uint8_t bReady = 1;
    
    for(uint8_t i = 0; i < CHANNEL_MAX; i++)
    {
        if(g_RxCount != 2 * g_ChipCount)
        {
            if((RadarPara.rxType & BIT(i % 2)) == 0)
            {
                continue;
            }
        }
        
        if(nChannelIndex[i] != RadarPara.chirpNum - 1)
        {
            bReady = 0;
            break;
        }
    }  
    
    return bReady;
}

/********************************************
 @名称；StartDataTransfer
 @功能：传输数据
 @参数：frameBuf，数据buffer
        bufLen，数据长度
        channel，接收通道
        index，chirp index
 @返回：none
*********************************************/
static void StartDataTransfer(uint8_t* frameBuf, uint16_t bufLen, uint8_t channel, uint16_t index)
{
    uint16_t uploadSampleRate = 0;
    uint8_t chipCount = g_TxCount;
	uint8_t directSend = 0;
    uint8_t canSend = 0;  
    
    uploadSampleRate = System_GetUploadSampleRate();
    if (DATA_TYPE_DFFT_PEAK == RadarPara.dataType)
    {
        uploadSampleRate = 1;
		directSend = 1;
    }
    
    nChannelIndex[channel] = index;
    
    //when uploadSampleRate = 1, must send
    if(uploadSampleRate == 1)
    {
        canSend = 1;
    }
    else
    {
        // if data have chirp index, need check and send data when chirp index = 0
        if (needCheckIndex[channel])
        {
            if (nRecvCnt[channel] == -1)
            {
                // When not sent, start from 0
                if (index == 0)
                {
                  nRecvCnt[channel] = 0;
                  canSend = 1;
                }
            }
            else
            {
                //It has been sent. Send it according to the downsampling interval
                //When the number of downsampling intervals is reached, it needs to be sent
                if (nRecvCnt[channel] % (uploadSampleRate * chipCount) < chipCount)
                {
                    canSend = 1;
                }
            }
        }
        else
        {
          //When the number of downsampling intervals is reached, it needs to be sent
          if (nRecvCnt[channel] % (uploadSampleRate * chipCount) < chipCount)
          {
              canSend = 1;
          }
        }
    }

    if (nRecvCnt[channel] != -1)
    {
        nRecvCnt[channel]++;
    }
    
    if(needCheckIndex[channel])
    {
       if(nRecvCnt[channel] == nRecvCntMax)
        {
            nRecvCnt[channel] = -1;
        }   
    } 
    else
    {  
        if(nRecvCnt[channel] == uploadSampleRate * chipCount)
        {
            nRecvCnt[channel] = 0;
        }           
    }        
       
    uint8_t bSend = 0;
    if(canSend)
    {
        if(sysPara.useMcuPackWrapper)
        {
            uint8_t *pHead = (uint8_t *)(((uint32_t)frameBuf) - 8);
            *(volatile uint32_t *)pHead = SPI_FRAME_WRAPPER_HEAD;
            *(volatile uint16_t *)&pHead[4] = bufLen;
            pHead[6] = 0x00;
            pHead[7] = channel;
            *(volatile uint32_t *)(&frameBuf[bufLen]) = SPI_FRAME_WRAPPER_TAIL;
            bSend = UsbTransfer_Send(pHead, bufLen+12, directSend); 
        }
        else
        {
            bSend = UsbTransfer_Send(frameBuf, bufLen, directSend);
        }

    }
    
#ifdef STM32_PLATFORM
    if(!bSend && IsFrameReady())
    {
        UsbTransfer_ForceSend();
    }
#endif   

}
#endif

/********************************************
 @名称；StartAlgorithm
 @功能：算法入口函数
 @参数：dataBuf，数据buffer
        bufLen，数据长度
        channel，接收通道
        index，chirp index
 @返回：none
*********************************************/
void StartAlgorithm(uint8_t* dataBuf, uint16_t dataLen, uint8_t channel, uint16_t index)
{
    /*do algo here*/

}

/************************************************************************
 @名称；DataDispatch
 @功能：数据解析后的功能选择，只能选择数据透传或者算法应用
 @参数：frameBuf：帧数据
        len：数据长度
        channel：通道
        index：索引
 @返回：无
 @作者：RSS SW TEAM
*************************************************************************/
static void DataDispatch(uint8_t* frameBuf, uint16_t bufLen, uint8_t channel, uint16_t index)
{
    static uint8_t isFirstIn = 1;
    
    uint8_t* dataBuf = NULL;
    uint16_t dataLen = 0;

    if (NULL == frameBuf || 0 == bufLen)
    {
        return;
    }
		
		if (RadarPara.rxType == 3)
		{
				if (( RadarDataParse[0].chirpIndex == RadarPara.chirpNum-1)  &&  (RadarDataParse[1].chirpIndex == RadarPara.chirpNum-1))
				{
					flag_read_PD = 1;
					flag_read_TS = 1;
				}
				else
				{
					flag_read_PD = 0;
					flag_read_TS = 0;
				}
			}
		
		if (RadarPara.rxType == 1)
		{
				if ( RadarDataParse[0].chirpIndex == RadarPara.chirpNum-1) 
				{
					flag_read_PD = 1;
					flag_read_TS = 1;
				}
				else
				{
					flag_read_PD = 0;
					flag_read_TS = 0;
				}
			}

        StartDataTransfer(frameBuf, bufLen, channel, index);

}

/************************************************************************
 @名称；DataProcess
 @功能：数据处理，对SPI接收的数据进行处理
 @参数：channel：SPI通道
        dmaFlag：接收乒乓缓存的标志，用于处理具体缓存数据
        recvBuf：处理数据
        bufLen：数据长度
 @返回：无
 @作者：RSS SW TEAM
*************************************************************************/
static void DataProcess(uint8_t channel, uint8_t dmaFlag, uint8_t *recvBuf, uint16_t bufLen)
{
    uint8_t parseFinish = 0;
    uint16_t bufLeftLen = bufLen;
    uint16_t index = 0;
    uint16_t threshold = INDICATOR_RECV_THRESHOLD;
    uint8_t startPos = sysPara.useMcuPackWrapper? SPI_FRAME_WRAPPER_SHIFT:0;
    
    if (channel >= CHANNEL_MAX || dmaFlag >= DMA_RECV_FLAG_MAX || NULL == recvBuf)
    {
        printf("Error para!\r\n");
        return;
    }
    
    if(DATA_TYPE_DFFT_PEAK != RadarPara.dataType &&
        g_RxCount != 2 * g_ChipCount)
    {
        if((RadarPara.rxType & BIT(channel % (2 * g_ChipCount))) == 0)
        {
            return;
        }
    }

    if (CmdProc_InCmdMode())
    {
        return;
    }
    
    while (bufLeftLen > 0)
    {
        parseFinish = DataParse(
            recvBuf + bufLen - bufLeftLen,
            bufLeftLen,
            channel,
            &bufLeftLen);
        g_dataRecvFlag[channel][dmaFlag] = 0;
        if (!parseFinish)
        {
            continue;
        }
        
        switch (RadarPara.dataType)
        {
            case DATA_TYPE_DSRAW:
            case DATA_TYPE_FFT:
//                printf("%d  %d\n", channel, RadarDataParse[channel].chirpIndex);
                CheckChirpIndex(channel, RadarDataParse[channel].chirpIndex);
                index = RadarDataParse[channel].chirpIndex;
                break;
            
            case DATA_TYPE_DFFT:
                CheckFrameCnt(channel, RadarDataParse[channel].frameCnt);
                index = RadarDataParse[channel].frameCnt;
                threshold >>= INDICATOR_RECV_THD_DFFT_SHIFT;
                break;
            
            case DATA_TYPE_DFFT_PEAK:
                threshold >>= INDICATOR_RECV_THD_DPEAK_SHIFT;
                break;
            
            default:
                break;
        }
        
        Indicator_RadarDataReceived(threshold);
        DataDispatch(&RadarDataParse[channel].buf[startPos], RadarPara.dataLen, channel, index);
        
    }
}

/********************************************
 @名称；DataProc_Recv
 @功能：SPI数据接收轮询函数
 @参数：none
 @返回：none
*********************************************/
void DataProc_Recv(void)
{
    uint8_t channel = 0;
    uint8_t dmaFlag = 0;
    uint16_t dataPos = 0;

    for (channel = 0; channel < 2*g_ChipCount; channel++)
    {
        for (dmaFlag = 0; dmaFlag < DMA_RECV_FLAG_MAX; dmaFlag++)
        {
            if (!g_dataRecvFlag[channel][dmaFlag])
            {
                continue;
            }
            
            if (DMA_RECV_FLAG_MEM_0 == dmaFlag)
            {
                dataPos = 0;
            }
            else
            {
                dataPos = RadarPara.dataLen;
            }
            
            DataProcess(channel, dmaFlag, &g_dataRecvBuf[channel][dataPos], RadarPara.dataLen);
        }
    }
}

/********************************************
 @名称；Radar_GetIsDataMerge
 @功能：返回merge模式flag
 @参数：none
 @返回：0/1
*********************************************/
uint8_t Radar_GetIsDataMerge(void)
{
	uint16_t val = 0;
    SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_COB2CH, &val);
	if(val & BIT(13))
	{
		return 1;
	}
	return 0; 
}

//uint8_t Radar_GetRxGainType(void)
//{
//#ifdef GD32_PLATFORM
//	return 1;
//#else	
//    uint8_t valRet = 0;
//    uint16_t valLpf = 0;
//    uint16_t valLna = 0;
//    SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_LPF_CHANNEL_ENABLE, &valLpf);
//    SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_LNA_CHANNEL_ENABLE, &valLna);
//    
//    if((((valLpf >> 8) & 0xa) | ((valLna >> 14) & 0x2)) != 0)
//    {
//        valRet |= 1;
//    }
//    
//    if((((valLpf >> 8) & 0x5) | ((valLna >> 14) & 0x1)) != 0)
//    {
//        valRet |= 2;
//    }   
//    
//    if(SPI4_Read(I2C_ADDR_BanYan_Chip1, BANYAN_LPF_CHANNEL_ENABLE, &valLpf) == HAL_OK)
//    {  
//        SPI4_Read(I2C_ADDR_BanYan_Chip1, BANYAN_LNA_CHANNEL_ENABLE, &valLna);
//        if((((valLpf >> 8) & 0xa) | ((valLna >> 14) & 0x2)) != 0)
//        {
//            valRet |= 4;
//        }
//        
//        if((((valLpf >> 8) & 0x5) | ((valLna >> 14) & 0x1)) != 0)
//        {
//            valRet |= 8;
//        }      
//    }

//	return valRet; 
//#endif
//}

/********************************************
 @名称；Radar_GetDataFormat
 @功能：返回接收格式flag
 @参数：none
 @返回：0/1
*********************************************/
uint8_t Radar_GetDataFormat(void)
{
    uint16_t val = 0;
    SPI4_Read(I2C_ADDR_BanYan_Chip0, BANYAN_DIG_HEAD_INFO, &val);  
    if(val & BANYANE_DATA_FORMAT)  
    {
        return 0;
    }
    else
    {
        return 1;
    }

}

static int calc_gcd(int x, int y)
{
    if ((x <= 0) || (y <= 0))
    {
        return 1;
    }

    while (x != y)
    {
        if (x > y)
        {
            x = x % y;
            if (x == 0)
            {
                return y;
            }
        }
        else
        {
            y = y % x;
            if (y == 0)
            {
                return x;
            }
        }
    }
    return x;
}

static int calc_lcm(int x, int y)
{
    int temp1 = calc_gcd(x, y);  
    return (x / temp1 * y);
}

/********************************************
 @名称；GetRadarPara
 @功能：配置radarPara结构体
 @参数：radarPara，雷达参数结构体
        dataType，数据类型
 @返回：0/1
*********************************************/
static int8_t GetRadarPara(RADAR_PARA_T *radarPara, uint8_t dataType)
{
#if defined (EVBKS5_E)
    uint8_t dfftDataNum = 0;
    uint8_t dfftChirpNum = 0;
#endif

    if (NULL == radarPara)
    {
        return FAIL;
    }
    
    if(dataType < DATA_TYPE_MAX)
    {
        radarPara->dataType = dataType;
    }
    else
    {
        radarPara->dataType = Radar_GetDataType();
    }  
    
//    radarPara->dataType = Radar_GetDataType();
//    radarPara->dataType = DATA_TYPE_DSRAW;
    
    radarPara->mergeData = Radar_GetIsDataMerge();
//    radarPara->mergeData = 0;
    
    radarPara->rxType = Radar_GetRxGainType();

    radarPara->newDataFormat = Radar_GetDataFormat();
//    radarPara->newDataFormat = 1;
    
#if defined(CONFIG_DEBUG)
    printf("dataType: %d\r\n", radarPara->dataType);
#endif
    
    switch (radarPara->dataType)
    {
        case DATA_TYPE_DSRAW:
            radarPara->dataLen = Radar_GetRawPoint() * 2 * 2 + SPI_FRAME_HLEN + SPI_FRAME_TLEN;  /*16 bit, IQ*/
//            radarPara->dataLen = RAW_POINT_512 * 2 * 2 + SPI_FRAME_HLEN + SPI_FRAME_TLEN;  /*16 bit, IQ*/
        
            break;
        
        case DATA_TYPE_FFT:
            radarPara->dataLen = Radar_GetFftPoint() * 2 * 2 + SPI_FRAME_HLEN + SPI_FRAME_TLEN;  /*16 bit, IQ*/
            break;
        
#if defined (EVBKS5_E) 
        case DATA_TYPE_DFFT:
            dfftDataNum = Radar_GetDfftDataNum();
            dfftChirpNum = Radar_GetDfftChirpNum();
#if defined(CONFIG_DEBUG)
            printf("dfftDataNum: %d, dfftChirpNum: %d\r\n", dfftDataNum, dfftChirpNum);
#endif
            radarPara->dataLen = dfftDataNum * 2 * 2 * dfftChirpNum + SPI_FRAME_HLEN + SPI_FRAME_TLEN;  /*16 bit, IQ*/
            break;
#endif
        
        case DATA_TYPE_DFFT_PEAK:
            radarPara->dataLen = Radar_GetDfftPeakSize() + SPI_FRAME_HLEN + SPI_FRAME_TLEN;
            break;
        
        default:
            printf("Error: unsupport dataType\r\n");
            return FAIL;
    }

#if defined(CONFIG_DEBUG)
    printf("dataLen: %d\r\n", radarPara->dataLen);
#endif
    if (radarPara->dataLen > SPI_FRAME_LEN_MAX)
    {
        printf("Error: dataLen is too long\r\n");
        return FAIL;
    }

    radarPara->chirpNum = Radar_GetOneFrameChirpNum();
//    radarPara->chirpNum = 64;
#if defined(CONFIG_DEBUG)
    printf("chirpNum: %d\r\n", radarPara->chirpNum);
#endif

    return OK;
}

/********************************************
 @名称；DataProc_Init
 @功能：SPI数据接收初始化
 @参数：none
 @返回：none
*********************************************/
void DataProc_Init(void)
{
    uint8_t channel = 0;
    
    memset(&RadarDataParse, 0 ,sizeof(RadarDataParse));
    memset(&RadarPara, 0 ,sizeof(RadarPara));

    if (FAIL == GetRadarPara(&RadarPara, Radar_GetDataTypeByRegisterList()))
    {
        //RunFailed((uint8_t *)__FILE__, __LINE__);
    }
    
    nRecvCntMax = calc_lcm(RadarPara.chirpNum, System_GetUploadSampleRate()) * g_TxCount;

    for (channel = 0; channel < CHANNEL_MAX; channel++)
    {
        RadarDataParse[channel].needCopyLen = RadarPara.dataLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
        if (DATA_TYPE_FFT == RadarPara.dataType || DATA_TYPE_DSRAW == RadarPara.dataType)
        {
            nRecvCnt[channel] = -1;
            needCheckIndex[channel] = 1;      
        }
        else
        {
            nRecvCnt[channel] = 0;
            needCheckIndex[channel] = 0;   
        }
    }
    
    uint16_t uSpiDataLen = RadarPara.mergeData ? RadarPara.dataLen * 4 : RadarPara.dataLen * 2;
    SPI4_DeInit();
		SPI_Init(uSpiDataLen);            /*radar data received by spi dma, ping-pang buffer*/


}

/********************************************
 @名称；DataProc_NeedReconfig
 @功能：SPI数据接收重新初始化
 @参数：dataType，数据类型
 @返回：none
*********************************************/
uint8_t DataProc_NeedReconfig(uint8_t dataType)
{    
    uint8_t channel = 0;
    uint8_t needReconfig = 0;
    RADAR_PARA_T radarParaTmp = {0};
    
    if (FAIL == GetRadarPara(&radarParaTmp, dataType))
    {
        //RunFailed((uint8_t *)__FILE__, __LINE__);
    }

    RadarPara.chirpNum = radarParaTmp.chirpNum;
    RadarPara.mergeData = radarParaTmp.mergeData; 
    RadarPara.rxType = radarParaTmp.rxType;
    RadarPara.newDataFormat = radarParaTmp.newDataFormat;
    
    nRecvCntMax = calc_lcm(RadarPara.chirpNum, System_GetUploadSampleRate())  * g_TxCount;
    if ((radarParaTmp.dataType != RadarPara.dataType) || (radarParaTmp.dataLen != RadarPara.dataLen))
    {
        needReconfig = 1;
        RadarPara.dataType = radarParaTmp.dataType;
        RadarPara.dataLen = radarParaTmp.dataLen;
        memset(&RadarDataParse, 0 ,sizeof(RadarDataParse));
    }
    
//    memset(&RadarDataParse, 0 ,sizeof(RadarDataParse));
    for (channel = 0; channel < CHANNEL_MAX; channel++)
    {
        RadarDataParse[channel].needCopyLen = RadarPara.dataLen - SPI_FRAME_HLEN - SPI_FRAME_TLEN;
        if (DATA_TYPE_FFT == RadarPara.dataType || DATA_TYPE_DSRAW == RadarPara.dataType)
        {
            nRecvCnt[channel] = -1;
            needCheckIndex[channel] = 1;      
        }
        else
        {
            nRecvCnt[channel] = 0;
            needCheckIndex[channel] = 0;   
        }
    }

    return needReconfig;
}

/********************************************
 @名称；DataProc_GetRadarDataType
 @功能：返回雷达数据类型
 @参数：none
 @返回：none
*********************************************/
uint8_t DataProc_GetRadarDataType(void)
{    
    return RadarPara.dataType;
}

/********************************************
 @名称；DataProc_GetRadarDataLen
 @功能：返回雷达数据长度
 @参数：none
 @返回：none
*********************************************/
uint16_t DataProc_GetRadarDataLen(void)
{    
    return RadarPara.dataLen;
}

#ifdef STM32_PLATFORM
osThreadId dataProcTaskHandle;
QueueHandle_t radarDataQueue[CHANNEL_MAX];
osThreadId mcuSpecialFuncTaskHandle;

/********************************************
 @名称；DataProc_Send2RadarDataQueue
 @功能：发送数据至队列
 @参数：channel，接收通道
        radarData，雷达数据指针
 @返回：none
*********************************************/
void DataProc_Send2RadarDataQueue(uint8_t channel, void *radarData)
{
    BaseType_t xHigherPriorityTaskWoken = 0;
    
    if (channel >= CHANNEL_MAX || NULL == radarData)
    {
        return;
    }
    
    if (xQueueSendFromISR(radarDataQueue[channel], radarData, &xHigherPriorityTaskWoken) != pdPASS) 
    {
        Indicator_RadarDataRecvOverFlow();
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

/********************************************
 @名称；DataProcTask
 @功能：SPI数据接收任务初始化
 @参数：none
 @返回：none
*********************************************/
void DataProcTask(void const * argument)
{
    RADAR_DATA_T radarData = {0};
    uint16_t loop = 0;
        
    while(1)
    { 
        for (loop = 0; loop < 2*g_ChipCount; loop++)
        {
            if (DATA_TYPE_DFFT_PEAK == RadarPara.dataType
                || RadarPara.mergeData)
            {
                if (loop % 2 == 1 )
                {
                    continue;
                }
                else if(BIT(loop) > RadarPara.rxType)
                {
                    continue;
                }
            }
            else if((RadarPara.rxType & BIT(loop)) == 0)
            {                
                continue;     
            }
            
            if(!uxQueueSpacesAvailable(radarDataQueue[loop]))
            {
                Indicator_RadarDataRecvOverFlow();
                DataProc_ResetRadarDataQueue(); 
            }   

            if (xQueueReceive(radarDataQueue[loop], &radarData, portMAX_DELAY))
            {
                if (CmdProc_InCmdMode())
                {
                    break;
                }
                
                gDataprocessFlag = 1;  
                CACHE_InvalDCache((uint32_t*)radarData.buf, radarData.len); 
                                    
                if(DATA_TYPE_DFFT_PEAK != RadarPara.dataType &&
                    RadarPara.mergeData)
                {
                    uint32_t* pBuf0 = NULL;
                    uint32_t* pBuf2 = NULL;
                    if(sysPara.useMcuPackWrapper)
                    {
                        pBuf0 = (uint32_t*)&g_dataRecvBuf[loop][SPI_FRAME_WRAPPER_SHIFT];
                        pBuf2 = (uint32_t*)(&g_dataRecvBuf[loop][SPI_FRAME_LEN_MAX + SPI_FRAME_WRAPPER_SHIFT]);
                    }
                    else
                    {
                        pBuf0 = (uint32_t*)g_dataRecvBuf[loop];
                        pBuf2 = (uint32_t*)(&g_dataRecvBuf[loop][SPI_FRAME_LEN_MAX]);
                    }
                    uint32_t* pBuf1 = (uint32_t*)radarData.buf;
                    uint32_t countTemp = radarData.len / 8;
                    for(uint16_t i = 0; i < countTemp; i++)
                    {
                        pBuf0[i] = pBuf1[2 * i];
                        pBuf2[i] = pBuf1[2 * i + 1];
                    }
                    
                    DataProcess(loop, radarData.dmaFlag, (uint8_t*)pBuf0, countTemp * 4);
                    DataProcess(loop + 1, radarData.dmaFlag, (uint8_t*)pBuf2, countTemp * 4);
                }
                else
                {
                    DataProcess(loop, radarData.dmaFlag, radarData.buf, radarData.len);
                }
                gDataprocessFlag = 0;
            }
        }
    } 
}

/************************************************************************
 @名称；DataProc_ResetRadarDataQueue
 @功能：重置spi数据队列
 @参数：none
 @返回：none
*************************************************************************/
void DataProc_ResetRadarDataQueue(void)
{
	uint16_t loop = 0;
	for (loop = 0; loop < CHANNEL_MAX; loop++)
    {
		xQueueReset(radarDataQueue[loop]);
    }
}

/************************************************************************
 @名称；McuSepcialFuncTask
 @功能：读取功率和温度，USB输出供上位机进行显示
 @参数：none
 @返回：none
*************************************************************************/
/* 
    温度读取步骤：
    radar->readReg(0x71, R71);
    value = R71;
    value = value & 0x07FF;
    value = value | 0x5800;
    radar->writeReg(0x71, value);
    radar->readReg(0x73, temp_code);
    temp_code = temp_code & 0x03FF;
    temp_value = -0.746 * temp_code + 506.716;      //temp_value  unit:度
    radar->writeReg(0x71, R71);
*/
void McuSepcialFuncTask(void const * argument)
{
    uint16_t reg77[4] = {0};
    uint16_t reg71[4] = {0};
    uint16_t reg73[4] = {0};
    uint16_t regTemp = 0;
    uint8_t i;
    uint8_t valueValid = 0;

    while(1)
    {
        
//        if(!sysPara.useMcuPackWrapper || ((mcuSpecialFunc.tempReportGap == 0) && (mcuSpecialFunc.powerReportGap == 0)))
		if(!sysPara.useMcuPackWrapper)
        {
            xSemaphoreTake(g_McuSpecialFuncSig, portMAX_DELAY);
        }

        if(sysPara.useMcuPackWrapper)
        {
            if ((!CmdProc_InCmdMode())  && (flag_read_TS == 1) )
            {
							
							  flag_read_TS = 0;
							  SPI_DeInit();
                SPI4_Config_Init();

                valueValid = 1;
                for(i = 0; i < sysPara.chipNum;i++)
                {
                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x77, &reg77[i]);
                    regTemp = reg77[i];
                    regTemp |= 0x500; // Set bit8, bit10
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x77, regTemp);
                    
                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x71, &reg71[i]);
                    
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, 0x5021);

                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, 0x5821);
                    
                    /*regTemp &= 0x07FF;
                    regTemp |= 0x5800;
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, regTemp);     */
                    
                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x73, &reg73[i]);

                    if((reg73[i] & 0x800) == 0)
                    {
                        valueValid = 0;
                    }
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, reg71[i]);

                    *(volatile uint32_t *)&g_ChipTempDataPack[16 * i] = SPI_FRAME_WRAPPER_HEAD;
                    *(volatile uint16_t *)&g_ChipTempDataPack[16 * i + 4] = 4;
                    g_ChipTempDataPack[16 * i + 6] = 0x80;
                    g_ChipTempDataPack[16 * i + 7] = sysPara.chipAddr[i];
                    *(volatile uint32_t *)&g_ChipTempDataPack[16 * i + 8] = (uint32_t)reg73[i];
                    *(volatile uint32_t *)&g_ChipTempDataPack[16 * i + 12] = SPI_FRAME_WRAPPER_TAIL;
                }
								SPI_DeInit();
                uint16_t uSpiDataLen = RadarPara.mergeData ? RadarPara.dataLen * 4 : RadarPara.dataLen * 2;
		            SPI_Init(uSpiDataLen); 
                if(valueValid)
                {   
////					vTaskDelay(mcuSpecialFunc.tempReportGap);
                    UsbTransfer_Send(g_ChipTempDataPack, 16 * sysPara.chipNum, 0);

                }
            }
            
        }
        if(sysPara.useMcuPackWrapper)
        {
            if ((!CmdProc_InCmdMode())  && (flag_read_PD == 1))
            {
							  flag_read_PD = 0;
							  SPI_DeInit();
                SPI4_Config_Init();
							
                valueValid = 1;
                for(i = 0; i < sysPara.chipNum;i++)
                {

                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x77, &reg77[i]);
                    regTemp = reg77[i];
                    regTemp |= 0x500; // Set bit8, bit10
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x77, regTemp);
                    
                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x71, &reg71[i]);
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, 0xE021);

                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, 0xE821);

                    SPI4_Read(sysPara.chipAddr[i] * 2, 0x73, &reg73[i]);
                    
                    if((reg73[i] & 0x800) == 0)
                    {
                        valueValid = 0;
                    }
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x71, reg71[i]);
                    SPI4_Write(sysPara.chipAddr[i] * 2, 0x77, reg77[i]);
                    
                    *(volatile uint32_t *)&g_ChipPowerDataPack[16 * i] = SPI_FRAME_WRAPPER_HEAD;
                    *(volatile uint16_t *)&g_ChipPowerDataPack[16 * i + 4] = 4;
                    g_ChipPowerDataPack[16 * i + 6] = 0x81;
                    g_ChipPowerDataPack[16 * i + 7] = sysPara.chipAddr[i];
                    *(volatile uint32_t *)&g_ChipPowerDataPack[16 * i + 8] = (uint32_t)reg73[i];
                    *(volatile uint32_t *)&g_ChipPowerDataPack[16 * i + 12] = SPI_FRAME_WRAPPER_TAIL;
                }
								
								SPI_DeInit();
                uint16_t uSpiDataLen = RadarPara.mergeData ? RadarPara.dataLen * 4 : RadarPara.dataLen * 2;
		            SPI_Init(uSpiDataLen); 
								
                if(valueValid)
                {
//									  vTaskDelay(mcuSpecialFunc.powerReportGap);
                    UsbTransfer_Send(g_ChipPowerDataPack, 16 * sysPara.chipNum, 0);
                }
            }
            
        }
    }
}

/************************************************************************
 @名称；DataProc_TaskInit
 @功能：spi数据接收任务初始化
 @参数：none
 @返回：none
*************************************************************************/
void DataProc_TaskInit(void)
{
    uint16_t loop = 0;

    for (loop = 0; loop < CHANNEL_MAX; loop++)
    {
        radarDataQueue[loop] = xQueueCreate(RADAR_DATA_QUEUE_SIZE, sizeof(RADAR_DATA_T));
    }

    osThreadDef(dataProcTask, DataProcTask, osPriorityHigh, 0, DATA_PROC_STACK_SIZE);
    dataProcTaskHandle = osThreadCreate(osThread(dataProcTask), NULL);
    if (NULL == dataProcTaskHandle)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
    
    osThreadDef(mcuSepcialFuncTask, McuSepcialFuncTask, osPriorityNormal, 0, 256);
    mcuSpecialFuncTaskHandle = osThreadCreate(osThread(mcuSepcialFuncTask), NULL);
    if (NULL == mcuSpecialFuncTaskHandle)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
    
    if(sysPara.useMcuPackWrapper)
    {
        mcuSpecialFunc.isPowerCheckEnable = 1;
        mcuSpecialFunc.isTempCheckEnable = 1;
    }
    else
    {
        mcuSpecialFunc.isPowerCheckEnable = 0;
        mcuSpecialFunc.isTempCheckEnable = 0;
    }
    
    mcuSpecialFunc.powerReportGap = 0;
    mcuSpecialFunc.tempReportGap = 0;

}
#endif

