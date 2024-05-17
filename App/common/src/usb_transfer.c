/**
  ******************************************************************************
  * @file           : usb_transfer.c
  * @author         : iclm team
  * @brief          : usb transfer module
  ******************************************************************************
  */
#include <stdio.h>
#include "rtos.h"
#include "cmsis_os.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include "usb_transfer.h"
#include "usb_device.h"
#include "utilities.h"
#include "platform.h"

USBD_HandleTypeDef hUsbDeviceFS;
QueueHandle_t usbDataQueue;
osThreadId usbTransferTaskHandle;
#if defined(SUPPORT_DATA_PASSTHROUGH) || defined(SUPPORT_DYNAMIC_SYS_MODE)
static uint16_t dataIndex = 0;
static uint8_t bufIndex = 0;
static uint8_t usbBuffer[USB_BUF_CNT][USB_BUF_LEN];
#endif
static uint32_t usbPktCnt = 0;
static uint32_t usbPktCntOld = 0;

/************************************************************************
 @���ƣ�UsbTransfer_UsbPktCntCallBack
 @���ܣ�USB�������ݰ����ص�����
 @������none
 @���أ�none
*************************************************************************/
void UsbTransfer_UsbPktCntCallBack(void)
{
    static uint16_t tickCnt = 0;
    uint32_t usbPktInc = 0;

    if (0 == osKernelRunning())
    {
        return;
    }
    
    if (tickCnt % USB_PKT_CHECK_TICK == 0)
    {
        usbPktInc = usbPktCnt - usbPktCntOld;
        if (usbPktInc < USB_PKT_INC_PER_CHECK)
        {
            usbPktCnt = 0;
            LED_Off(RADAR_DATA_SEND_OVER_FLOW_LED);
        }
        usbPktCntOld = usbPktCnt;
    }
    tickCnt++;
}

//void UsbDevInit(void)
//{
//    USB_Patch();
//    
//    if (USBD_Init(&hUsbDeviceFS, &VCP_Desc, 0) != USBD_OK)
//    {
//        RunFailed((uint8_t *)__FILE__, __LINE__);
//    }
//    
//    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
//    {
//        RunFailed((uint8_t *)__FILE__, __LINE__);
//    }
//    
//    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
//    {
//        RunFailed((uint8_t *)__FILE__, __LINE__);
//    }
//    
//    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
//    {
//        RunFailed((uint8_t *)__FILE__, __LINE__);
//    }
//}

/************************************************************************
 @���ƣ�UsbTransfer_ResetUsbBuffer
 @���ܣ����USB����buffer
 @������none
 @���أ�none
*************************************************************************/
void UsbTransfer_ResetUsbBuffer(void)
{
#if defined(SUPPORT_DATA_PASSTHROUGH) || defined(SUPPORT_DYNAMIC_SYS_MODE)
	memset(usbBuffer, 0, sizeof(usbBuffer));
    dataIndex = 0;
#endif
	xQueueReset(usbDataQueue);
}

void UsbTransfer_ForceSend(void)
{
    USB_DATA_T usbData = {0};  
    usbData.len = dataIndex;
    usbData.buf = usbBuffer[bufIndex];

    if(uxQueueSpacesAvailable(usbDataQueue))
    {
        UsbTransfer_Send2UsbDataQueue(&usbData);
    }
    else
    {
        if (usbPktCnt > USB_PKT_THRESHOLD)
        {
            Indicator_RadarDataSendOverFlow();
        }
        UsbTransfer_ResetUsbBuffer();
    }

    bufIndex = (bufIndex + 1) % USB_BUF_CNT;
    dataIndex = 0;
}

#if defined(SUPPORT_DATA_PASSTHROUGH) || defined(SUPPORT_DYNAMIC_SYS_MODE)
/************************************************************************
 @���ƣ�UsbTransfer_Send
 @���ܣ�����usb����
 @������pBuffer��������buffer
        uBufLen�����ݳ���
        bDirectSend���Ƿ���������flag
 @���أ�none
*************************************************************************/
uint8_t UsbTransfer_Send(uint8_t* pBuffer, uint16_t uBufLen, uint8_t bDirectSend)
{	
	USB_DATA_T usbData = {0};
    uint16_t usbLeftDataLen = 0;
    uint16_t usbCopyDataLen = 0;
    uint8_t bSend = 0;
    
    while (uBufLen > 0)
    {
        usbLeftDataLen = USB_BUF_LEN - dataIndex;
        usbCopyDataLen = usbLeftDataLen < uBufLen ? usbLeftDataLen : uBufLen;
        memcpy(&usbBuffer[bufIndex][dataIndex], pBuffer, usbCopyDataLen);
        pBuffer += usbCopyDataLen;
        uBufLen -= usbCopyDataLen;
        dataIndex += usbCopyDataLen;

		if (bDirectSend || dataIndex == USB_BUF_LEN)
        {
            usbData.len = dataIndex;
            usbData.buf = usbBuffer[bufIndex];
            if(uxQueueSpacesAvailable(usbDataQueue))
            {
                UsbTransfer_Send2UsbDataQueue(&usbData);
            }
            else
            {
                if (usbPktCnt > USB_PKT_THRESHOLD)
                {
                    Indicator_RadarDataSendOverFlow();
                }
                UsbTransfer_ResetUsbBuffer();
            }
            bSend = 1;
            bufIndex = (bufIndex + 1) % USB_BUF_CNT;
            dataIndex = 0;
        }
    }
    return bSend;
}
#endif

/************************************************************************
 @���ƣ�UsbTransfer_Send2UsbDataQueue
 @���ܣ�����������usb���ݶ���
 @������usbData��usb����ָ��
 @���أ�none
*************************************************************************/
void UsbTransfer_Send2UsbDataQueue(void *usbData)
{	
	BaseType_t res = 0;

    if (NULL == usbData)
    {
        return;
    }
    
    res = xQueueSend(usbDataQueue, usbData, USB_SEND_WAIT);
    if (res != pdPASS)
    {
        if (usbPktCnt > USB_PKT_THRESHOLD)
        {
            Indicator_RadarDataSendOverFlow();
        }
    }
}

/************************************************************************
 @���ƣ�UsbTransferTask
 @���ܣ�usb��������
 @������none
 @���أ�none
*************************************************************************/
void UsbTransferTask(void const * argument)
{
    USB_DATA_T usbData = {0};
    
	while(1)
	{
		if (xQueueReceive(usbDataQueue, &usbData, portMAX_DELAY))
		{
			while (1)
			{
				if (CDC_Transmit_HS((uint8_t *)usbData.buf, usbData.len) == USBD_OK)
				{
				    usbPktCnt++;
					break;
				}
				else
				{
					vTaskDelay(1);
				}            
			}
		}
   }
}

/************************************************************************
 @���ƣ�UsbTransfer_TaskInit
 @���ܣ�usb���������ʼ��
 @������none
 @���أ�none
*************************************************************************/
void UsbTransfer_TaskInit(void)
{
	  MX_USB_DEVICE_Init();
	
    usbDataQueue = xQueueCreate(USB_DATA_QUEUE_SIZE, sizeof(USB_DATA_T));
    
    osThreadDef(usbTransferTask, UsbTransferTask, osPriorityAboveNormal, 0, USB_TRANSFER_STACK_SIZE);
    usbTransferTaskHandle = osThreadCreate(osThread(usbTransferTask), NULL);
    if (NULL == usbTransferTaskHandle)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
}


