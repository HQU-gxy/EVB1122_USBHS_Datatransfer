/**
  ******************************************************************************
  * @file           : usb_transfer.h
  * @author         : iclm team
  * @brief          : usb transfer header file
  ******************************************************************************
  */
#ifndef __USB_TRANSFER_H__
#define __USB_TRANSFER_H__

#ifdef __cplusplus
 extern "C" {
#endif 
#include <stdint.h>

#define USB_DATA_QUEUE_SIZE             (10)
#define USB_TRANSFER_STACK_SIZE         (512)

//2t4r 1024 point must one chirp is 16k,buffer has to be able to accommodate two chirp
//USB_BUF_LEN can set 3000
#define USB_BUF_CNT                     (USB_DATA_QUEUE_SIZE + 2)
#define USB_BUF_LEN                     (5120)
#define USB_SEND_WAIT                   (1)

#define USB_PKT_THRESHOLD               (20)
#define USB_PKT_CHECK_TICK              (300)
#define USB_PKT_INC_PER_CHECK           (1)

void UsbTransfer_TaskInit(void);
uint8_t UsbTransfer_Send(uint8_t* pBuffer, uint16_t uBufLen, uint8_t bDirectSend);
void UsbTransfer_Send2UsbDataQueue(void *usbData);
void UsbTransfer_UsbPktCntCallBack(void);
void UsbTransfer_ResetUsbBuffer(void);
void UsbTransfer_ForceSend(void);
#ifdef __cplusplus
}
#endif

#endif

