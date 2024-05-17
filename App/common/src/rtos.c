/**
  ******************************************************************************
  * @file    rtos.c
  * @author  iclm team
  * @brief   rtos services
  ******************************************************************************
  */
#include "rtos.h"
#include "cmsis_os.h"
#include "dataprocess.h"
#include "usb_transfer.h"
#include "cmdprocess.h"
#ifdef STM32_PLATFORM
#include "I2CStressTest.h"
#endif


static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}                   

/************************************************************************
 @名称；RTOS_Init
 @功能：rtos初始化
 @参数：none
 @返回：none
*************************************************************************/
void RTOS_Init(void) 
{
    DataProc_TaskInit();
    UsbTransfer_TaskInit();
    CmdProc_TaskInit();

}


