/**
  ******************************************************************************
  * @file           : I2CStressTest.c
  * @author         : iclm team
  * @brief          : I2C Stress testing
  ******************************************************************************
  */
#include <stdio.h>
#include "I2CStressTest.h"
#include "global_conf.h"
#include "utilities.h"
#include "cmdprocess.h"
#include "platform.h"
#include "banyan.h"
#ifdef STM32_PLATFORM
#include "cmsis_os.h"
#include "rtos.h"
#endif

#ifdef STM32_PLATFORM
osThreadId I2CStressTestTaskHandle;
QueueHandle_t I2CTestQueue;

static I2CTEST_RESULT_T testResult = {0};
static uint8_t forceStop = 0;

/************************************************************************
 @名称；I2CStressTest_Send2I2CTestDataQueue
 @功能：发送I2C数据至数据队列
 @参数：I2CData，数据指针
 @返回：none
*************************************************************************/
void I2CStressTest_Send2I2CTestDataQueue(void *I2CData)
{
    BaseType_t xHigherPriorityTaskWoken = 0;
    if (xQueueSendFromISR(I2CTestQueue, I2CData, &xHigherPriorityTaskWoken) != pdPASS) 
    {
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

/************************************************************************
 @名称；I2CStressTest_Process
 @功能：I2C压测函数
 @参数：buf，数据指针
        len，数据长度
 @返回：none
*************************************************************************/
void I2CStressTest_Process(uint8_t* buf, uint16_t len)
{
    CMD_I2CTEST_T* cmdI2CTest = NULL;
    uint16_t regNum = 0;
    uint32_t executionCount;
    uint16_t devAddress = I2C_ADDR_BanYan_Chip0;
    uint16_t loop = 0;
    I2CTEST_ITEM_T *I2cTestItem = NULL;
    uint8_t alwaysTest = 0;
    
    if (NULL == buf || 0 == len)
    {
        return;
    }
    
    cmdI2CTest = (CMD_I2CTEST_T*)buf;
    devAddress = cmdI2CTest->devAddress << 1;
    regNum = cmdI2CTest->itemCount;
    executionCount = cmdI2CTest->executionCount;
    //I2C_ResetSpeed(cmdI2CTest->speed);
    
    if(executionCount == 0)
    {
        alwaysTest = 1;
    }
    
    while(alwaysTest || executionCount--)
    {
        for (loop = 0; loop < regNum; loop++) 
        {
            if(forceStop)
            {
                return;
            }
            
            testResult.curTestCount++;
            
            I2cTestItem = (I2CTEST_ITEM_T*)((uint8_t*)buf + 10 + (loop * sizeof(I2CTEST_ITEM_T)));
            
            switch(I2cTestItem->mode)
            {
                case I2CTEST_RWMODE_READONLY:
                {
                    uint16_t regVal = 0;
                    if(I2C_Read(devAddress, (uint8_t)(I2cTestItem->regAddress), &regVal) != HAL_OK)
                    {
                        testResult.curTestErrCount++;
                    } 
                    break;
                }
                case I2CTEST_RWMODE_WRITEONLY:
                {
                    if(I2C_Write(devAddress, (uint8_t)(I2cTestItem->regAddress), I2cTestItem->regValue) != HAL_OK)
                    {
                        testResult.curTestErrCount++;
                    } 
                    break;
                }
                case I2CTEST_RWMODE_WRITEANDREAD:
                {
                    uint16_t regVal = 0;
                    if(I2C_Write(devAddress, (uint8_t)(I2cTestItem->regAddress), I2cTestItem->regValue) != HAL_OK)
                    {
                        testResult.curTestErrCount++;
                        break;
                    }
                    
                    if((I2C_Read(devAddress, (uint8_t)(I2cTestItem->regAddress), &regVal) != HAL_OK) ||
                        regVal != I2cTestItem->regValue)
                    {
                        testResult.curTestErrCount++;
                        break;
                    } 
                    break;
                }
                default:
                {
                    break;
                }
            } 
            vTaskDelay(I2cTestItem->delayMs);
        }
    }
}

/************************************************************************
 @名称；I2CStressTest_IsStressTestProcessRunning
 @功能：返回I2C是否运行
 @参数：none
 @返回：none
*************************************************************************/
uint8_t I2CStressTest_IsStressTestProcessRunning()
{
	return testResult.running;
}

/************************************************************************
 @名称；I2CStressTest_StressTestProcessEnd
 @功能：结束I2C压测
 @参数：none
 @返回：none
*************************************************************************/
void I2CStressTest_StressTestProcessEnd()
{
	//printf("I2CStressTest_StressTestProcessEnd curTestCount=%d, curTestErrCount=%d \r\n", testResult.curTestCount, testResult.curTestErrCount);
	forceStop = 0;
	ResetI2CTestDataBuf();
}

/************************************************************************
 @名称；I2CStressTest_ForceStopI2CStressTestProcess
 @功能：强制结束I2C压测
 @参数：none
 @返回：none
*************************************************************************/
uint8_t I2CStressTest_ForceStopI2CStressTestProcess()
{
	forceStop = 1;
	uint8_t timeout = 100;
	while(testResult.running && timeout--)
	{
		HAL_Delay(1);
	}
	forceStop = 0; 
	return timeout;
}

/************************************************************************
 @名称；I2CStressTest_GetI2CTestResult
 @功能：返回压测结果
 @参数：none
 @返回：testResult，压测结果
*************************************************************************/
//I2CTEST_RESULT_T I2CStressTest_GetI2CTestResult()
//{
//	return testResult;
//}

/************************************************************************
 @名称；I2CStressTest_ResetRunningState
 @功能：复位I2C运行状态
 @参数：none
 @返回：none
*************************************************************************/
void I2CStressTest_ResetRunningState()
{
	testResult.curTestCount = 0;
	testResult.curTestErrCount = 0;
	testResult.running = 1;
}

/************************************************************************
 @名称；I2CStressTestTask
 @功能：I2C压测任务
 @参数：none
 @返回：none
*************************************************************************/
void I2CStressTestTask(void const * argument)
{
	I2CTEST_DATA_T I2CTestData = {0};
        
    while(1)
    { 
        if (xQueueReceive(I2CTestQueue, &I2CTestData, portMAX_DELAY))
        {
            I2CStressTest_Process(I2CTestData.buf, I2CTestData.len);
			I2CStressTest_StressTestProcessEnd();
			testResult.running = 0; 
        }
    } 
}

/************************************************************************
 @名称；I2CStressTest_TaskInit
 @功能：I2C压测任务初始化
 @参数：none
 @返回：none
*************************************************************************/
void I2CStressTest_TaskInit(void)
{
	I2CTestQueue = xQueueCreate(10, sizeof(I2CTEST_DATA_T));
	
    osThreadDef(I2CStressTestTask, I2CStressTestTask, osPriorityHigh, 0, 512);
    I2CStressTestTaskHandle = osThreadCreate(osThread(I2CStressTestTask), NULL);
    if (NULL == I2CStressTestTaskHandle)
    {
        RunFailed((uint8_t *)__FILE__, __LINE__);
    }
}
#endif
