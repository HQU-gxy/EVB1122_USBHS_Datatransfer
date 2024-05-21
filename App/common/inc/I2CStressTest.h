/**
 ******************************************************************************
 * @file           : I2CStressTest.h
 * @author         : iclm team
 * @brief          : I2CStressTest header file
 ******************************************************************************
 */
#ifndef __I2CSTRESSTEST_H__
#define __I2CSTRESSTEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct CMD_I2CTEST {
	uint16_t devAddress;
	uint16_t speed;
	uint32_t executionCount;
	uint16_t itemCount;
} CMD_I2CTEST_T;

typedef struct I2CTEST_ITEM {
	uint16_t mode;
	uint16_t regAddress;
	uint16_t regValue;
	uint16_t delayMs;
} I2CTEST_ITEM_T;

typedef enum {
	I2CTEST_RWMODE_READONLY = 0,
	I2CTEST_RWMODE_WRITEONLY,
	I2CTEST_RWMODE_WRITEANDREAD,
} I2CTEST_RWMODE;

typedef struct I2CTEST_DATA {
	uint8_t *buf;
	uint16_t len;
} I2CTEST_DATA_T;

typedef struct I2CTEST_RESULT {
	uint16_t running;
	uint32_t curTestCount;
	uint32_t curTestErrCount;
} I2CTEST_RESULT_T;

void I2CStressTest_Send2I2CTestDataQueue(void *I2CData);
uint8_t I2CStressTest_IsStressTestProcessRunning(void);
uint8_t I2CStressTest_ForceStopI2CStressTestProcess(void);
I2CTEST_RESULT_T I2CStressTest_GetI2CTestResult(void);
void I2CStressTest_TaskInit(void);
void I2CStressTest_ResetRunningState(void);
#ifdef __cplusplus
}
#endif

#endif
