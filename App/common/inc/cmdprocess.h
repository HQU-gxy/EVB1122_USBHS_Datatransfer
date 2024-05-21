/**
 ******************************************************************************
 * @file           : cmdprocess.h
 * @author         : iclm team
 * @brief          : command process header file
 ******************************************************************************
 */
#ifndef __CMDPROCESS_H__
#define __CMDPROCESS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "global_conf.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern uint8_t gDataprocessFlag;

extern SemaphoreHandle_t g_McuSpecialFuncSig;

typedef enum {
	MCU_RUN_NONE       = 0,
	MCU_RUN_BOOTLOADER = 1,
	MCU_RUN_APP0       = 2,
	MCU_RUN_APP1       = 4,
	MCU_RUN_APP2       = 8,
	MCU_RUN_APP3       = 16
} MCU_RUN_APP_T;

typedef enum {
	CMD_STATE_HEAD0 = 0,
	CMD_STATE_HEAD1,
	CMD_STATE_HEAD2,
	CMD_STATE_HEAD3,
	CMD_STATE_LEN0,
	CMD_STATE_LEN1,
	CMD_STATE_DATA,
	CMD_STATE_TAIL0,
	CMD_STATE_TAIL1,
	CMD_STATE_TAIL2,
	CMD_STATE_TAIL3
} cmdStateEnum;

typedef enum {
	CMD_HEAD_0 = 0,
	CMD_HEAD_1,
	CMD_HEAD_2,
	CMD_HEAD_3,
	CMD_HEAD_MAX
} cmdHeadEnum;

typedef enum {
	CMD_TAIL_0 = 0,
	CMD_TAIL_1,
	CMD_TAIL_2,
	CMD_TAIL_3,
	CMD_TAIL_MAX
} cmdTailEnum;

typedef enum {
	NOPCONFIG_ENABLE = 0,
	NOPCONFIG_STARTTIME,
	NOPCONFIG_STOPTIME,
	NOPCONFIG_STARTREGISTER,
	NOPCONFIG_STOPREGISTER,
} nopCofigParaEnum;

#ifdef STM32_PLATFORM
#define CMD_LEN_MAX         (1024)
#define CMD_REG_MAX         (500)
#define CMD_PARA_MAX        (200)
#define CMD_DATA_QUEUE_SIZE (CMD_RECV_BUF_MAX + 2)
#else
#define CMD_LEN_MAX         (64)
#define CMD_REG_MAX         (25)
#define CMD_PARA_MAX        (13)
#define CMD_DATA_QUEUE_SIZE (2)
#endif


#define CMD_LENGTH_LEN           (2)
#define CMD_TYPE_LEN             (2)
#define CMD_STATUS_LEN           (2)
#define CMD_STR_LEN              (2)
#define CMD_OVERHEAD_LEN         (CMD_HEAD_MAX + CMD_LENGTH_LEN + CMD_TYPE_LEN + CMD_TAIL_MAX)
#define CMD_BUF_LEN              (CMD_LEN_MAX - CMD_HEAD_MAX - CMD_TAIL_MAX - CMD_LENGTH_LEN)
#define CMD_LEN_HIGH_POS         (8)
#define CMD_DATA_POS_OLD         (CMD_HEAD_MAX + CMD_LENGTH_LEN + CMD_TYPE_LEN)
#define CMD_DATA_POS             (CMD_HEAD_MAX + CMD_LENGTH_LEN + CMD_TYPE_LEN + CMD_STATUS_LEN)

#define CMD_DEV_ADDR_LEN         (2)
#define CMD_REG_ADDR_LEN         (2)
#define CMD_PARA_NAME_LEN        (2)

#define START_CFG_CMD            (0x00FF)
#define FINISH_CFG_CMD           (0x00FE)
#define READ_VER_CMD             (0x0000)
#define WRITE_REG_CMD            (0x0001)
#define READ_REG_CMD             (0x0002)
#define WRITE_MTT_CMD            (0x0003)
#define READ_MTT_CMD             (0x0004)
#define WRITE_SN_CMD             (0x0010)
#define READ_SN_CMD              (0x0011)
#define WRITE_SYS_CMD            (0x0012)
#define READ_SYS_CMD             (0x0013)
#define CASCADING_MODE_CMD       (0x0014)
#define FFT_ZEROFILL_CMD         (0x0015)
#define MCU_SPECIAL_FUNC_CMD     (0x0017)
#define START_I2C_TEST_CMD       (0x0020)
#define STOP_I2C_TEST_CMD        (0x0021)
#define GET_I2C_TEST_RESULT_CMD  (0x0022)
#define WRITE_PD_CALIBRATION_CMD (0x0026)
#define READ_PD_CALIBRATION_CMD  (0x0027)

// #define PARAM_CFG_CMD		  (0x0060)    //cmd: config algorithm param to flash
// #define PARAM_READ_CMD	  (0x0061)    //cmd: read algorithm param from flash or ram
// #define ENABLE_DEBUG_CMD	(0x0062)
// #define DISABLE_DEBUG_CMD	(0x0063)
// #define THRESHOLD_SET_CMD (0x0064)
// #define NOP_CONFIG_CMD   	(0x0065)
#define DEFAULT_PDPARA_PHASEDIFF (0)
#define DEFAULT_PDPARA1_TEMPER   (0)
#define DEFAULT_PDPARA1_POWER    (0)
#define DEFAULT_PDPARA1_REFVAL   (0)
#define DEFAULT_PDPARA1_SLOPE    (0)
#define DEFAULT_PDPARA2_TEMPER   (0)
#define DEFAULT_PDPARA2_POWER    (0)
#define DEFAULT_PDPARA2_REFVAL   (0)
#define DEFAULT_PDPARA2_SLOPE    (0)

// 跟踪模式：单目标模式，多目标模式
/*
FD FC FB FA, 02 00, 80 00, 04 03 02 01 (单人) 返回：FD FC FB FA 04 00 80 01 00 00 04 03 02 01
FD FC FB FA, 02 00, 90 00, 04 03 02 01 (多人) 返回：FD FC FB FA 04 00 90 01 00 00 04 03 02 01
*/
#define TRACK_ONE_CMD       (0x0080)
#define TRACK_TWO_CMD       (0x0090)

#define CMD_ACK_TYPE        (0x0100)
#define ACK_OK              (0)
#define ACK_FAIL            (1)

#define ACK_OK_OLD          (1)
#define ACK_FAIL_OLD        (0)

#define RESET_DELAY_TIME    (2)
#define SN_LEN              (34)

#define CMD_PROC_STACK_SIZE (512)

/* IAP 相关命令 */
// #define GET_MCU_ID_CMD          (0x0075)
// #define SOFT_GET_APP_USE_CMD    (0x0071)
// #define GET_APP_RUN_CMD         (0x0070)
// #define SOFT_RESET_CMD          (0x0074)

#define GET_MCU_ID_CMD       (0x0065)
#define SOFT_GET_APP_USE_CMD (0x0061)
#define GET_APP_RUN_CMD      (0x0060)
#define SOFT_RESET_CMD       (0x0064)
#define GET_WHICH_DEMO       (0X0066)

typedef struct CMD {
	uint16_t cmdType;
	uint16_t cmdData[0];
} CMD_T;

typedef struct CMD_REG {
	uint16_t addr;
	uint16_t val;
} CMD_REG_T;

#pragma pack(1)
typedef struct CMD_PARA {
	uint16_t type;
	int32_t val;
} CMD_PARA_T;
#pragma pack()

#pragma pack(4)
typedef struct PD_CALIBRATION_PARAM {
	uint32_t calibrated_PhaseDiff;
	uint32_t PDCalib1_Temper;
	uint32_t PDCalib1_Power;
	uint32_t PDCalib1_Refval;
	uint32_t PDCalib1_SlopeVal;
	uint32_t PDCalib2_Temper;
	uint32_t PDCalib2_Power;
	uint32_t PDCalib2_Refval;
	uint32_t PDCalib2_SlopeVal;
} PD_CALIBRATION_PARAM_T;
#pragma pack()

extern PD_CALIBRATION_PARAM_T PD_CalibrationPara;

typedef struct CMD_PARSE {
	uint8_t buf[CMD_BUF_LEN];
	uint8_t state;
	uint16_t curIndex;
	uint16_t len;
} CMD_PARSE_T;


uint16_t FillCmdAck(uint16_t *data, uint16_t dataLen, uint16_t cmdType, uint16_t status);
void CmdProc_Recv(void);
void CmdProc_Init(void);
uint8_t CmdProc_InCmdMode(void);
void CmdProc_AdcReset(void);
void CmdProc_NopConfig(void);
#ifdef STM32_PLATFORM
void CmdProc_TaskInit(void);
void CmdProc_Send2CmdDataQueue(void *cmdData);
void ResetI2CTestDataBuf(void);
void PD_ParaInit(void);
uint32_t System_GetPDCaliParaLen(void);
void *System_GetPDCaliParaAddr(void);

uint8_t CmdProc_IsInDebugMode(void);


#endif
#ifdef __cplusplus
}
#endif

#endif
