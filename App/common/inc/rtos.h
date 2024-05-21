/**
 ******************************************************************************
 * @file           : rtos.h
 * @author         : iclm team
 * @brief          : rtos header file
 ******************************************************************************
 */
#ifndef __RTOS_H__
#define __RTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct RADAR_DATA {
	uint8_t *buf;
	uint8_t dmaFlag;
	uint16_t len;
} RADAR_DATA_T;

typedef struct CMD_DATA {
	uint8_t *buf;
	uint16_t len;
} CMD_DATA_T;

typedef struct USB_DATA {
	uint8_t *buf;
	uint16_t len;
} USB_DATA_T;

typedef struct FRAME_DATA_T {
	uint8_t *buf;
	uint16_t len;
} FRAME_DATA_T;

typedef struct SIG_PROC_DATA {
	uint8_t *buf;
	uint16_t len;
} SIG_PROC_DATA_T;

void RTOS_Init(void);

#ifdef __cplusplus
}
#endif

#endif
