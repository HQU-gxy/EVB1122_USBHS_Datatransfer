/**
 ******************************************************************************
 * @file           : gaincalibration.h
 * @author         : iclm team
 * @brief          : gaincalibration header file
 ******************************************************************************
 */
#ifndef __GAINCALIBRATION_H__
#define __GAINCALIBRATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SHORT_MIN (-32768)
#define SHORT_MAX 32767

typedef enum eGAIN_CALIBRATION_STATE {
	GAIN_CALIBRATION_STATE_NONE = 0,
	GAIN_CALIBRATION_STATE_IVCM,
	GAIN_CALIBRATION_STATE_TX,
	GAIN_CALIBRATION_STATE_RX1,
	GAIN_CALIBRATION_STATE_RX2,
	GAIN_CALIBRATION_STATE_COMPLETE
} GAIN_CALIBRATION_STATE;

typedef enum eIVCM_CALIBRATION_STATE {
	IVCM_CALIBRATION_STATE_NONE = 0,
	IVCM_CALIBRATION_STATE_CALIBRATION,
	IVCM_CALIBRATION_STATE_CHECKGAIN,
	IVCM_CALIBRATION_STATE_COMPLETE,
} IVCM_CALIBRATION_STATE;

void StartGainCalibration(void);
uint8_t IsGainCalibration(void);
#ifdef __cplusplus
}
#endif

#endif
