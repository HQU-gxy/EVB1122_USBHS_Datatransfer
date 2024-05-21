/**
 ******************************************************************************
 * @file           : system.c
 * @author         : iclm team
 * @brief          : system module
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "fftzerofill.h"

static RADAR_FFT_ZERO_FILL_T RadarFFTZeroFillPara = {
	.enableZeroFill = 0,
	.rawSampleSize  = 256,
	.fftCalcSize    = 256,
	.fftCalcMap     = FFT_CALCSIZE_256,
	.fftOffset      = 0,
	.fftOutputSize  = 64,
	.fftWindowType  = 1,
	.rawValueScale  = 16};

uint8_t FFTZero_GetEnable(void) {
	return RadarFFTZeroFillPara.enableZeroFill;
}

uint16_t FFTZero_GetRawSampleSize(void) {
	return RadarFFTZeroFillPara.rawSampleSize;
}

fftCalSizeEnum FFTZero_GetCalcMap(void) {
	return RadarFFTZeroFillPara.fftCalcMap;
}

uint16_t FFTZero_GetCalcSize(void) {
	return RadarFFTZeroFillPara.fftCalcSize;
}

uint16_t FFTZero_GetOffset(void) {
	return RadarFFTZeroFillPara.fftOffset;
}

uint16_t FFTZero_GetOutputSize(void) {
	return RadarFFTZeroFillPara.fftOutputSize;
}

void FFTZero_ResizeOffsetAndOutput(void) {
	uint16_t uMaxSize = RadarFFTZeroFillPara.fftCalcSize;
	if (RadarFFTZeroFillPara.fftOutputSize > uMaxSize) {
		RadarFFTZeroFillPara.fftOutputSize = uMaxSize;
	}

	if (RadarFFTZeroFillPara.fftOffset + RadarFFTZeroFillPara.fftOutputSize > uMaxSize) {
		RadarFFTZeroFillPara.fftOffset = uMaxSize - RadarFFTZeroFillPara.fftOutputSize;
	}
}

uint8_t FFTZero_GetWindowType(void) {
	return RadarFFTZeroFillPara.fftWindowType;
}

uint16_t FFTZero_GetRawValueScale(void) {
	return RadarFFTZeroFillPara.rawValueScale;
}

fftCalSizeEnum FFTZero_log2(uint16_t val) {
	uint16_t calsize = 0;
	while (val >>= 1) {
		calsize++;
	}
	calsize -= 4;
	return (fftCalSizeEnum)calsize;
}

int8_t fftzerofill_ParaUpdate(uint16_t type, int32_t val) {
	switch (type) {
	case FFTZERO_ENABLE:
		RadarFFTZeroFillPara.enableZeroFill = (int16_t)val;
		break;
	case FFTZERO_RAWSAMPLESIZE:
		RadarFFTZeroFillPara.rawSampleSize = (int16_t)val;
		break;
	case FFTZERO_FFTCALCSIZE:
		RadarFFTZeroFillPara.fftCalcSize = (int16_t)val;
		RadarFFTZeroFillPara.fftCalcMap  = FFTZero_log2((int16_t)val);
		break;
	case FFTZERO_FFTOUTPUTOFFSET:
		RadarFFTZeroFillPara.fftOffset = (int16_t)val;
		break;
	case FFTZERO_FFTOUTPUTSIZE:
		RadarFFTZeroFillPara.fftOutputSize = (int16_t)val;
		break;
	case FFTZERO_FFTWINDOWTYPE:
		RadarFFTZeroFillPara.fftWindowType = (int16_t)val;
		break;
	case FFTZERO_RAWVALUESCALE:
		RadarFFTZeroFillPara.rawValueScale = (int16_t)val;
		break;
	default:
		return -1;
	}

	return 0;
}
