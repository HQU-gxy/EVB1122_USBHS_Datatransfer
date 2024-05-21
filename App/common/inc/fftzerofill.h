/**
 ******************************************************************************
 * @file           : system.h
 * @author         : iclm team
 * @brief          : system header file
 ******************************************************************************
 */
#ifndef __FFTZEROFILL_H__
#define __FFTZEROFILL_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// #define USE_FFT_Q31

#define SAMP_MAX_FIXED16 32767
#define SAMP_MIN_FIXED16 -SAMP_MAX_FIXED16
#ifndef LIMITTOBOUND
#define LIMITTOBOUND(x, l, h)   \
	{                           \
		if ((x) < (l)) {        \
			(x) = (l);          \
		} else if ((x) > (h)) { \
			(x) = (h);          \
		}                       \
	}
#endif
typedef enum {
	FFTZERO_ENABLE = 0,
	FFTZERO_RAWSAMPLESIZE,
	FFTZERO_FFTCALCSIZE,
	FFTZERO_FFTOUTPUTOFFSET,
	FFTZERO_FFTOUTPUTSIZE,
	FFTZERO_FFTWINDOWTYPE,
	FFTZERO_RAWVALUESCALE,
} fftZeroParaEnum;

typedef enum {
	FFT_WINDOW_NOWND = 0,
	FFT_WINDOW_HAMMING,
	FFT_WINDOW_HANNING,
	FFT_WINDOW_MAX,
} fftWindowTypeEnum;

typedef enum {
	FFT_CALCSIZE_16 = 0,
	FFT_CALCSIZE_32,
	FFT_CALCSIZE_64,
	FFT_CALCSIZE_128,
	FFT_CALCSIZE_256,
	FFT_CALCSIZE_512,
	FFT_CALCSIZE_1024,
	FFT_CALCSIZE_MAX
} fftCalSizeEnum;

typedef struct RADAR_FFT_ZERO_FILL {
	uint8_t enableZeroFill;
	uint16_t rawSampleSize;
	fftCalSizeEnum fftCalcMap;
	uint16_t fftCalcSize;
	uint16_t fftOffset;
	uint16_t fftOutputSize;
	uint8_t fftWindowType;
	uint16_t rawValueScale;
} RADAR_FFT_ZERO_FILL_T;

uint8_t FFTZero_GetEnable(void);
uint16_t FFTZero_GetRawSampleSize(void);
fftCalSizeEnum FFTZero_GetCalcMap(void);
uint16_t FFTZero_GetCalcSize(void);
uint16_t FFTZero_GetOffset(void);
uint16_t FFTZero_GetOutputSize(void);
void FFTZero_ResizeOffsetAndOutput(void);
uint8_t FFTZero_GetWindowType(void);
uint16_t FFTZero_GetRawValueScale(void);
int8_t fftzerofill_ParaUpdate(uint16_t type, int32_t val);
#ifdef __cplusplus
}
#endif

#endif
