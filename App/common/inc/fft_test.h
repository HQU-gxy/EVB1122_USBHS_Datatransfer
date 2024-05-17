/**
  ******************************************************************************
  * @file           : fft_test.h
  * @author         : iclm team
  * @brief          : fft test header file
  ******************************************************************************
  */
#ifndef __FFT_TEST_H__
#define __FFT_TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#define FFT_POINT           (64)
#define SAMPLING_FREQ       (512) //Hz
#define SIN_FREQ            (32) //Hz
#define MAX_INT16           (32768)
#define SIN_COEF            (MAX_INT16 * SIN_FREQ / SAMPLING_FREQ)

#define MAX_RANGE           (256)
#define MAX_CHIRP           FFT_POINT

void FFT_Verify(void);
void FFT_Verify_R_q15(void);
void FFT_Test(void);

#ifdef __cplusplus
}
#endif

#endif


