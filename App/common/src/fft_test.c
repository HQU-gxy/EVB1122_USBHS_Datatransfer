/**
  ******************************************************************************
  * @file           : fft_test.c
  * @author         : iclm team
  * @brief          : test fft function
  ******************************************************************************
  */
#include <stdio.h>
#include "global_conf.h"
#include "platform.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "fft_test.h"
#include "dsp.h"

q15_t signal[FFT_POINT * 2] = {0};
q15_t fftAbs[FFT_POINT] = {0};
q15_t rangeFFT[MAX_RANGE][MAX_CHIRP * 2] = {0};

void FFT_Verify(void)
{
    uint16_t i = 0;
    uint16_t j = 0;
    arm_cfft_radix4_instance_q15 S;

    arm_cfft_radix4_init_q15(&S, FFT_POINT, 0, 1);
    
    for(i = 0; i < FFT_POINT; i++)
    {
        j = i % (SAMPLING_FREQ / SIN_FREQ);
        signal[i * 2] = arm_sin_q15(SIN_COEF * j); 
        printf("%d\r\n", signal[i * 2]);
    }
    
    printf("**************************************************\r\n");
    
    arm_cfft_radix4_q15(&S, signal);               //shrink FFT_POINT
    arm_cmplx_mag_q15(signal, fftAbs, FFT_POINT);  //shrink 2
   
    for(i = 0; i < FFT_POINT; i++)
    {
        fftAbs[i] = fftAbs[i] << 2;  //fftAbs[i] = fftAbs[i] * FFT_POINT * 2 / (FFT_POINT/2);
        printf("%d\r\n", fftAbs[i]);
    }
}

void FFT_Verify_R_q15(void)
{
    uint16_t i = 0;
    uint16_t j = 0;
    arm_rfft_instance_q15 S;

    arm_rfft_init_q15(&S, FFT_POINT, 0, 1);
    
    for(i = 0; i < FFT_POINT; i++)
    {
        j = i % (SAMPLING_FREQ / SIN_FREQ);
        signal[i] = arm_sin_q15(SIN_COEF * j); 
        printf("%d\r\n", signal[i]);
    }
    
    printf("**************************************************\r\n");
    
    arm_rfft_q15(&S, signal, &signal[FFT_POINT]);               //shrink FFT_POINT
    arm_cmplx_mag_q15(&signal[FFT_POINT], fftAbs, FFT_POINT/2);  //shrink 2
   
    for(i = 0; i < FFT_POINT/2; i++)
    {
        fftAbs[i] = fftAbs[i] << 2;  //fftAbs[i] = fftAbs[i] * FFT_POINT * 2 / (FFT_POINT/2);
        printf("%d\r\n", fftAbs[i]);
    }
}

static void SignalGenerate(void)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;

    for(i = 0;  i < MAX_RANGE; i++)
    {
        for(j = 0; j < MAX_CHIRP; j++)
        {
            k = j % (SAMPLING_FREQ / SIN_FREQ);
            rangeFFT[i][j * 2] = arm_sin_q15(SIN_COEF * k); 
        }
    }
}

static void DopplerFFTCalc(q15_t (*rfft)[MAX_CHIRP * 2])
{
    uint16_t i = 0;
    arm_cfft_radix4_instance_q15 S;

    arm_cfft_radix4_init_q15(&S, MAX_CHIRP, 0, 1);
    
    for(i = 0; i < MAX_RANGE; i++)
    {
        arm_cfft_radix4_q15(&S, rfft[i]);               
    }
}

void FFT_Test(void)
{
    uint32_t tickstart = 0;
    uint32_t tickend = 0;

    SignalGenerate();
    
    tickstart = HAL_GetTick();
    DopplerFFTCalc(rangeFFT);
    tickend = HAL_GetTick();

    printf("doppler fft calc time: %d\r\n", tickend - tickstart);
}

