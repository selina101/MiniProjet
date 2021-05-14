#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <fft.h>

#include <arm_math.h>
#include <arm_const_structs.h>

/* Define complex multiplication and its conjugate */
#define  rmul(x,y)      (x.real * y.real - x.imag * y.imag)
#define  imul(x,y)      (x.imag * y.real + x.real * y.imag)
#define rcmul(x,y)      (x.real * y.real + x.imag * y.imag)
#define icmul(x,y)      (x.imag * y.real - x.real * y.imag)


/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	
}

