#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <ir_processing.h>

#include "selector.h"


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micFront_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micFront_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 
#define SELECT_LIMIT 8

// frequency = position*15.625 (resolution - explanation in TP5 page 6)

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1) 	//
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void direction_detection(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int select=get_selector();

	int16_t max_norm_index = -1; 

	if(select<SELECT_LIMIT){

		//search for the highest peak
		for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(data[i] > max_norm){
				max_norm = data[i];
				max_norm_index = i;
			}
		}

		//go forward
		if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
			ok_to_move(FORWARDS);
		}
		//turn left
		else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
			ok_to_move(LEFT);
		}
		//turn right
		else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
			ok_to_move(RIGHT);
		}
		//go backward
		else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
			ok_to_move(BACK);
		}
		//no sound
		else{
			ok_to_move(STOP);
		}
	}
	else{
		 e_puck_follow();
	}

}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//i=+4 car data = [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
			// sinon il faudrait changer le driver (d'après Iacopo)

		//construct an array of complex numbers. Put 0 to the imaginary part
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];
		nb_samples++;
		micFront_cmplx_input[nb_samples] = 0;
		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){	//if buffer is full --> FFT
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/

		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		nb_samples = 0;

		direction_detection(micFront_output);
	}
}
