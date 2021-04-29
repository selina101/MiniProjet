#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define FORWARDS 0
#define LEFT 1
#define RIGHT 2
#define BACK 3
#define STOP 4
#define STUCK 5


typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

void processAudioData(int16_t *data, uint16_t num_samples);
void direction_detection(float* data);

#endif /* AUDIO_PROCESSING_H */
