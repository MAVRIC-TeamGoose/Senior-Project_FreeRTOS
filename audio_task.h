
/*
 * audio.h
 *
 *  Created on: Apr 29, 2015
 *      Author: root
 */
#include "arm_math.h"

#ifndef AUDIO_H_
#define AUDIO_H_

/*
 * Constants
 */

// Number of samples for FFT
#define NUM_SAMPLES 128

// Rate to sample input
#define SAMPLING_RATE 44100

// Number of frequencies to transmit to brain
#define NUM_FREQS 4

void configureADC();
void runFFT(float32_t*, float32_t*);
void TIMER1_Handler();
void ADC0_SampleHandler();
void ConfigureUART();
int freqIndex(int);
static void AudioTask(void *);
uint32_t AudioTaskInit(void);


#endif /* AUDIO_H_ */
