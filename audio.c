/*
 * audio.c
 *
 *  Created on: Apr 28, 2015
 *      Authors: MAVRIC Team B
 *               Thinh Le
 *               Keith Lueneburg
 *               Drew May
 *               Brandon Thomas
 */

#include <stdbool.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "priorities.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "audio.h"
/*
 * Constants
 */

// Number of samples for FFT
#define NUM_SAMPLES 128

// Rate to sample input
#define SAMPLING_RATE 44100

// Number of frequencies to use for testing
#define NUM_FREQS 4

/*
 *  Global variables
 */
// Used to guard UART
extern xSemaphoreHandle g_pUARTSemaphore;

// Frequencies to check
uint32_t freqs[NUM_FREQS] = { 1000, 2000, 3000, 4000 };


// Pointer to where the next sample should
// go in the input buffer.
uint32_t inputIndex;

// Input data buffer for FFT
float32_t inputDataL[NUM_SAMPLES];
float32_t inputDataR[NUM_SAMPLES];

// Intermediate RFFT complex output goes here in paired format
// We need to get the complex magnitude to analyze
// each frequency bucket
// We can use this for multiple channels
static float32_t rfftOutput[NUM_SAMPLES];


// "Final" output goes here; the magnitude
// of the raw complex output. We can look at each index
// of this array to see the magnitude of that frequency range.
// Each bucket will cover
//        B = (SAMPLING_RATE / NUM_SAMPLES) Hz,
// where index 0 represents
//        0 <--> B Hz,
// and the last index, ((NUM_SAMPLES / 2) - 1), represents
//        (SAMPLING_RATE / 2) - B) <--> (SAMPLING_RATE / 2) Hz
//
// For example, with a 44100 Hz sampling rate, and a 512 point FFT,
// we have a bucket width of 44100 / 512 ~= 86.1 Hz.
// Bucket 0 will represent 0 <--> 86.1 Hz,
// and Bucket 255 will represent 21964 <--> 22050 Hz.
static float32_t outputDataL[NUM_SAMPLES/2];
static float32_t outputDataR[NUM_SAMPLES/2];

// Used to retrieve data from ADC sequencer
uint32_t adc_value[2];

// FFT instance
arm_rfft_fast_instance_f32 fft;

/* ------------------------------------------------------------------
 * Global variable for system clock
 * ------------------------------------------------------------------- */
extern uint32_t g_ui32SysClock;

/* ------------------------------------------------------------------
 * Flags for RFFT initialization
 * ------------------------------------------------------------------- */

// Do the forward transform (0), or inverse (1)
uint32_t ifftFlag = 0;

// Used internally by CMSIS Math & DSP libraries
uint32_t doBitReverse = 1;

//*****************************************************************************
//
// The stack size for the task.
//
//*****************************************************************************
#define AUDIOTASKSTACKSIZE        128         // Stack size in words

// Audio Task
static void AudioTask(void *pvParameters)
{
	while(1) {


		vTaskDelay(1000);
		// Start HW timer for sampling (ADC interrupt will
		// trigger FFT when buffer full)

		// Enable interrupts for sample conversion complete
		MAP_ADCIntEnable(ADC0_BASE, 2);

		// Set priority for ADC0 SS2 interrupt
		//MAP_IntPrioritySet(PRIORITY_ADC0_SS2_INT);

		// Enable NVIC interrupt for ADC0 SS3
		MAP_IntEnable(INT_ADC0SS2);

		// Start timer
		MAP_TimerEnable(TIMER2_BASE, TIMER_A);

	}

}

// Returns the magnitude output buffer index for a
// given frequency in Hz
int freqIndex(int freq)
{
	// This is based on buckets frequency range being
	// CENTERED on n*bucketSize.
	float bucketSize = SAMPLING_RATE / ((float) NUM_SAMPLES);

	return (int)( (freq + (bucketSize/2) ) / bucketSize);
}

void runFFT(float32_t* inputArray, float32_t* magOutput)
{
	//int32_t startTime, stopTime, totalTime; // Variables for timing testing
	//	float32_t maxValue;

	// Setup testing timer
	// TODO: This is just used for timing testing
	//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	//	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock); // 1 second

	//	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	//	startTime = TIMER0_TAR_R;

	// Run the FFT

	/* Process the real data through the RFFT module */
	arm_rfft_fast_f32(&fft, inputArray, rfftOutput, ifftFlag);

	/* Process the data through the Complex Magnitude Module for
	  calculating the magnitude at each bin */
	arm_cmplx_mag_f32(rfftOutput, magOutput, NUM_SAMPLES / 2);

	/* Calculates maxValue and returns corresponding BIN value */
	//arm_max_f32(magOutput, NUM_SAMPLES / 2, &maxValue, &peakBucket);

	//	stopTime = TIMER0_TAR_R;
	//	totalTime = startTime - stopTime;
	//	int totalTimeUs = totalTime / 120;

	// Convert peak frequency bucket number to frequency
	//	int peakFrequency = peakBucket * SAMPLING_RATE / NUM_SAMPLES;

	/*
	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	int i;
	for (i = 0; i < NUM_FREQS; i++)
	{
		// Print each frequency (closest bucket) and its magnitude
		// Note: Print output is cast to integer due to restrictions in UARTprintf function
		UARTprintf("%d Hz: %d\n", freqs[i], (int) magOutput[freqIndex(freqs[i])]);

	}
	UARTprintf("\n\n");
	xSemaphoreGive(g_pUARTSemaphore);
	*/

}

void ADC0_SampleHandler()
{
	// Turn on debuggin signal
	MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);

	// Clear interrupt signal in ADC
	MAP_ADCIntClear(ADC0_BASE, 2);

	// Get data and store in array
	MAP_ADCSequenceDataGet(ADC0_BASE, 2, adc_value);

	// Cast as floating point and store in input buffer
	inputDataL[inputIndex] = (float) adc_value[0];
	inputDataR[inputIndex] = (float) adc_value[1];
	inputIndex++;

	// If input buffer is full, run the FFT
	if (inputIndex > NUM_SAMPLES - 1)
	{
		// Reset input buffer index
		inputIndex = 0;

		// Disable interrupt and timer
		MAP_ADCIntDisable(ADC0_BASE, 3);
		MAP_TimerDisable(TIMER2_BASE, TIMER_A);

		// Run the transform
		runFFT(inputDataL, outputDataL);
		runFFT(inputDataR, outputDataR);


		// TODO: maybe give a semaphore to signal done??
	}
}

// Configure ADC peripheral and FFT
uint32_t AudioTaskInit(void)
{

	//UARTprintf("\033[2JFFT Test\n");

	// Enable FPU
	ROM_FPUEnable();

	// Initialize input buffer index for FFT data
	inputIndex = 0;

	// Create a RFFT instance
	arm_rfft_fast_init_f32(&fft,NUM_SAMPLES);

	// Set up ADC sampling and interrupt
	configureADC();

	//
	// Create the audio task.
	//
	if(xTaskCreate(AudioTask, (signed portCHAR *)"Audio",
			AUDIOTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_AUDIO_TASK, NULL) != pdTRUE)
	{
		return(1);
	}
	//
	// Success.
	//
	return(0);
}

// Set up ADC for audio sampling
void configureADC()
{
	// Enable peripherals
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

	// Wait for peripherals to activate
	MAP_SysCtlDelay(2);

	// Disable sequencer 0 before configuring
	MAP_ADCSequenceDisable(ADC0_BASE, 2);

	// Configure GPIO Pin for ADC
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

	// TODO: Used for measuring timing, can be removed later
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
	MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);

	/*
	 * Configure timer for ADC triggering
	 */
	MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

	// Load timer for periodic sampling of ADC
	MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock/SAMPLING_RATE);

	// Enable ADC triggering
	MAP_TimerControlTrigger(TIMER2_BASE, TIMER_A, true);

	// Trigger ADC on timer A timeout
	MAP_TimerADCEventSet(TIMER2_BASE, TIMER_ADC_TIMEOUT_A);


	/*
	 * Configure ADC
	 */

	// Clear the interrupt raw status bit (should be done early
	// on, because it can take several cycles to clear.)
	MAP_ADCIntClear(ADC0_BASE, 2);

	// ADC0, Seq 0, Timer triggered, Priority 0
	MAP_ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0);

	// Configure sequencer step 0 for proper input channel, interrupt enable, and end of sequence.
	MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH4);
	MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

	// Enable sequencer
	MAP_ADCSequenceEnable(ADC0_BASE, 2);
}
