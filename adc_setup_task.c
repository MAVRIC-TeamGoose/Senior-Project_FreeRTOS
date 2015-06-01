/*
 * Author : Drew May
 * Date   : March 5th 2015
 * Version: 1.0
 * Summary: Sets up ADC functionality for MAVRIC
 */

/*
 * Pin Connections
 * N/A Currently
 */


#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_adc.h" //Had to change one line here on line 416 from 3 to 1
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h" //Header file for using i2c interface
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "adc_setup_task.h"

//*****************************************************************************
//
// ADC Value.
//
//*****************************************************************************
uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

uint8_t adc_i2c[2];    //Two byte array to hold adc value (mark as extern value and add mutex to it)

extern xSemaphoreHandle g_pTemperatureSemaphore;

extern xSemaphoreHandle g_pUARTSemaphore;

extern uint32_t g_ui32SysClock;

void
ConfigureADC()
{
    //
    // Setup ADC Using ROM functions
    //
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Enable ADC0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enable GPIO E
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64); //Configure hardware oversampling to sample 64 times and average
	HWREG(ADC0_BASE + ADC_O_PC) = ADC_PC_SR_125K; //Set ADC speed to 125K

	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //Enable ADC on PE3
	ROM_ADCSequenceDisable(ADC0_BASE, 3); //Disable sequence before configuring it
	ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); //Use sequencer 3 to trigger at all times with a priority of 0 (highest)
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END); //Enable sampling using sequencer 3 on Temp sensor
	ROM_ADCSequenceEnable(ADC0_BASE, 3); //Enable the sequencer

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    MAP_ADCIntClear(ADC0_BASE, 3);
}

void
ConfigureTempTimer()
{
    //
    // Enable ADC Timer
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //Temperature

    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock);  //Temp      1000ms

    ROM_IntEnable(INT_TIMER1A); //Temp

    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //Temp

    ROM_TimerEnable(TIMER1_BASE, TIMER_A); //Temp
}

//*****************************************************************************
//
// The interrupt handler for the ADC. (temperature)
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
	// Disable context switching
	taskENTER_CRITICAL();
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Trigger ADC Conversion
    //
    ROM_ADCProcessorTrigger(ADC0_BASE, 3);

    //
    //Wait for ADC conversion to complete
    //
    while(!ROM_ADCIntStatus(ADC0_BASE, 3, false))
    { //Wait for ADC to finish sampling
    }
    xSemaphoreTakeFromISR(g_pTemperatureSemaphore, &xHigherPriorityTaskWoken);

    //
    // Clear ADC interrupt
    //
    ROM_ADCIntClear(ADC0_BASE, 3);

	ROM_ADCSequenceDataGet(ADC0_BASE, 3, adc_value); //Get data from Sequencer 3

	xSemaphoreTakeFromISR(g_pUARTSemaphore, &xHigherPriorityTaskWoken);
	UARTprintf("Temp:%d\n", adc_value[0]);
	xSemaphoreGiveFromISR(g_pUARTSemaphore, &xHigherPriorityTaskWoken);

	xSemaphoreGiveFromISR(g_pTemperatureSemaphore, &xHigherPriorityTaskWoken);
	// Enable context switching
	taskEXIT_CRITICAL();
}

//*****************************************************************************
//
// Initializes the ADC modules and timers.
//
//*****************************************************************************
uint32_t
ADCInit(void)
{

    //
    // Initialize ADC
    //
    ConfigureADC();
    //
    // Enable Temperature Timer.
    //
    ConfigureTempTimer();

	//
	// Success.
	//
	return(0);
}
