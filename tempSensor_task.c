/*
 * Battery.c
 *
 * Thinh
 */


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/fpu.h"
#include "priorities.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tempSensor_task.h"

//Math Constants
float DENOMINATOR = 0.000001790830963;

int B = 3950;

double VIN = 3.3;


/*
 * A global value to be called in your transmit
 */
uint32_t i32TempValue;


//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define TEMPERATURETASKSTACKSIZE        128         // Stack size in words


// Semaphore for using the UART
extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pTemperatureSemaphore;


//*********************************************
// NOTE: Setting up the GPIO to enable the battery
//
//*******************************

uint32_t i32Temp_Period;  // Timer delay period
uint8_t ui8Temp_PinData = 1;

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32Temp_SysClock;   // number of clock cycles returned from system clock




//*****************************************************************************
//
// Read Data
//
//*****************************************************************************
uint32_t Temp_ADCvalue [1] ; //Sequencer 3 has a FIFO of size 1



static void
temperatureTask(void *pvParameters)
{
	while(1){
		temperature_Out();
		UARTprintf("Value for Drew: %d", i32TempValue); // Debug value
		vTaskDelay(10000/portTICK_RATE_MS);  // Delay for 10 seconds
	}
}


/*
 * Battery Conversion
 */
int temperature_Level_Conversion(){
	//
		// Trigger ADC Conversion
		//
		ROM_ADCProcessorTrigger(ADC1_BASE, 3);

		//
		//Wait for ADC conversion to complete
		//
		while(!ROM_ADCIntStatus(ADC1_BASE, 3, false)) { //Wait for ADC to finish sampling
		}
		ROM_ADCSequenceDataGet(ADC1_BASE, 3, Temp_ADCvalue); //Get data from Sequencer 3
		//
		// Update the interrupt status.
		//
		//ROM_IntMasterDisable();


		int tempReturn;
		int rawTemp = Temp_ADCvalue[0]; //Save current temp
		float tempK = (float) B / log((((float) VIN/(rawTemp*3.3 / 4096)) - 1) / DENOMINATOR); //Temp in Kelvin

		float tempF = (tempK - 273.15) * 1.8000 + 32.00;
		float tempC = (tempK - 273.15);
		tempReturn = (int) tempF;

		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("ADC Temp Value = %d\n", Temp_ADCvalue[0]); // ADC raw value
		UARTprintf("Fahrenheit Temp = %d\n", (int) tempF); //Print out temp in Fahrenheit
		UARTprintf("Celcius Temp = %d\n", (int) tempC); //Print out temp in Celcius
		xSemaphoreGive(g_pUARTSemaphore);
		//ROM_IntMasterEnable();

		return tempReturn;

}

/*
 * Battery Configuration
 */
void temperature_GPIO_Configure()
{
	//
	// Enable the GPIO port that is used for the on-board LEDs.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5); // for ADC
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6); // for RTC

	//
	// Enable the GPIO pins for the LEDs PN0
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // LED debugging
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);  // For battery enable signal

	// Timer configuration
	ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC); // for ADC
	ROM_TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC); // for RTC

	// Delay to toggle the pin
	i32Temp_Period = g_ui32Temp_SysClock;  // Toggle GPIO 1Hz after every 35 seconds  // Remember to *35
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, i32Temp_Period -1);  // interrup fires at 0 count
	ROM_TimerLoadSet(TIMER6_BASE, TIMER_A, g_ui32Temp_SysClock / 10); //RTC 100ms

	// Battery interrupt
	ROM_IntEnable(INT_TIMER5A); // For temp
	ROM_IntEnable(INT_TIMER6A); //RTC
	ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT); //For temp
	ROM_TimerIntEnable(TIMER6_BASE, TIMER_TIMA_TIMEOUT); //RTC
	ROM_IntMasterEnable();

	// Star the battery timer
	ROM_TimerEnable(TIMER5_BASE, TIMER_A); //For temp
	ROM_TimerEnable(TIMER6_BASE, TIMER_A); //RTC


}



//*****************************************************************************
//
// The battery timer interrupt for checking voltage level
//
//*****************************************************************************
void temperature_Out(void)
{

	// Read the current state of the GPIO pin and
	// write back the opposite state
	if(ROM_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0))
	{
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
	}
	else
	{
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ui8Temp_PinData); // For LED debug
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, (ui8Temp_PinData + 3)); // Turn on Pin 2

		//UARTprintf("About to sample the temperature level.......\n");
		xSemaphoreTake(g_pTemperatureSemaphore, portMAX_DELAY);
		i32TempValue = temperature_Level_Conversion();

		//UARTprintf("Done sampling!!\n\n");
		xSemaphoreGive(g_pTemperatureSemaphore);

		// Turn everything off to save power
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0); // Debug LEDs
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0); // For battery sampling enable

	}
}


/*
 * Temp ADC
 */
void temperature_ADC_Configure()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); //Enable ADC0/ADC1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enable GPIO E
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64); //Configure hardware oversampling to sample 64 times and average
	HWREG(ADC1_BASE + ADC_O_PC) = ADC_PC_SR_125K; //Set ADC speed to 125K

	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); //Enable ADC on PE5
	ROM_ADCSequenceDisable(ADC1_BASE, 3); //Disable sequence before configuring it
	ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); //Use sequencer 3 to trigger at all times with a priority of 0 (highest)
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END); //Enable sampling using sequencer 3 on CH8
	ROM_ADCSequenceEnable(ADC1_BASE, 3); //Enable the sequencer
}


////*****************************************************************************
////
//// The interrupt handler for the ADC. (temperature)
////
////*****************************************************************************
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

	ROM_ADCSequenceDataGet(ADC0_BASE, 3, Temp_ADCvalue); //Get data from Sequencer 3

	xSemaphoreTakeFromISR(g_pUARTSemaphore, &xHigherPriorityTaskWoken);
	UARTprintf("Temp:%d\n", Temp_ADCvalue[0]);
	xSemaphoreGiveFromISR(g_pUARTSemaphore, &xHigherPriorityTaskWoken);

	xSemaphoreGiveFromISR(g_pTemperatureSemaphore, &xHigherPriorityTaskWoken);
	// Enable context switching
	taskEXIT_CRITICAL();
}

//*****************************************************************************
//
// Initializes the temperature task GPIO and ADC modules.
//
//*****************************************************************************
uint32_t
temperature_TaskInit(void)
{

	temperature_GPIO_Configure();

	temperature_ADC_Configure();

	//
	// Create the switch task.
	//
	if(xTaskCreate(temperatureTask, (signed portCHAR *)"Temperature",
			TEMPERATURETASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_TEMPERATURE_TASK, NULL) != pdTRUE)
	{
		return(1);
	}
	//
	// Success.
	//
	return(0);
}
