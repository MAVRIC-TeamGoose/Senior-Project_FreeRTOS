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
#include "BatterySensor.h"

/*
 * A global value to be called in your transmit
 */
uint32_t i32VoltageValue;



//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define BATTERYTASKSTACKSIZE        128         // Stack size in words


// Semaphore for using the UART
extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pBatterySemaphore;

//*********************************************
// NOTE: Setting up the GPIO to enable the battery
//
//*******************************

uint32_t i32Period;  // Timer delay period
uint8_t ui8PinData = 1;


//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;   // number of clock cycles returned from system clock



//*****************************************************************************
//
// Read Data
//
//*****************************************************************************
uint32_t adc_battery_array [1] ; //Sequencer 3 has a FIFO of size 1

static void
BatteryTask(void *pvParameters)
{
	while(1){
		Battery_Voltage_Out();
		//UARTprintf("Value for Drew: %d", i32VoltageValue); // Debug value
		vTaskDelay(10000/portTICK_RATE_MS);  // Delay for 10 seconds
	}
}

/*
 * Battery Conversion
 */
int Battery_Level_Conversion(){
	//
	// Trigger ADC Conversion
	//
	ROM_ADCProcessorTrigger(ADC1_BASE, 3);

	//
	//Wait for ADC conversion to complete
	//
	while(!ROM_ADCIntStatus(ADC1_BASE, 3, false)) { //Wait for ADC to finish sampling
	}
	ROM_ADCSequenceDataGet(ADC1_BASE, 3, adc_battery_array); //Get data from Sequencer 3
	//
	// Update the interrupt status.
	//
	//ROM_IntMasterDisable();


	int V_actual;
	//12-bit ADC ranges from 0 to 4095
	//V_adc = adc_value[0]/(1666.363636/1.34286); corespondent match from the range
	//V_converted = V_adc*14.7/4.7*1000; converting to  mili volt for accuracy
	// Sumary of all steps into one final step
	V_actual =  adc_battery_array[0]*2.52046;

	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("Battery ADC = %d\n", adc_battery_array[0]); // ADC raw value
	UARTprintf("Battery Voltage = %d mV\n", V_actual); //Print out the first (and only) value
	xSemaphoreGive(g_pUARTSemaphore);

	//ROM_IntMasterEnable();
	return V_actual;
}

/*
 * Battery Configuration
 */
void Battery_Enable_Configure()
{
	//
	// Enable the GPIO port that is used for the on-board LEDs.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//
	// Enable the GPIO pins for the LEDs PN0
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // LED debugging
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);  // For battery enable signal

}



//*****************************************************************************
//
// The battery timer interrupt for checking voltage level
//
//*****************************************************************************
void Battery_Voltage_Out(void)
{

	// Read the current state of the GPIO pin and
	// write back the opposite state
	if(ROM_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0))
	{
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
	}
	else
	{
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ui8PinData); // For LED debug
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, (ui8PinData + 3)); // Turn on Pin 2
		//UARTprintf("About to sample the voltage level.......\n");
		xSemaphoreTake(g_pBatterySemaphore, portMAX_DELAY);
		i32VoltageValue = Battery_Level_Conversion();
		//UARTprintf("Done sampling!!\n\n");
		xSemaphoreGive(g_pBatterySemaphore);

		// Turn everything off to save power
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0); // Debug LEDs
		ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0); // For battery sampling enable

	}
}


/*
 * Battery ADC
 */
void Battery_ADC_Configure()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); //Enable ADC0/ADC1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enable GPIO E
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64); //Configure hardware oversampling to sample 64 times and average
	HWREG(ADC1_BASE + ADC_O_PC) = ADC_PC_SR_125K; //Set ADC speed to 125K

	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //Enable ADC on PE2
	ROM_ADCSequenceDisable(ADC1_BASE, 3); //Disable sequence before configuring it
	ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); //Use sequencer 3 to trigger at all times with a priority of 0 (highest)
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END); //Enable sampling using sequencer 3 on CH1 (PE2)
	ROM_ADCSequenceEnable(ADC1_BASE, 3); //Enable the sequencer
}

//*****************************************************************************
//
// Initializes the Battery GPIO and ADC modules.
//
//*****************************************************************************
uint32_t
BatteryTaskInit(void)
{

	Battery_Enable_Configure();

	Battery_ADC_Configure();

	//
	// Create the switch task.
	//
	if(xTaskCreate(BatteryTask, (signed portCHAR *)"Battery",
			BATTERYTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_BATTERY_TASK, NULL) != pdTRUE)
	{
		return(1);
	}
	//
	// Success.
	//
	return(0);
}
