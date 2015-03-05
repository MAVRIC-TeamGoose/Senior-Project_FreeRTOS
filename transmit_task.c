/*
 * Author : Drew May
 * Date   : March 5th 2015
 * Version: 1.0
 * Summary: Tiva TM4C receives raw temperature from internal temperature module
 * 			and transmits two bytes worth of information through I2C.
 * 			In this example the master is a Raspberry Pi.
 *
 * Pi Code: Located in GitHub repository
 *
 */

/*
 * Pin Connections
 *
 * Ground is connected between both devices
 * PB2 on Tiva is connected to SCL (Pin 5) on the Raspberry Pi
 * PB3 on Tiva is connected to SDA (Pin 2) on the Raspberry Pi
 *
 * See http://pi.gadgetoid.com/pinout for a detailed
 * pinout of the Raspberry Pi. We are using Pins 2 and 5.
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

#include "transmit_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define TRANSMITTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
//
//*****************************************************************************
#define SLAVE_ADDRESS 0x04

//*****************************************************************************
//
// ADC Value.
//
//*****************************************************************************
uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

uint8_t adc_i2c[2];    //Two byte array to hold adc value

extern uint32_t g_ui32SysClock;

extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// Configure the I2C0 and its pins.
// Note: Changed function calls to ROM calls by adding MAP_ to statements.
//
//*****************************************************************************
void
ConfigureI2C0(void)
{
	//
	//enable I2C module 0
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	//
	//enable GPIO peripheral that contains I2C 0
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	//
	MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	//
	// Select the I2C function for these pins.
	//
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	//
	// Enable the I2C0 slave module.
	//
	MAP_I2CSlaveEnable(I2C0_BASE);
	//
	// Set the slave address to SLAVE_ADDRESS.
	//
	MAP_I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);
}

//Should remove from transmit task
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
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END); //Enable sampling using sequencer 3 on Temp sensor

	ROM_ADCSequenceEnable(ADC0_BASE, 3); //Enable the sequencer

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    MAP_ADCIntClear(ADC0_BASE, 3);
}

//Should remove from the transmit task
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

	// Disable context switching
	taskENTER_CRITICAL();
    //
    // Clear ADC interrupt
    //
    ROM_ADCIntClear(ADC0_BASE, 3);

	ROM_ADCSequenceDataGet(ADC0_BASE, 3, adc_value); //Get data from Sequencer 3

	//Convert adc into two byte array
	adc_i2c[0] = (*adc_value & 0xff00) >> 8;
	adc_i2c[1] = (*adc_value & 0x00ff);      //Lowest 8 bits
	// Enable context switching
	taskEXIT_CRITICAL();
}

//*****************************************************************************
//
// This task transmits data read in through the ADC through an I2C interface.
//
//*****************************************************************************
static void
TransmitTask(void *pvParameters)
{
    while(1) //Loop for all eternity
    {
        //
        // Wait until slave data is requested
        //
        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));

        //Note temp is recorded into a 32-bit value but is actually 12 bits.
        //Break temp into bytes and send lower two bytes
        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[0]); //Send back the temperature

        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));

        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[1]); //Send the lowest 8 bits
    }
}

//*****************************************************************************
//
// Initializes the I2C task.
//
//*****************************************************************************
uint32_t
TransmitTaskInit(void)
{
	//
	// Initialize I2C
	//
	ConfigureI2C0();
    //
    // Initialize ADC
    //
    ConfigureADC();
    //
    // Enable Temperature Timer.
    //
    ConfigureTempTimer();

    // Create the switch task.
    //
    if(xTaskCreate(TransmitTask, (signed portCHAR *)"Transmit",
    		TRANSMITTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
    		PRIORITY_TRANSMIT_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

	//
	// Success.
	//
	return(0);
}
