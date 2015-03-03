/*
 * transmit_task.c
 *
 *  Created on: Feb 22, 2015
 *      Author: Drew
 *
 *  Todo Need to add transmit/receive functions
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h" //Header file for using i2c interface
#include "transmit_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"

//*****************************************************************************
//
// This task transmits data read in through the ADC through an I2C interface.
// ADDED BY DREW!!!!!!!!!!!!
//
//*****************************************************************************
/*static void
TESTTask(void *pvParameters)
{
	//portTickType ui32WakeTime;
	//uint32_t ui32TestDelay;

	//
	// Initialize the Test Delay to default value.
	//
	//ui32TestDelay = TEST_DELAY;

	//
	// Get the current tick count.
	//
	//ui32WakeTime = xTaskGetTickCount();

	//
	// Loop forever.
	//
	while(1)
	{
		//
		// Wait for the required amount of time.
		//
		//vTaskDelayUntil(&ui32WakeTime, ui32TestDelay / portTICK_RATE_MS);


		// Disable context switching
		taskENTER_CRITICAL();


		// Re-enable context switching
		taskEXIT_CRITICAL();
	}
}*/




//*****************************************************************************
//
// Initializes the I2C task.
// ADDED BY DREW!!!!!!!!!!!!
//
//*****************************************************************************
uint32_t
TestTaskInit(void)
{
	ConfigureI2C0();
	//
	// Success.
	//
	return(0);
}

//*****************************************************************************
//
// Configure the I2C0 and its pins.  This must be called before I2C0SlaveTX().
// ADDED BY DREW!!!!!!!!!!!!
//
//*****************************************************************************

void
ConfigureI2C0(void)
{
	//
	//enable I2C module 0
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	//
	//enable GPIO peripheral that contains I2C 0
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	//
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	//
	// Select the I2C function for these pins.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	//
	// Enable and initialize the I2C0 slave module. Set the slave address to
	// 0x04 to match I2C master program running on Raspberry Pi
	//
    I2CSlaveInit(I2C0_BASE, 0x04); //Slave address of 0x04
}



