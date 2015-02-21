//
// test_task.c - A test task for FreeRTOS on the TM4C1294XL development board
//
// Prints a message over the UART at a regular interval
//
// Based on Texas Instruments - "led_task.c" (see below)
//
// Modified by Keith Lueneburg

//*****************************************************************************
//
// led_task.c - A simple flashing LED task.
//
// Copyright (c) 2012-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "test_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define TESTTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the test message queue.
//
//*****************************************************************************
/*
#define TEST_ITEM_SIZE           sizeof(uint8_t)
#define TEST_QUEUE_SIZE          5
 */

// Constants for ultrasonic ranger
#define TIMER4_TAMR_R           (*((volatile uint32_t *)0x40034004))
#define TIMER4_RIS_R            (*((volatile uint32_t *)0x4003401C))
#define TIMER4_TAR_R            (*((volatile uint32_t *)0x40034048))
#define TIMER4_TAV_R            (*((volatile uint32_t *)0x40034050))
#define TIMER4_ICR_R            (*((volatile uint32_t *)0x40034024))

#define SOUND_CM_PER_S 34326

#define TESTS_PER_SCENARIO 10

//*****************************************************************************
//
// Default test message delay value. Message will print once per interval
//
//*****************************************************************************
#define TEST_DELAY        100

//*****************************************************************************
//
// The queue that holds messages sent to the LED task.
//
//*****************************************************************************
// xQueueHandle g_pTESTQueue;

volatile uint32_t g_ui32PulseLengthTicks; // Length of ultrasonic echo pulse in system clock ticks

extern xSemaphoreHandle g_pUARTSemaphore;

extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Delay for 5 us.
//
//*****************************************************************************
void delayFiveMicroseconds(uint32_t g_ui32SysClock) {
	//
	// Delay for 5 us. The value of the number provided to SysCtlDelay
	// is the number of loops (3 assembly instructions each) to iterate through.
	// Interrupts are disabled temporarily to ensure the pulse length is 5us.
	//
	ROM_IntMasterDisable();

	//ROM_SysCtlDelay(g_ui32SysClock / 3 / 200000); // 5us delay
	ROM_SysCtlDelay(g_ui32SysClock / 3 /   100000); //100 us

	ROM_IntMasterEnable();
}

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void
TESTTask(void *pvParameters)
{
	portTickType ui32WakeTime;
	uint32_t ui32TestDelay;
	// uint8_t i8Message;

	//
	// Initialize the Test Delay to default value.
	//
	ui32TestDelay = TEST_DELAY;

	//
	// Get the current tick count.
	//
	ui32WakeTime = xTaskGetTickCount();

	//
	// Counts number of tests for each setup
	//
//	uint8_t testCount = 10;

	//
	// Loop forever.
	//
	while(1)
	{
		//
		// Wait for the required amount of time.
		//
		vTaskDelayUntil(&ui32WakeTime, ui32TestDelay / portTICK_RATE_MS);
/*

		if (testCount > TESTS_PER_SCENARIO) {
			testCount = 0;
			xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
			UARTprintf("\nEnter:\nRANGE | ANGLE (L/R) | SHAPE | HEIGHT\n");

			char c = 0;
			while (c!='\n' && c!='\r'){
				c = UARTgetc();
			}

			xSemaphoreGive(g_pUARTSemaphore);
		}
		testCount++;
*/

		uint32_t ui32PulseStartTime = 0; // Timer value at echo pulse rising edge
		uint32_t ui32PulseStopTime = 0; // Timer value at echo pulse falling edge

		// // // Send a trigger pulse \\ \\ \\

		// Disable context switching
		taskENTER_CRITICAL();

		//
		// Turn on pulse.
		//
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);

		//
		// Delay for 5 us.
		//
		delayFiveMicroseconds(g_ui32SysClock);

		//
		// Turn off pulse.
		//
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);


		// // // Measure pulse length \\ \\ \\

		//
		// Clear Timer A capture flag
		//
		TIMER4_ICR_R |= (1 << 2);

		//
		// Start Timer4 A
		//
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);

		//
		// Wait for first capture event
		//
		while((TIMER4_RIS_R & (0x1 << 2)) == 0){};

		//
		// After first event, save timer A's captured value.
		//
		ui32PulseStartTime = TIMER4_TAR_R;

		//
		// Clear Timer A capture flag
		//
		TIMER4_ICR_R |= (1 << 2);

		//
		// Wait for second capture event
		//
		while((TIMER4_RIS_R & (0x1 << 2)) == 0){};

		//
		// After second event, save timer A's captured value.
		//
		ui32PulseStopTime = TIMER4_TAR_R;

		//
		// Calculate length of the pulse by subtracting the two timer values.
		// Note that even if stop time is less than start time, due to the
		// timer overflowing and starting over at 0, the 24 LSBs of our result are
		// still valid
		//
		g_ui32PulseLengthTicks = (ui32PulseStopTime - ui32PulseStartTime) & 0x0FFFFFF;

		// Re-enable context switching
		taskEXIT_CRITICAL();

		//
		// Print the start time, stop time, and pulse length
		//
		//	UARTprintf("Pulse length: %d - %d = %d\n", ui32PulseStopTime,
		//		ui32PulseStartTime, ui32PulseLengthTicks);

		//
		// Print the distance
		//
		//UARTprintf("Distance: %d\n", ui32DistanceCM);

		//
		// Stop Timer4 A.
		//
		ROM_TimerDisable(TIMER4_BASE, TIMER_A);

		//Calculates Proximity reading
		uint32_t ui32DistanceCM; // Sensor output converted to centimeters
		ui32DistanceCM = 0;

		// pulse length / system clock = pulse length in seconds
		// pulse length in seconds / (1 / sound speed cm per s) = total wave flight distance
		// total distance / 2 = distance from sensor
		//
		ui32DistanceCM = g_ui32PulseLengthTicks / 2 / (g_ui32SysClock / SOUND_CM_PER_S);

		//
		// Guard UART from concurrent access.
		// Print test results
		//
		xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
		UARTprintf("\nProx= %d cm\n", ui32DistanceCM);
		xSemaphoreGive(g_pUARTSemaphore);
	}
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t
TestTaskInit(void)
{
	//*****************************************************************************
	//
	// Configure the timer and its pins for measuring the length of
	// ultrasonic sensor echo pulse.
	//
	//*****************************************************************************

	//
	// Enable GPIO M
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	//
	// Enable Timer 4
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);


	ROM_SysCtlDelay(2); // Wait before doing anything with GPIO registers

	//
	// Enable the GPIO pin for the trigger pulse (M5).
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);

	//
	// Enable GPIO pin for timer event capture (B0).
	//
	ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

	//
	// Configure PB0 as Timer 4 CCP0
	//
	ROM_GPIOPinConfigure(GPIO_PB0_T4CCP0);

	//
	// Configure timer 4A as a 16-bit event capture up-counter
	//
	ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

	//
	// Set prescaler to 255. This essentially makes
	// the 16-bit timer a 24-bit timer.
	//
	ROM_TimerPrescaleSet(TIMER4_BASE, TIMER_A, 0xFF);

	//
	// The timer should capture events on both rising and falling edges
	//
	ROM_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

	//
	    // Create the switch task.
	    //
	    if(xTaskCreate(TESTTask, (signed portCHAR *)"Test",
	    		TESTTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
	    		PRIORITY_TEST_TASK, NULL) != pdTRUE)
	    {
	        return(1);
	    }

	//
	// Success.
	//
	return(0);
}
