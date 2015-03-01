//
// sonar_task.c - A sonar task for FreeRTOS on the TM4C1294XL development board
//
// Operates the MAVRIC sonar/sonar array
//
// Based on Texas Instruments - "led_task.c" (see below)
//
// Modified by Team Goose from original Texas Instruments code (see below)

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
#define SONARTASKSTACKSIZE        128         // Stack size in words

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

// 2,000,000 clock ticks at 120 MHz is around 16.7 ms
// which is near the expected pulse length for
// the max range of 3m (286 cm)
#define RESPONSE_WAIT_TICKS 2000000

#define TESTS_PER_SCENARIO 10

//*****************************************************************************
//
// Default test message delay value (in ms). Message will print once per interval
//
//*****************************************************************************
#define TEST_DELAY        1000

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
// Delay for 10 us.
//
//*****************************************************************************
void delayTenMicroseconds(uint32_t g_ui32SysClock) {
	//
	// Delay for 10 us. The value of the number provided to SysCtlDelay
	// is the number of loops (3 assembly instructions each) to iterate through.
	// Interrupts are disabled temporarily to ensure the pulse length is 5us.
	//
	MAP_IntMasterDisable();

	//MAP_SysCtlDelay(g_ui32SysClock / 3 / 200000); // 5us delay
	MAP_SysCtlDelay(g_ui32SysClock / 3 /   100000); // 10 us

	MAP_IntMasterEnable();
}

//*****************************************************************************
//
// Select which of the sonar devices to make active.
//
//*****************************************************************************
void selectSonar(uint8_t select)
{
	// disable muxes
	// digital mux ~enable = 1
	MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
	// power mux enable = 0
	MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);

	// mux S0 (B5) = select & 0x1
	// mux S1 (B6) = select & 0x2
	// mux S2 (B7) = select & 0x4
	MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, (select << 5));

	// reenable muxes
	// digital mux ~enable = 0
	MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0);
	// power mux enable = 1
	MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void
SonarTask(void *pvParameters)
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

		uint32_t ui32waitStartTime;

		// Flag for a sonar that is not working or not connected
		bool sonarUnresponsive = false;

		// // // Send a trigger pulse \\ \\ \\

		// Disable context switching
		taskENTER_CRITICAL();

		//
		// Turn on pulse.
		//
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_PIN_7);

		//
		// Delay for 5 us.
		//
		delayTenMicroseconds(g_ui32SysClock);

		//
		// Turn off pulse.
		//
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, 0);


		// // // Measure pulse length \\ \\ \\

		//
		// Clear Timer A capture flag
		//
		TIMER4_ICR_R |= (1 << 2);

		//
		// Save timer start time for failsafe check
		//
		ui32waitStartTime = TIMER4_TAV_R;

		//
		// Start Timer4 A
		//
		MAP_TimerEnable(TIMER4_BASE, TIMER_A);

		//
		// Wait for first capture event
		//
		// TODO: make failsafe, in case sensor does not respond.
		// add "&& TIMER5 not timed out"
		while((TIMER4_RIS_R & (0x1 << 2)) == 0 && ((TIMER4_TAV_R - ui32waitStartTime) & 0x0FFFFFF) <= RESPONSE_WAIT_TICKS)
		//while((TIMER4_RIS_R & (0x1 << 2)) == 0 && (TIMER4_TAV_R - ui32waitStartTime) <= RESPONSE_WAIT_TICKS)
		{
		}
		if (((TIMER4_TAV_R - ui32waitStartTime) & 0x0FFFFFF) > RESPONSE_WAIT_TICKS)
		{
			sonarUnresponsive = true;
		}

		//
		// After first event, save timer A's captured value.
		//
		ui32PulseStartTime = TIMER4_TAR_R;

		//
		// Clear Timer A capture flag
		//
		TIMER4_ICR_R |= (1 << 2);

		//
		// Save timer start time for failsafe check
		//
		ui32waitStartTime = TIMER4_TAV_R;

		//
		// Wait for second capture event
		// skip this wait entirely if the first timer failed
		//
		if (!sonarUnresponsive)
		{
			while((TIMER4_RIS_R & (0x1 << 2)) == 0 && ((TIMER4_TAV_R - ui32waitStartTime) & 0x0FFFFFF) <= RESPONSE_WAIT_TICKS)
			//while((TIMER4_RIS_R & (0x1 << 2)) == 0 && (TIMER4_TAV_R - ui32waitStartTime) <= RESPONSE_WAIT_TICKS)
			{
			}

			if (((TIMER4_TAV_R - ui32waitStartTime) & 0x0FFFFFF) > RESPONSE_WAIT_TICKS)
			{
				sonarUnresponsive = true;
			}
		}
		//
		// After second event, save timer A's captured value.
		//
		ui32PulseStopTime = sonarUnresponsive ? TIMER4_TAV_R : TIMER4_TAR_R;

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
		MAP_TimerDisable(TIMER4_BASE, TIMER_A);

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
		UARTprintf("unresp: %d", sonarUnresponsive);
		UARTprintf("\nProx= %d cm\n", ui32DistanceCM);
		xSemaphoreGive(g_pUARTSemaphore);
	}
}

//*****************************************************************************
//
// Configure the timer and its pins for measuring the length of
// ultrasonic sensor echo pulse.
//
//*****************************************************************************
uint32_t
SonarTaskInit(void)
{

	//
	// Enable GPIO M and B
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Enable Timer 4
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);


	MAP_SysCtlDelay(2); // Wait before doing anything with GPIO registers

	//
	// Enable the GPIO pin for the trigger pulse (M7).
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);

	//
	// Enable GPIO pin for timer event capture (M4).
	//
	MAP_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4);

	//
	// Enable GPIO pin for analog (power) mux enable
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);

	//
	// Enable GPIO pin for digital (data) mux enable
	// NOTE: Negative logic enable (i.e. 0 -> enabled; 1 -> disabled)
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);

	//
	// Enable GPIO pin for mux select bit 0
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

	//
	// Enable GPIO pin for mux select bit 1
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);

	//
	// Enable GPIO pin for mux select bit 2
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);

	//
	// Configure PM4 as Timer 4 CCP0
	//
	MAP_GPIOPinConfigure(GPIO_PM4_T4CCP0);

	//
	// Configure timer 4A as a 16-bit event capture up-counter
	//
	MAP_TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

	//
	// Set prescaler to 255. This essentially makes
	// the 16-bit timer a 24-bit timer.
	//
	MAP_TimerPrescaleSet(TIMER4_BASE, TIMER_A, 0xFF);

	//
	// The timer should capture events on both rising and falling edges
	//
	MAP_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

	//
	    // Create the switch task.
	    //
	    if(xTaskCreate(SonarTask, (signed portCHAR *)"Sonar",
	    		SONARTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
	    		PRIORITY_SONAR_TASK, NULL) != pdTRUE)
	    {
	        return(1);
	    }

	//
	// Success.
	//
	return(0);
}
