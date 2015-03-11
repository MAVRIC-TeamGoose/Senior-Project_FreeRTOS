//*****************************************************************************
//
// MAVRIC sensor/motor control platform software
//
// Team Goose
//     Thinh Le
// 	   Keith Lueneburg
//     Drew May
//     Brandon Thomas
//
// Using the TI TM4C1294XL development board, this software provides an
// interface between the MAVRIC robot's "brain" (Raspberry Pi) and the
// robot's "body" (sensors and motors).
//
// Movement will be controlled through two motors, and sensor data will be read from
// a temperature sensor, ultrasonic distance sensors, battery level, and simulated
// sense of smell (implementation to be determined).
//
// For a list of used GPIO pins, see "pin_list.xlsx"
// !! Please keep this updated when you commit !!
//
// Code based off of Texas Instruments example (see below).
//
//*****************************************************************************

//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
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
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


/*#include "driverlib/i2c.h"*/ //Added by Drew!!!!!!!

/*#include "drivers/pinout.h"*/ //Added by Drew!!!!!!

#include "utils/uartstdio.h"

#include "motors.h"
#include "sonar_task.h"
// #include "led_task.h"
// #include "switch_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************

//****************************************************************************
//
// Motor Speeds
//
//****************************************************************************

int32_t leftSpeed;
int32_t rightSpeed;


//****************************************************************************
//
// Debugging Macros
//
//****************************************************************************
#define SONAR_CONNECTED 1
#define TIMER_A                 0x000000ff  // Timer A
#define TIMER_TIMA_TIMEOUT      0x00000001  // TimerA time out interrupt

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;
uint32_t currentRightSpeed = 0;
uint32_t currentLeftSpeed = 0;
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);
}
//****************************************************************************
// Timer 3 Handler; updates leftSpeed and rightSpeed
//****************************************************************************

void
Timer3IntHandler(void)
{
	UARTprintf("\n");
	UARTprintf("\n");
	UARTprintf("ENTERED TIMER 3");
	UARTprintf("ENTERED TIMER 3");
	UARTprintf("ENTERED TIMER 3");


//ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}
uint32_t
speedDifference(uint32_t x, uint32_t y)
{
	uint32_t temp = 0;
	temp = x - y;
	if(temp <= 0){
		return -temp;
	} else {
		return temp;
	}
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
	//
	// Set the clocking to run directly from the crystal at 120MHz.
	//
	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
			SYSCTL_OSC_MAIN |
			SYSCTL_USE_PLL |
			SYSCTL_CFG_VCO_480), 120000000);


	//
	// Initialize the UART and configure it for 115,200, 8-N-1 operation.
	//
	ConfigureUART();
	//
	// Initialize the I2C and set it as slave
	//ConfigureI2C0();  //Added by Drew!!!!!!!!!!!

	//
	// Print demo introduction.
	//
	//UARTprintf("\033[2J\nWelcome to a simple FreeRTOS Demo for the EK-TM4C1294XL!\n");


	/**
	 * Motor Control Code Section
	 * delays are added to functions to allow proper initialization
	 */
	//Enable Timer 3
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	//Enable Timer 3 to go off every 100 ms
	ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock/10);
	//Enable Interrupts on Timer3
	ROM_IntEnable(INT_TIMER3A);
	//Enable Interrupts on Timer3 to go off according to a timer
	ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

	ROM_IntMasterEnable();

	leftSpeed = 40;
	rightSpeed = 37;
	ConfigurePWM();
	ConfigureMotorGPIO();
	setMotorSpeed(leftSpeed, rightSpeed);


	ConfigureMotorSpeedSensorGPIO();

	UARTprintf("\033[2J");
	while(1){
		UARTprintf("\033[;H");
		UARTprintf("Testing motor speeds \n");

		SysCtlDelay(g_ui32SysClock/3);
		currentRightSpeed = returnRightSpeed();
		SysCtlDelay(200);
		currentLeftSpeed = returnLeftSpeed();

		UARTprintf("Left Speed : ");
		//move cursor
		UARTprintf("\033[2;15H");
		//delete and make room
		UARTprintf("\033[K");

		UARTprintf("%d \n", currentLeftSpeed);
		UARTprintf("Right Speed: ");
		//move cursor
		UARTprintf("\033[3;15H");
		//delete and make room
		UARTprintf("\033[K");

		UARTprintf("%d \n", currentRightSpeed);
		//UARTprintf("\033[2J");
		UARTprintf("Difference : ");

		UARTprintf("\033[4;15]");

		UARTprintf("\033[K");

		UARTprintf("%d \n" , speedDifference(currentLeftSpeed, currentRightSpeed));


	}




	/**
	 * Motor Control Code Section End
	 */


	//
	// Create a mutex to guard the UART.
	//
	g_pUARTSemaphore = xSemaphoreCreateMutex();

	// Initialize the Ultrasonic sensor task
//	if (SONAR_CONNECTED) { //recall to add this later on when sonars do get connected!!!!!!! - Brandon
//		if(SonarTaskInit() != 0)
//		{
//			while(1)
//			{
//			}
//		}
//	}

	//
	// Start the scheduler.  This should not return.
	//
	vTaskStartScheduler();

	//
	// In case the scheduler returns for some reason, print an error and loop
	// forever.
	//
	while(1)
	{
	}
}
