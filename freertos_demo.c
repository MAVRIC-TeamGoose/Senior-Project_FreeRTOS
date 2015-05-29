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
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "sonar_task.h"
#include "transmit_task.h"
#include "adc_setup_task.h"
#include "audio.h"
#include "motors_task.h"
#include "batterySensor_task.h"

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

uint32_t currentRightSpeed = 0;
uint32_t currentLeftSpeed = 0;

//****************************************************************************
//
// Debugging Macros
//
//****************************************************************************
#define SONAR_CONNECTED 1

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

xSemaphoreHandle g_pTemperatureSemaphore;

xSemaphoreHandle g_pProximitySemaphore;

xSemaphoreHandle g_pBatterySemaphore;

xSemaphoreHandle g_pAudioSemaphore;

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

// Waits for start button press, and returns a random number
// to be used to seed the RNG. Entropy for the random number
// comes from human input (button press) and a hardware timer.
//
// NOTE: Should be called before any other configuration, as the
// function turns enables and eventually disables several peripherals
// (GPIO and timer)
uint32_t waitForStart()
{
	uint32_t newSeed;

	// Enable peripherals
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);

	// Enable Startup GPIO
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

	MAP_SysCtlDelay(2);

	// Set up timer
	MAP_TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC_UP);

	// Set up button
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_AHB_BASE, GPIO_PIN_0);
	MAP_GPIOIntTypeSet(GPIO_PORTJ_AHB_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);
	MAP_GPIODirModeSet(GPIO_PORTJ_AHB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
	MAP_GPIOPadConfigSet(GPIO_PORTJ_AHB_BASE, GPIO_PIN_0,
	                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	MAP_GPIOIntEnable(GPIO_PORTJ_AHB_BASE, GPIO_INT_PIN_0);
	MAP_GPIOIntClear(GPIO_PORTJ_AHB_BASE, GPIO_INT_PIN_0);

	// Set up MAVRIC Program Start GPIO
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0);
	MAP_GPIODirModeSet(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);

	// Start timer
	MAP_TimerEnable(TIMER4_BASE, TIMER_A);

	// Wait for button
	while(!(MAP_GPIOIntStatus(GPIO_PORTJ_AHB_BASE, GPIO_INT_PIN_0) & GPIO_INT_PIN_0)) {}

	// Capture timer value
	newSeed = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	// Send signal to Raspberry Pi
	MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 1);

	// Disable and shutdown timer
	MAP_TimerDisable(TIMER4_BASE, TIMER_A);
	MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_TIMER4);

	// Disable button and shutdown GPIOJ
	MAP_GPIOIntDisable(GPIO_PORTJ_AHB_BASE, GPIO_INT_PIN_0);
	MAP_GPIOIntClear(GPIO_PORTJ_AHB_BASE, GPIO_INT_PIN_0);
	MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOJ);

	// Return timer
	return newSeed;
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
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, g_ui32SysClock);
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

	// Wait for startup, and get a random seed.
	// Human button input is used as a source of entropy
	uint32_t seed = waitForStart();

	//Seed the RNG for drunken sailor walk routine
	srand(seed);

	//
	// Initialize the UART and configure it for 115,200, 8-N-1 operation.
	//
	ConfigureUART();

	//
	// Print demo introduction.
	//
	UARTprintf("\033[2J\nUniversity of Washington Tacoma"
			 "\nComputer Engineering & Systems"
			 "\nSenior Project -"
			 "June 4, 2015"
			 "\nMobile Autonomous Vehicle for Research in Intelligent Control - MAVRIC II"
			 "\nSponsor: Ph.D George Mobus"
			 "\nTeam Goose: Thinh Le, Drew Seth May, Keith Lueneburg, Brandon Thomas Dean\n\n");


//	UARTprintf("\033[2J\nWelcome to a simple FreeRTOS Demo for the EK-TM4C1294XL!\n");

	// Debugging printout for RNG seeding
	//UARTprintf("Seed:%u\nR1:%u\nR2:%u\nR3:%u\nR4:%u\n", seed, rand(), rand(), rand(), rand());

	//
	// Create a mutex to guard the UART.
	//
	g_pUARTSemaphore = xSemaphoreCreateMutex();
	//
	// Create a mutex to guard the ADC
	//
	g_pTemperatureSemaphore = xSemaphoreCreateMutex();
	//
	// Create a mutex to guard the Proximity values
	//
	g_pProximitySemaphore = xSemaphoreCreateMutex();
	//
	// Create a mutex to guard the battery level
	//
	g_pBatterySemaphore = xSemaphoreCreateMutex();
	//
	// Create a mutex to guard the battery level
	//
	g_pAudioSemaphore = xSemaphoreCreateMutex();

	//****************************************
	//Motor Stuff
	/*
	void ConfigurePWM();
	// Setup additional GPIO pins for motors
	void ConfigureMotorGPIO();
	// Setup GPIO pins for monitoring motor speed
	void ConfigureMotorSpeedSensorGPIO();
	// Detect Left Motor Rotations (speed)
	uint32_t CalculateLeftSpeed();
	// Detect Right Motor Rotations (speed)
	uint32_t CalculateRightSpeed();
	// More accurate left rotation (speed)
	uint32_t returnLeftSpeed();
	// More accurate right rotation (speed)
	uint32_t returnRightSpeed();
	// Set motor speeds
	void setMotorSpeed(int32_t leftSpeed, int32_t rightSpeed); //rightSpeed -= 4;
	*/
	//****************************************

	leftSpeed = 50;
	rightSpeed = 50;
	ConfigurePWM();
	ConfigureMotorGPIO();
	setMotorSpeed(leftSpeed, rightSpeed);

	//Initialize the ADC functionality
	if (ADCInit() != 0)
	{
		UARTprintf("\nADC failed to initialize\n");
	}
	// Initialize the Ultrasonic sensor task
	if (SONAR_CONNECTED) {
		if(SonarTaskInit() != 0)
		{
			while(1)
			{
				UARTprintf("\nSonar task failed to initialize\n");
			}
		}
	}
	// Initialize the I2C transmitting task
	if(TransmitTaskInit() != 0)
	{
		while(1)
		{
			UARTprintf("\nTransmit task failed to initialize\n");
		}
	}

	if(BatteryTaskInit() != 0)
	{
		while(1)
		{
			UARTprintf("\nBattery task failed to initialize\n");
		}
	}

	if(AudioTaskInit() != 0)
	{
		while(1)
		{
			UARTprintf("\nAudio task failed to initialize\n");
		}
	}

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

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
