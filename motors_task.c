/*
 * motors.c
 *
 *  Created on: Feb 21, 2015
 *      Author: Team Goose
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"

#include "motors_task.h"


//*****************************************************************************
//
// Configure PWM functions
//
//*****************************************************************************

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
uint32_t rightSpeedArray[10];
uint32_t leftSpeedArray[10];


void ConfigurePWM()
{
	// Enable peripherals
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	// Wait for peripherals to be enabled.
	MAP_SysCtlDelay(2);

	// Configure pin PF2 for PWM0 (Generator 1 - Signal 0)
	// Left motor
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
	MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);

	// Configure pin PG0 for PWM0 (Generator 2 - Signal 0)
	// Right motor
	MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
	MAP_GPIOPinConfigure(GPIO_PG0_M0PWM4);

	// Divide 120MHz system clock by 8
	// 20 kHz PWM will be achieved by 750 (divided) clock ticks
	// 20 kHz is the maximum allowed by the Pololu motor controller
	MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

	// Countdown Mode
	PWM0_CTL_R = 0;

	// Immediate updates for parameters
	PWM0_1_GENA_R = 0x0000008C;
	PWM0_1_GENB_R = 0x0000080C;

	PWM0_2_GENA_R = 0x0000008C;
	PWM0_2_GENB_R = 0x0000080C;

	// Set PWM period to 750. (LOAD_R = period - 1)
	//PWM0_0_LOAD_R = 749;
	MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 750);
	MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 750);

	// Set PWM period to 37500. (LOAD = period - 1)
	//PWM0_0_LOAD_R = 37499;
	//MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 37500);

	// Set duty cycle to 100%
	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_1, 750);
	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_2, 750);

	// Set duty cycle to 7.5%  (5% = -90deg; 7.5% = 0deg; 10% = 90deg
	//MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_0, 2813);

	// Enable PWM generator
	MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_2);

	// Enable PWM
	PWM0_ENABLE_R = 0x0000003C;
	//Because these methods are called as a sequence with other methods to initialize tools, delays are added
	//to give time for other initialization methods to begin.
	MAP_SysCtlDelay(2);
}

void ConfigureMotorGPIO()
{
	// Enable GPIO M
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	// Wait for peripherals to be enabled.
	MAP_SysCtlDelay(2);

	// Enable the GPIO pin for the left motor IN_A
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);

	// Enable the GPIO pin for the left motor IN_B
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_1);

	// Enable the GPIO pin for the right motor IN_A
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);

	// Enable the GPIO pin for the right motor IN_B
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);

	// Turn all pins off
	MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5, 0);
	//Because these methods are called as a sequence with other methods to initialize tools, delays are added
	//to give time for other initialization methods to begin.
	MAP_SysCtlDelay(2);
}

void ConfigureMotorSpeedSensorGPIO()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	SysCtlDelay(2);
	//set Pin E0 as input
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
	//set Pin E1 as input
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
}

uint32_t CalculateLeftSpeed()
{
	uint32_t currentLeftSpeed = 0;
	while(1) {
		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)){
			while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)) {
				currentLeftSpeed++;
			}
			return currentLeftSpeed;
		}
	}
}
uint32_t CalculateRightSpeed()
{
	uint32_t currentRightSpeed = 0;
		while(1) {
			if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)){
				while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)) {
					SysCtlDelay(2);
					currentRightSpeed++;
				}
				return currentRightSpeed;
			}
		}
}

uint32_t returnRightSpeed()
{
	uint32_t i = 0;
	uint32_t returnSpeed = 0;
	while(i < 10){
		rightSpeedArray[i] = CalculateRightSpeed();
		i++;
		returnSpeed += rightSpeedArray[i];
	}
	return returnSpeed / 10;

}

uint32_t returnLeftSpeed()
{
	uint32_t i = 0;
	uint32_t returnSpeed = 0;
	while(i < 10){
		leftSpeedArray[i] = CalculateLeftSpeed();
		i++;
		returnSpeed += rightSpeedArray[i];
	}
	return returnSpeed / 10;
}

// Set the left and right motor speeds
// Valid inputs are from -100 to 100 for each motor
void setMotorSpeed(int32_t leftSpeed, int32_t rightSpeed)
{



	if (leftSpeed < 0)
	{
		// left IN_A = 0
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0);
		// left IN_B = 1
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1);

		// Get rid of negative sign on speed so PWM formula works correctly
		leftSpeed = -leftSpeed;
	}
	else
	{
		// left IN_A = 1
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_PIN_0);
		// left IN_B = 0
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);
	}




	// Speed is from 0-100, so we multiply by 15 / 2 to map to 0-750,
	// which is the range for 0-100% duty cycle PWM.
	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_1, (leftSpeed * 15) / 2);

	if (rightSpeed < 0)
	{
		// right IN_A = 1
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);
		// right IN_B = 0
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);

		rightSpeed = -rightSpeed;
	}
	else
	{
		// right IN_A = 0
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);
		// right IN_B = 1
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);
	}

	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_2, (rightSpeed * 15) / 2);

	if((leftSpeed == 0) && (rightSpeed == 0)){
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0);
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);
	}

}
/*
void ConfigureGrabberArmGPIO()
{
	//Enable Pins PK0 and PK1
}

void OpenGrabberArm()
{
	//Set PK1 to 0 and PK0 to 1
}

void CloseGrabberArm()
{
	//Set PK0 to 0 and PK1 to 1
}
*/

