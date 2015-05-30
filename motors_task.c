/*
 * motors.c
 *
 *  Created on: Feb 21, 2015
 *      Author: Team Goose
 */

//****NOTE******
//PWM pins might be reversed! I wasn't going to address this issue until our entire bot was assembled. Fixes can all be done in software.
//**************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_qei.h"
#include "driverlib/qei.h"
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
#define TIMER_TIMA_TIMEOUT      0x00000001
#define TIMER_A                 0x000000ff
#define GPIO_PS6_T5CCP0         0x00101803
#define GPIO_PS7_T5CCP1         0x00101C03

uint32_t switcher = GPIO_PIN_3;
uint32_t rightSpeedArray[10];
uint32_t leftSpeedArray[10];
uint32_t ui32QEIVelocityLeft[10];
uint32_t ui32QEIVelocityRight[10];
uint32_t ui32DefaultVelocity = 0;
double modifier = 0;
int32_t leftTemp = 0;
int32_t rightTemp = 0;
int motorSet = 0;
int timesCalled = 0;

volatile int CalibrationReady = 1; //1 means not ready, 0 means ready

int32_t setLeftSpeed = 0; //current speed of left wheel
int32_t setRightSpeed = 0; //current speed of right wheel

int32_t originalLeftSet = 0;
int32_t originalRightSet = 0;


volatile uint32_t motorFreq1 = 0; //1200 hz is the maximum
volatile uint32_t motorFreq2 = 0;
uint32_t lastCount = 0;
//
uint32_t leftPWM = 0;
uint32_t rightPWM = 0;
int counter = 0;
int i = 0; // the values inside the speed capture arrays
int j = 1; // the number of non zero integers in QEIVelocityLeft
int k = 1; // the number of non zero integers in QEIVelocityRight

uint32_t maxMotorFreq = 1250;

double radius = 6.25;
int speed = 0;


//Setup a timer and have it count how long each signal is.
//void testSpeed()
//{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
//	GPIOPinTypeGPIOInput(GPIO_P_BASE, GPIO_PIN_1);
//	if(GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1)){
//		while((GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1))){
//
//		}
//		break;
//	}
//}



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

void Timer3IntHandler(void)
{
	MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, switcher);
	switcher ^= GPIO_PIN_3;
	//TimerDisable(TIMER3_BASE, TIMER_A);
	MAP_TimerDisable(TIMER5_BASE, TIMER_BOTH);
	//TimerDisable(TIMER5_BASE, TIMER_B);
	motorFreq1 = 20000 - MAP_TimerValueGet(TIMER5_BASE, TIMER_B);
//	counter = TimerValueGet(TIMER5_BASE, TIMER_B);
//	motorFreq1 = (counter - lastCount) & 0xFFFF;
//	lastCount = counter;
	motorFreq2 = 20000 - MAP_TimerValueGet(TIMER5_BASE, TIMER_A);
	//UARTprintf("Rotation Speed : %d", motorFreq);
	//(TIMER3_BASE, TIMER_A, 120000000);
	MAP_TimerLoadSet(TIMER5_BASE, TIMER_BOTH, 20000); // Timer5
	//TimerLoadSet(TIMER5_BASE, TIMER_B, 1250);
	MAP_TimerEnable(TIMER5_BASE, TIMER_BOTH);
	//TimerEnable(TIMER3_BASE, TIMER_A);
	//TimerEnable(TIMER5_BASE, TIMER_B);
	MAP_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ui32DefaultVelocity = MAP_QEIVelocityGet(QEI0_BASE);
	if(counter < 9){
		if(switcher / 8){
			ui32QEIVelocityLeft[counter]= MAP_QEIVelocityGet(QEI0_BASE);
		}else {
			ui32QEIVelocityRight[counter] = QEIVelocityGet(QEI0_BASE);
		}
	}
	if(counter == 9){
		setMotorSpeed(setLeftSpeed, setRightSpeed);
		MAP_SysCtlDelay(10);
		CalibrationReady = 0; //Calibration is done
	}
	counter++;
}



uint32_t whichMotorSpeed(){
	return switcher / 8;
}

int currentRPS(){
	return ((ui32DefaultVelocity));
}
//int currentRPS2(){
//	return ((motorFreq2));
//}
int currentSpeed1(){ //GPIO B3
	//return (int)((((((float)motorFreq1) / 1600.0)) * (2.0 * 3.14159265359 * radius)) * 1000)
	//(motor-frequency / gear ratio / 16 pulses per 1 axle rotation) * (2 * pi * radius) * (1000, to make a double look like an int during UART printf)
	return (int)((double)ui32DefaultVelocity * 24.34734);
}

int currentSpeed2(){ //GPIO B2
	return (int)((double)ui32DefaultVelocity * 24.34734);
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

//void ConfigureMotorSpeedSensorGPIO()
//{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlDelay(2);
//	//set Pin E0 as input
//	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
//	//set Pin E1 as input
//	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
//}

/**
 * 1. Set timer to capture pulses for one second
 * 2. Record that value and divide it by 16
 * 3. Then divide again by 100
 * 4. This value is rotations per second.
 */
void ConfigureMotorSpeedSensorTimers(double rad)
{
	int j = 0;
	for(j = 0; j < 10; j++){
		ui32QEIVelocityLeft[j] = 0;
		ui32QEIVelocityRight[j] = 0;
	}
	MAP_IntMasterEnable();
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    MAP_GPIOPinConfigure(GPIO_PL1_PHA0);
    MAP_GPIOPinConfigure(GPIO_PL2_PHB0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3, GPIO_PIN_3);
    MAP_GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	MAP_GPIOPinConfigure(GPIO_PB2_T5CCP0);
	MAP_GPIOPinConfigure(GPIO_PB3_T5CCP1);
	radius = rad;
	MAP_IntEnable(INT_TIMER3A);
	MAP_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	//TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT);
	MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT | TIMER_CFG_B_CAP_COUNT);
	MAP_TimerLoadSet(TIMER5_BASE, TIMER_BOTH, 20000);
	//TimerLoadSet(TIMER5_BASE, TIMER_A, 1250);
	MAP_TimerLoadSet(TIMER3_BASE, TIMER_A, 120000000);
	MAP_TimerControlEvent(TIMER5_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
	//TimerControlEvent(TIMER5_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
	//TimerEnable(TIMER5_BASE, TIMER_B);
	MAP_TimerEnable(TIMER5_BASE, TIMER_BOTH);
	MAP_TimerEnable(TIMER3_BASE, TIMER_A);
	MAP_QEIDisable(QEI0_BASE);
	MAP_QEIVelocityDisable(QEI0_BASE);
	MAP_QEIIntDisable(QEI0_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
	MAP_SysCtlDelay(10);
	MAP_QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_RESET_IDX | QEI_CONFIG_QUADRATURE |
			QEI_CONFIG_NO_SWAP), 1599);
	MAP_SysCtlDelay(10);
	MAP_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, 120000000);
	MAP_SysCtlDelay(10);
	MAP_QEIEnable(QEI0_BASE);
	MAP_SysCtlDelay(10);
	MAP_QEIVelocityEnable(QEI0_BASE);
	MAP_SysCtlDelay(10);
	MAP_QEIPositionSet(QEI0_BASE, 999);

}

int MotorsAreCalibrating(){ // 0 = ready, 1 = not ready
	return CalibrationReady;
}
//
//uint32_t CalculateLeftSpeed()
//{
//	uint32_t currentLeftSpeed = 0;
//	while(1) {
//		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)){
//			while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)) {
//				currentLeftSpeed++;
//			}
//			return currentLeftSpeed;
//		}
//	}
//}
//uint32_t CalculateRightSpeed()
//{
//	uint32_t currentRightSpeed = 0;
//	while(1) {
//		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)){
//			while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)) {
//				SysCtlDelay(2);
//				currentRightSpeed++;
//			}
//			return currentRightSpeed;
//		}
//	}
//}
//
//uint32_t returnRightSpeed()
//{
//	return ui32DefaultVelocity;
//}
//
//uint32_t returnLeftSpeed()
//{
//	uint32_t i = 0;
//	uint32_t returnSpeed = 0;
//	while(i < 10){
//		leftSpeedArray[i] = CalculateLeftSpeed();
//		i++;
//		returnSpeed += rightSpeedArray[i];
//	}
//	return returnSpeed / 10;
//}

void MotorFunctionsInit(){
	ConfigurePWM();
	ConfigureMotorGPIO();
//	ConfigureMotorSpeedSensorGPIO();
	ConfigureMotorSpeedSensorTimers(6.25);
}

// Set the left and right motor speeds
// Valid inputs are from -100 to 100 for each motor
void setMotorSpeed(int32_t leftSpeed, int32_t rightSpeed)
{
	timesCalled++;
	if(motorSet == 0){
		originalLeftSet = leftSpeed;
		originalRightSet = rightSpeed;
		motorSet++;
	}
	setLeftSpeed = leftSpeed;
	setRightSpeed = rightSpeed;

	if(counter == 9){
		motorSet--;
		counter++;
		leftTemp = 0;
		rightTemp = 0;
		ui32QEIVelocityRight[0] = 0;
		ui32QEIVelocityRight[8] = 0;
		for(i = 0; i < 10; i++){
			if((ui32QEIVelocityLeft[i] != 0) && ((leftTemp / j) * 0.80) < ((ui32QEIVelocityLeft[i]))){
				leftTemp += ui32QEIVelocityLeft[i];
				j++;
			}
			if((ui32QEIVelocityRight[i] != 0) && ((rightTemp / k) * 0.80) < ((ui32QEIVelocityRight[i]))){
				rightTemp += ui32QEIVelocityRight[i];
				k++;
			}
		}
		leftTemp = leftTemp / (j - 1);
		rightTemp = rightTemp / (k - 1);
		if(leftTemp < rightTemp){
			modifier = (double)leftTemp / (double)rightTemp;
		} 	else {
			modifier = (double)rightTemp / (double)leftTemp;
		}
		if(modifier < 0){
			modifier *= -1;
		}

	}
	if(leftTemp < rightTemp){
		modifier = (double)leftTemp / (double)rightTemp;
	} 	else {
			modifier = (double)rightTemp / (double)leftTemp;
	}
	if(leftTemp > rightTemp){
		leftSpeed = (int32_t)(leftSpeed * modifier);
		leftSpeed = leftSpeed + ((originalLeftSet - leftSpeed) / 2);
		setLeftSpeed = leftSpeed;

	}else if (rightTemp > leftTemp) {
		rightSpeed = (int32_t)(rightSpeed * modifier);
		rightSpeed = rightSpeed + ((originalRightSet - rightSpeed) / 2);
		setRightSpeed = rightSpeed;
	}
//	rightSpeed = (int32_t)(rightSpeed * 0.99); not accurate enough. approx 0.7 cm/s off at best when varying constant multiplier

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
	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_1, leftSpeed);

	if (rightSpeed < 0)
	{
		// right IN_A = 1
		MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 1);
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

	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_GEN_2, rightSpeed);

}
