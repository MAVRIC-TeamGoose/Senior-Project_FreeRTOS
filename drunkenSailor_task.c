/*
 * drunkenTask.c
 * Thinh-Goose Team
 * 5/1/2015
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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
#include "motors_task.h"
#include "drunkenSailor_task.h"


//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define DRUNKENTASKSTACKSIZE        128         // Stack size in words


/** Semaphore for using the UART and  proximity
 *
 */
extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pProximitySemaphore;



//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
extern uint32_t g_ui32SysClock;   // number of clock cycles returned from system clock



//*****************************************************************************
//
// Read Data from sonar
//
//*****************************************************************************

extern int32_t ranges[7]; // imported the array of sonars

// The drunken task
static void
DrunkenTask(void *pvParameters)
{
	while(1){
		drunken_Walk();
//		vTaskDelay(500/portTICK_RATE_MS);  // Delay for 10 seconds
	}
}


/*
 * A backing left turn function when there is object closed to the right side of the robot
 */
void leftTurn(){
	int leftSpeed = -100;
	int rightSpeed = 0;
	setMotorSpeed(leftSpeed, rightSpeed);
	//ROM_SysCtlDelay(g_ui32SysClock*2/3); // delay for 2 seconds
	vTaskDelay(500 / portTICK_RATE_MS);
}


/*
 * A backing right turn function when there is object closed to the left side of the robot
 */
void rightTurn(){
	int leftSpeed = 0;
	int rightSpeed = -100;
	setMotorSpeed(leftSpeed, rightSpeed);
	//ROM_SysCtlDelay(g_ui32SysClock*2/3); // delay for 2 seconds
	vTaskDelay(500 / portTICK_RATE_MS);
}

/*
 * A back up function that lets the robot move backward a little bit when there is object
 * closed to the front side.
 */
void backUp(){
	int leftSpeed = -50;
	int rightSpeed = -50;
	setMotorSpeed(leftSpeed, rightSpeed);
	vTaskDelay(2000 / portTICK_RATE_MS);

	//  Stop the motors
	setMotorSpeed(0, 0);
	vTaskDelay(300 / portTICK_RATE_MS);
}


/*
 * A generate random value function to generate random values for motor speed.
 * NOTE: The stdlib RNG is seeded randomly upon system startup.
 */
int genRand(int min, int max){
	/*int randValue;
	int i = 0;
	while(1){
		srand(i);
		// randValue = rand()%(max-min)+min;
		randValue = (rand()% max) + min;
		ROM_SysCtlDelay(g_ui32SysClock/3/1000);
		i++;
		return randValue;*/
	return (rand() % (max - min)) + min;
}

/*
 * A let go function for the robot
 */
void startWandering(){
	int leftSpeed = genRand(1, 100); // generate random number between 1 and 100
	int rightSpeed = genRand(1, 100);// generate random number between 1 and 100
	setMotorSpeed(leftSpeed, rightSpeed);
	UARTprintf("left = %d, right = %d\n", leftSpeed, rightSpeed);
}


/**
 * The drunk walk for the robot to foraging the enviroment
 * Note: 1. Start random speeds for both motors
 * 		 2. Back up when getting too closed to an object, turn a certain amount of time into another direction
 * 		 3. Turn left or right for about 0.5 second and resume wandring task
 */
void drunken_Walk(){
	xSemaphoreTake(g_pProximitySemaphore, portMAX_DELAY);
	// Check for moving condition
	if(ranges[0] < 25 && ranges[1] < 25){   // if left two sonars detected objects
		backUp();
		rightTurn();
		startWandering();
	}else if((ranges[2]<30 && ranges[3] <30 && ranges[4]<30)||(ranges[2]<30 && ranges[3]<30)|| (ranges[3]<30 && ranges[4]<30)){   // if the three front sonars detect objects
		backUp();
		leftTurn();
		startWandering();
	}else if(ranges[5] <25 && ranges[6]<25){  // if the two right sonars detect objects
		backUp();
		leftTurn();
		startWandering();
	}else{
		startWandering();   // other than that, resume wandering
	}
	xSemaphoreGive(g_pProximitySemaphore);

}



//*****************************************************************************
//
// Initializes the drunken task modules.
//
//*****************************************************************************
uint32_t
DrunkenTaskInit(void)
{

	//
	// Create the drunken task.
	//
	if(xTaskCreate(DrunkenTask, (signed portCHAR *)"DrunkenWalk",
			DRUNKENTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_DRUNKEN_TASK, NULL) != pdTRUE)
	{
		return(1);
	}
	//
	// Success.
	//
	return(0);
}
