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
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "drunkenSailer_task.h"
#include "motors_task.h"


//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define DRUNKENTASKSTACKSIZE        128         // Stack size in words


// Semaphore for using the UART
extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pProximitySemaphore;



//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32Drunken_SysClock;   // number of clock cycles returned from system clock



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
		vTaskDelay(10000/portTICK_RATE_MS);  // Delay for 10 seconds
	}
}

/*
 * A left turn function when there is object closed to the right side of the robot
 */
void leftTurn(){
	int leftSpeed = 0;
	int rightSpeed = 100;
	setMotorSpeed(leftSpeed, rightSpeed);
}

/*
 * A right turn functin when there is object closed to the left side of the robot
 */
void rightTurn(){
	int leftSpeed = 100;
	int rightSpeed = 0;
	setMotorSpeed(leftSpeed, rightSpeed);
}

/*
 * A back up function that lets the robot move backward a little bit when there is object
 * closed to the front side.
 */
void backUp(){
	int leftSpeed = 50;
	int rightSpeed = 50;
	setMotorSpeed(leftSpeed, rightSpeed);
	ROM_SysCtlDelay(g_ui32Drunken_SysClock*2/3); // delay for 2 seconds
}


/*
 * A generate random value function to generate random values for motor speed
 */
int genRand(int min, int max){
	int randValue;
	int i = 0;
	while(1){
		srand(i);
		// randValue = rand()%(max-min)+min;
		randValue = (rand()% max) + min;
		ROM_SysCtlDelay(g_ui32Drunken_SysClock/3/1000);
		i++;
		return randValue;
	}
}

/*
 * A let go function for the robot
 */
void startWandering(){
	int leftSpeed = genRand(1, 100); // generate random number between 0 and 100
	int rightSpeed = genRand(1, 100);// generate random number between 0 and 100
	setMotorSpeed(leftSpeed, rightSpeed);
}


/**
 * The drunk walk for the robot to search around the enviroment
 * Note: 1. Start random speeds for both motors
 * 		 2. Back up when getting too closed to an object (1 maybe): turn a certain amount of time or degreee
 * 		 3. Turn left or right for about 2 seconds and resume wandring task
 */
void drunken_Walk(){
	// Start the wandering task first
	startWandering();

	xSemaphoreTake(g_pProximitySemaphore, portMAX_DELAY);
	// Check for moving condition
	if(ranges[0] < 10 || ranges[1]<10){   // if left two sonars detected objects
		backUp();
		rightTurn();
		startWandering();
	}else if(ranges[2]<10 || ranges[3] <10 || ranges[4]<10){   // if the three front sonars detect objects
		backUp();
		leftTurn();
		startWandering();
	}else if(ranges[5] <10 || ranges[6]<10){  // if the two right sonars detect objects
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
	// Create the switch task.
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
