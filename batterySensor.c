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

//*****************************************************************************
//
// Configure PWM functions
//
//*****************************************************************************
void Configure_BatteryGIPO()
{

}

void Configure_BatteryADC()
{

}

// Set the left and right motor speeds
// Valid inputs are from -100 to 100 for each motor
void samplingVoltage(int32_t leftSpeed, int32_t rightSpeed)
{

}
