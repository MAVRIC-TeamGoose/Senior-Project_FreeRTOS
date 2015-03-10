/*
 * motors.h
 *
 *  Created on: Feb 21, 2015
 *      Author: Team Goose
 */

#ifndef BATTERY_H_
#define BATTERY_H_

// Setup PWM
void Configure_BatteryGIPO();

// Setup additional GPIO pins for motors
void Configure_BatteryADC();

// Set motor speeds
void samplingVoltage(int32_t leftSpeed, int32_t rightSpeed);

#endif /* BATTERY_H_ */
