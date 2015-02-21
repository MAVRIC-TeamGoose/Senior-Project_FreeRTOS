/*
 * motors.h
 *
 *  Created on: Feb 21, 2015
 *      Author: Team Goose
 */

#ifndef MOTORS_H_
#define MOTORS_H_

// Setup PWM
void ConfigurePWM();

// Setup additional GPIO pins for motors
void ConfigureMotorGPIO();

// Set motor speeds
void setMotorSpeed(int32_t leftSpeed, int32_t rightSpeed);

#endif /* MOTORS_H_ */
