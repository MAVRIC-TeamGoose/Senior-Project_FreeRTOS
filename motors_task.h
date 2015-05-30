/*
 * motors.h
 *
 *  Created on: Feb 21, 2015
 *      Author: Team Goose
 */

#ifndef MOTORS_H_
#define MOTORS_H_
//used for calibrating the motors.
//implement this code in the beginning of main:
//	leftSpeed = x;
//	rightSpeed = y;
//	MotorFunctionsInit();
//	setMotorSpeed(leftSpeed, rightSpeed);
//	while(MotorsAreCalibrating()){
//		//wait
//	}
int MotorsAreCalibrating();
//Initialize motor pins and speed capture.
void MotorFunctionsInit();

void Timer3IntHandler(void);
//returns a speed from one quadrature encoder, 1 second will ellapse and if you read this again, you'll get the speed from the other motor.
int currentSpeed();

// Configure Motor Speed sensors and input radius of wheel
void ConfigureMotorSpeedSensorTimers(double rad);

// Setup PWM
void ConfigurePWM();

// Setup additional GPIO pins for motors
void ConfigureMotorGPIO();

//rotations per second
int currentRPS();
//return 0 if left motor is being checked, returns 1 if right motor is being checked.
uint32_t whichMotorSpeed();

// Detect Left Motor Rotations (speed)
//uint32_t CalculateLeftSpeed();

// Detect Right Motor Rotations (speed)
//uint32_t CalculateRightSpeed();
// More accurate left rotation (speed)
//uint32_t returnLeftSpeed();
// More accurate right rotation (speed)
//uint32_t returnRightSpeed();
//speed2
int currentSpeed2();
//speed1
int currentSpeed1();


// Set motor speeds
void setMotorSpeed(int32_t leftSpeed, int32_t rightSpeed);

#endif /* MOTORS_H_ */
