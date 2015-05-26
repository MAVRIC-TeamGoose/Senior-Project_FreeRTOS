//*****************************************************************************
//
// priorities.h - Priorities for the various FreeRTOS tasks.
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

#ifndef __PRIORITIES_H__
#define __PRIORITIES_H__

//*****************************************************************************
//
// The priorities of the various tasks.
//
//*****************************************************************************

// FreeRTOS task priorities: High number = high priority
#define PRIORITY_SONAR_TASK       5
#define PRIORITY_BATTERY_TASK	  3
#define PRIORITY_AUDIO_TASK       4
#define PRIORITY_TEMPERATURE_TASK 3
#define PRIORITY_DRUNKEN_TASK	  3

// ARM interrupt priorities: low number = high priority
#define PRIORITY_ADC0_SS2_INT	  6 // Sampling for audio data
#define PRIORITY_TIMER1_INT       6 // Timer for temp sensor
#define PRIORITY_I2C0_INT         4 // Data transmission btw brain and MCU


#endif // __PRIORITIES_H__
