/*
 * battery_task.h
 *
 * Date: Mar 4, 2015
 * Author: Thinh
 */



#ifndef BATTERY_TASK_H_
#define BATTERY_TASK_H_

//*****************************************************************************
//
// Prototypes for the Battery task.
//
//*****************************************************************************
extern uint32_t BatteryTaskInit(void);

// A function to enable the battery signal pin
void Battery_Enable_Configure(void);

// A function for configuring the ADC before using it
void Battery_ADC_Configure(void);

// A function to convert the ADC value to voltage level
int Battery_Level_Conversion();

// A function to get the voltage out
void Battery_Voltage_Out(void);


#endif /* BATTERY_TASK_H_ */
