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
// Prototypes for the Batter task.
//
//*****************************************************************************
extern uint32_t BatteryTaskInit(void);

void Battery_Enable_Configure(void);

void Battery_ADC_Configure(void);

int Battery_Level_Conversion();

void Battery_Voltage_Out(void);


#endif /* BATTERY_TASK_H_ */
