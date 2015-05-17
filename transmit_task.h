/*
 * transmit_task.h
 *
 * Date: Mar 4, 2015
 * Author: Drew
 */

#ifndef TRANSMIT_TASK_H_
#define TRANSMIT_TASK_H_

//*****************************************************************************
//
// Prototypes for the Transmit task.
//
//*****************************************************************************
extern uint32_t TransmitTaskInit(void);
void ConfigureI2C0(void);
void sendProx(uint8_t sensor);
void formatProx(uint8_t sensor);
#endif /* TRANSMIT_TASK_H_ */
