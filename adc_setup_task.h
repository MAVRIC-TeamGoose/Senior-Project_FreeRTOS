/*
 * adc_setup.h
 *
 * Date: Mar 5, 2015
 * Author: Drew
 */


//*****************************************************************************
//
// Prototypes for the Transmit task.
//
//*****************************************************************************
extern uint32_t ADCInit(void);
void ConfigureADC(void);
void ConfigureTempTimer(void);
void Timer1IntHandler(void);
