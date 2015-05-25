/*
 * Author : Drew May
 * Date   : April 30th 2015
 * Version: 0.7
 * Summary: Tiva TM4C receives sensor data and transmits it through I2C.
 * 			In this example the master is a Raspberry Pi.
 *
 * Pi Code: Located in GitHub repository
 *
 * Send smell data from transmit task
 * Send sound data from transmit tasks
 */

/*
 * Data Sent            |   Complete
 * ---------------------|-----------------
 * Temperature value    |       Y
 * Proximity values     |       Y
 * Battery Level values |       Y
 * Smell values         |
 * Sound values         |
 */

/*
 * Pin Connections
 *
 * Ground is connected between both devices
 * PB2 on Tiva is connected to SCL (Pin 5) on the Raspberry Pi
 * PB3 on Tiva is connected to SDA (Pin 2) on the Raspberry Pi
 *
 * See http://pi.gadgetoid.com/pinout for a detailed
 * pinout of the Raspberry Pi. We are using Pins 2 and 5.
 */

#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"

#include "inc/hw_adc.h" //Had to change one line here on line 416 from 3 to 1
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h" //Header file for using i2c interface
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"

#include "transmit_task.h"
#include "audio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//*****************************************************************************
//
// Data type macros to receive from Pi before sending data.
//
//*****************************************************************************
#define TEMPERATUREDATA             0x1

#define PROX1DATA                   0x2
#define PROX2DATA                   0x3
#define PROX3DATA                   0x4
#define PROX4DATA                   0x5
#define PROX5DATA                   0x6
#define PROX6DATA                   0x7
#define PROX7DATA                   0x8
#define PROX8DATA                   0x9

#define BATTDATA                    0xA

#define LEFTSMELLDATA               0xB
#define RIGHTSMELLDATA              0xC

#define LEFTSOUNDDATA               0xD
#define RIGHTSOUNDDATA              0xE

#define MOTORDATA                   0xF


//*****************************************************************************
//
// The stack size for the test task.
//
//*****************************************************************************
#define TRANSMITTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
//
//*****************************************************************************
#define SLAVE_ADDRESS 0x04


uint8_t g_data_type; //Holds currently requested data

//Flags for transmitting data
uint8_t g_temp_flag;

uint8_t g_prox_flag;

uint8_t g_batt_flag;

uint8_t g_motor_flag;

//*****************************************************************************
//
// Motor Values. (PROB MOVE TO MOTOR TASK AND DECLARE EXTERN HERE)
//
//*****************************************************************************
uint32_t motor_speed;

uint32_t motor_skew;

//*****************************************************************************
//
// ADC Values.
//
//*****************************************************************************
extern uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

uint8_t adc_i2c[2];    //Two byte array to hold adc value

//*****************************************************************************
//
// Proximity Values.
//
//*****************************************************************************
extern int32_t ranges[8];

uint8_t prox_i2c[2];    //Two byte array to hold prox value

extern uint32_t i32VoltageValue; //Battery voltage

uint8_t batt_i2c[2];

//*****************************************************************************
//
// Audio Frequency Values.
//
//*****************************************************************************

// Frequency magnitude information to be sent to brain
extern float32_t left_freq_magnitude[NUM_FREQS];
extern float32_t right_freq_magnitude[NUM_FREQS];

extern xSemaphoreHandle g_pTemperatureSemaphore;

extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pProximitySemaphore;

extern xSemaphoreHandle g_pBatterySemaphore;

portBASE_TYPE xHigherPriorityTaskWoken;

//*****************************************************************************
//
// The interrupt handler for the for I2C0 data slave interrupt.
//
//*****************************************************************************
void
I2C0SlaveIntHandler(void)
{
    //
    // Clear the I2C0 interrupt flag.
    //
    MAP_I2CSlaveIntClear(I2C0_BASE);
    xHigherPriorityTaskWoken = pdFALSE;

    if (MAP_I2CSlaveStatus(I2C0_BASE) == I2C_SLAVE_ACT_RREQ_FBR) {
    	//
    	// Read the data from the master. Values will be a key indicating data to be sent.
    	//
    	g_data_type = MAP_I2CSlaveDataGet(I2C0_BASE);
    	switch(g_data_type) { //Switch statement for sending different data points
    		case TEMPERATUREDATA :
    			//Ensure Temperature is not being read during conversion
    			xSemaphoreTakeFromISR(g_pTemperatureSemaphore, &xHigherPriorityTaskWoken);
    	    	//Convert adc reading into two byte array
    	    	adc_i2c[0] = (*adc_value & 0xff00) >> 8;
    	    	adc_i2c[1] = (*adc_value & 0x00ff);      //Lowest 8 bits
    	    	xSemaphoreGiveFromISR(g_pTemperatureSemaphore, &xHigherPriorityTaskWoken);

    	    	g_temp_flag = 1; //Set flag for temperature
    	    	g_prox_flag = 0;
    	    	g_batt_flag = 0;
    	    	g_motor_flag = 0;
    			break;
    		case PROX1DATA :
    			//Call prox function
    			formatProx(PROX1DATA);
    			break;

    		case PROX2DATA :
    			//Expressions
    			formatProx(PROX2DATA);
    			break;

    		case PROX3DATA :
    			//Expressions
    			formatProx(PROX3DATA);
    			break;

    		case PROX4DATA :
    			//Expressions
    			formatProx(PROX4DATA);
    			break;

    		case PROX5DATA :
    			//Expressions
    			formatProx(PROX5DATA);
    			break;

    		case PROX6DATA :
    			//Expressions
    			formatProx(PROX6DATA);
    			break;

    		case PROX7DATA :
    			//Expressions
    			formatProx(PROX7DATA);
    			break;

    		case PROX8DATA :
    			//Expressions
    			formatProx(PROX8DATA);
    			break;

    		case BATTDATA :
    			//xSemaphoreTake(g_pBatterySemaphore, portMAX_DELAY);
    			xSemaphoreTakeFromISR(g_pBatterySemaphore, &xHigherPriorityTaskWoken);
    			//Convert adc reading into two byte array
    	    	batt_i2c[0] = (i32VoltageValue & 0xff00) >> 8;
    	    	batt_i2c[1] = (i32VoltageValue & 0x00ff);      //Lowest 8 bits
    	    	//xSemaphoreGive(g_pBatterySemaphore);
    	    	xSemaphoreGiveFromISR(g_pBatterySemaphore, &xHigherPriorityTaskWoken);
    	    	g_temp_flag = 0;
    	    	g_prox_flag = 0;
    	    	g_batt_flag = 1;
    	    	g_motor_flag = 0;
    			break;

    		case LEFTSMELLDATA :
    			//Expressions
    			break;

    		case RIGHTSMELLDATA :
    			//Expressions
    			break;

    		case LEFTSOUNDDATA :
    			//Expressions
    			break;

    		case RIGHTSOUNDDATA :
    			//Expressions
    			break;

    		case MOTORDATA :
    			//Motor data is about to be sent
    			//Data sent will be speed and direction skew in that order
    			g_motor_flag = 1;
    			break;
    		default : //If not one of the constants it is motor data
    			if (g_motor_flag == 2) {
    				//Direction/Skew is available
    				motor_skew = g_data_type;
    				g_motor_flag = 0;
    			} else if (g_motor_flag == 1) {
    				//Speed is available
    				motor_speed = g_data_type;
    				g_motor_flag = 2;
    			}
    	}
    } else if (I2CSlaveStatus(I2C0_BASE) == I2C_SLAVE_ACT_TREQ) {

    	switch(g_data_type) { //Switch statement for sending different data points
    		case TEMPERATUREDATA :
    			if (g_temp_flag == 1) {
    				MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[0]); //Send back the temperature
    				g_temp_flag = 2;
    				break;
    			} else if (g_temp_flag == 2) {
    				MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[1]); //Send the lowest 8 bits
    				g_temp_flag = 0;
    			}
    			break;
    		case PROX1DATA :
    			//Call prox function
    			sendProx(PROX1DATA);
    			break;

    		case PROX2DATA :
    			//Expressions
    			sendProx(PROX2DATA);
    			break;

    		case PROX3DATA :
    			//Expressions
    			sendProx(PROX3DATA);
    			break;

    		case PROX4DATA :
    			//Expressions
    			sendProx(PROX4DATA);
    			break;

    		case PROX5DATA :
    			//Expressions
    			sendProx(PROX5DATA);
    			break;

    		case PROX6DATA :
    			//Expressions
    			sendProx(PROX6DATA);
    			break;

    		case PROX7DATA :
    			//Expressions
    			sendProx(PROX7DATA);
    			break;

    		case PROX8DATA :
    			//Expressions
    			sendProx(PROX8DATA);
    			break;
    		case BATTDATA:
    			if (g_batt_flag == 1) {
    				MAP_I2CSlaveDataPut(I2C0_BASE, batt_i2c[0]); //Send back the battery level
    				g_batt_flag = 2;
    				break;
    			} else if (g_batt_flag == 2) {
    				MAP_I2CSlaveDataPut(I2C0_BASE, batt_i2c[1]); //Send the lowest 8 bits
    				g_batt_flag = 0;
    			}
    			break;
    	}
    }

    // Perform a direct context switch if a higher priority task was woken
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

//*****************************************************************************
//
// Configure the I2C0 and its pins.
// Note: Changed function calls to ROM calls by adding MAP_ to statements.
//
//*****************************************************************************
void
ConfigureI2C0(void)
{
	//
	//enable I2C module 0
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	//
	//enable GPIO peripheral that contains I2C 0
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
	while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	//
	MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	//
	// Select the I2C function for these pins.
	//
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	//
	// Set the slave address to SLAVE_ADDRESS.
	//
	MAP_I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);

	//
	// Enable interrupts to the processor.
	//
	MAP_IntMasterEnable();

	// Set priority for I2C interrupt
    MAP_IntPrioritySet(INT_I2C0, (PRIORITY_I2C0_INT << 5));

	//
	// Enable the I2C0 interrupt on the processor (NVIC).
	//
	MAP_IntEnable(INT_I2C0);
    //
    // Configure and turn on the I2C0 slave interrupt.
    //
    MAP_I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
}

void
sendProx(uint8_t sensor)
{
	if (g_prox_flag == 2) {
		MAP_I2CSlaveDataPut(I2C0_BASE, prox_i2c[1]); //Send the lowest 8 bits
		g_prox_flag = 0;
	} else if (g_prox_flag == 1) {
		MAP_I2CSlaveDataPut(I2C0_BASE, prox_i2c[0]); //Send back the proximity
		g_prox_flag = 2;
	}
}

void
formatProx(uint8_t sensor)
{
	//Convert proximity into two byte array
	xSemaphoreTakeFromISR(g_pProximitySemaphore, &xHigherPriorityTaskWoken);
	prox_i2c[0] = (ranges[sensor - 2] & 0xff00) >> 8;
	prox_i2c[1] = (ranges[sensor - 2] & 0x00ff); //Lowest 8 bits
	xSemaphoreGiveFromISR(g_pProximitySemaphore, &xHigherPriorityTaskWoken);
	g_prox_flag = 1; //Set proximity flag
	g_batt_flag = 0;
	g_temp_flag = 0;
	g_motor_flag = 0;
}

//*****************************************************************************
//
// Initializes the I2C task.
//
//*****************************************************************************
uint32_t
TransmitTaskInit(void)
{
	//
	// Initialize I2C
	//
	ConfigureI2C0();

	//
	// Success.
	//
	return(0);
}
