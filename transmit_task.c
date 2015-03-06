/*
 * Author : Drew May
 * Date   : March 5th 2015
 * Version: 1.0
 * Summary: Tiva TM4C receives raw temperature from internal temperature module
 * 			and transmits two bytes worth of information through I2C.
 * 			In this example the master is a Raspberry Pi.
 *
 * Pi Code: Located in GitHub repository
 *
 * Todo
 * Write state machine to read input from Pi and send relevant data
 * Send proximity data from transmit task
 */

/*
 * Data Sent            |   Complete
 * ---------------------|-----------------
 * Temperature value    |       Y
 * Proximity values     |
 * Battery Level values |
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
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//*****************************************************************************
//
// Data type macros to send to Pi before sending data.
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

//*****************************************************************************
//
// ADC Value.
//
//*****************************************************************************
extern uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

uint8_t adc_i2c[2];    //Two byte array to hold adc value (mark as extern value and add mutex to it)

extern uint32_t g_ui32SysClock;

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
	// Enable the I2C0 slave module.
	//
	MAP_I2CSlaveEnable(I2C0_BASE);
	//
	// Set the slave address to SLAVE_ADDRESS.
	//
	MAP_I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);
}

//*****************************************************************************
//
// This task transmits data read in through the ADC through an I2C interface.
//
// Should wait for a byte to be sent that corresponds with the type of data
// requested and then send that data.
//
//*****************************************************************************
static void
TransmitTask(void *pvParameters)
{
	//Type of data being requested.
	uint8_t data_type = 0;

    while(1) //Loop for all eternity
    {
    	//
    	// Wait until data type is received.
    	//
    	while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ));

    	MAP_I2CSlaveDataGet(I2C0_BASE, data_type);

    	switch(data_type) { //Switch statement for sending different data points
    		case TEMPERATUREDATA :

    	    	//Convert adc reading into two byte array
    	    	adc_i2c[0] = (*adc_value & 0xff00) >> 8;
    	    	adc_i2c[1] = (*adc_value & 0x00ff);      //Lowest 8 bits

    	        //
    	        // Wait until slave data is requested
    	        //
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[0]); //Send back the temperature
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[1]); //Send the lowest 8 bits
    			break;

    		case PROX1DATA :
    			//Expressions
    			break;

    		case PROX2DATA :
    			//Expressions
    			break;

    		case PROX3DATA :
    			//Expressions
    			break;

    		case PROX4DATA :
    			//Expressions
    			break;

    		case PROX5DATA :
    			//Expressions
    			break;

    		case PROX6DATA :
    			//Expressions
    			break;

    		case PROX7DATA :
    			//Expressions
    			break;

    		case PROX8DATA :
    			//Expressions
    			break;

    		case BATTDATA :
    			//Expressions
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
    	}
    }
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

    // Create the switch task.
    //
    if(xTaskCreate(TransmitTask, (signed portCHAR *)"Transmit",
    		TRANSMITTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
    		PRIORITY_TRANSMIT_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

	//
	// Success.
	//
	return(0);
}
