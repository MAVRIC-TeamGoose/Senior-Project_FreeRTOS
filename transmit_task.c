/*
 * Author : Drew May
 * Date   : March 6th 2015
 * Version: 0.5
 * Summary: Tiva TM4C receives raw temperature from internal temperature module
 * 			and transmits two bytes worth of information through I2C.
 * 			In this example the master is a Raspberry Pi.
 *
 * Pi Code: Located in GitHub repository
 *
 * Todo
 * Turn I2C while loops into interrupt that feeds semaphore
 *
 * Send battery data from transmit task
 * Send smell data from transmit task
 * Send sound data from transmit tasks
 */

/*
 * Data Sent            |   Complete
 * ---------------------|-----------------
 * Temperature value    |       Y
 * Proximity values     |       Y
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

//*****************************************************************************
//
// ADC Values.
//
//*****************************************************************************
extern uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1

uint8_t adc_i2c[2];    //Two byte array to hold adc value (mark as extern value and add mutex to it)

//*****************************************************************************
//
// Proximity Values.
//
//*****************************************************************************
extern int32_t ranges[8];

extern uint32_t i32VoltageValue; //Battery voltage

uint8_t batt_i2c[2];

extern xSemaphoreHandle g_pTemperatureSemaphore;

extern xSemaphoreHandle g_pUARTSemaphore;

extern xSemaphoreHandle g_pI2CSemaphore;

extern xSemaphoreHandle g_pProximitySemaphore;

extern xSemaphoreHandle g_pBatterySemaphore;

//*****************************************************************************
//
// Global variable to hold the I2C data that has been received.
//
//*****************************************************************************
//static uint32_t g_ui32DataRx;

//*****************************************************************************
//
// This is a flag that gets set in the interrupt handler to indicate that an
// interrupt occurred.
//
//*****************************************************************************
//static bool g_bIntFlag = false;

//*****************************************************************************
//
// The interrupt handler for the for I2C0 data slave interrupt.
//
//*****************************************************************************
/*void
I2C0SlaveIntHandler(void) //Currently never reaching this ISR
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    //
    // Clear the I2C0 interrupt flag.
    //
    I2CSlaveIntClear(I2C0_BASE);

    //
    // Read the data from the slave.
    //
    //g_ui32DataRx = I2CSlaveDataGet(I2C0_BASE);

    //
    // Set a flag to indicate that the interrupt occurred.
    //
    //g_bIntFlag = true;
    //
    // Feed semaphore to wake task
    //
    xSemaphoreGiveFromISR(g_pI2CSemaphore, &xHigherPriorityTaskWoken);
}*/

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
    // Enable the I2C0 interrupt on the processor (NVIC).
    //
    //MAP_IntEnable(INT_I2C0);

    //
    // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
    //
    //MAP_I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
    //MAP_I2CSlaveIntEnable(I2C0_BASE);

	//
	// Enable the I2C0 slave module.
	//
	MAP_I2CSlaveEnable(I2C0_BASE);
	//
	// Set the slave address to SLAVE_ADDRESS.
	//
	MAP_I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);
}

void
sendProx(uint8_t sensor)
{
	//Convert proximity into two byte array
	uint8_t prox_i2c[2];    //Two byte array to hold prox value
	//extern uint32_t adc_value[1]; //Sequencer 3 has a FIFO of size 1
	//adc_i2c[0] = (*adc_value & 0xff00) >> 8;
	//adc_i2c[1] = (*adc_value & 0x00ff);      //Lowest 8 bits
	xSemaphoreTake(g_pProximitySemaphore, portMAX_DELAY);
	prox_i2c[0] = (ranges[sensor - 2] & 0xff00) >> 8;
	prox_i2c[1] = (ranges[sensor - 2] & 0x00ff); //Lowest 8 bits
	xSemaphoreGive(g_pProximitySemaphore);
    //
    // Wait until slave data is requested
    //
    while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    MAP_I2CSlaveDataPut(I2C0_BASE, prox_i2c[0]); //Send back the proximity
    while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    MAP_I2CSlaveDataPut(I2C0_BASE, prox_i2c[1]); //Send the lowest 8 bits
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
	//uint32_t error = 0;

    while(1) //Loop for all eternity
    {

        //
        // Wait for interrupt to occur. (Turn into semaphore)
        //
        //while(!g_bIntFlag)
        //{
        //}
        //xSemaphoreTake(g_pI2CSemaphore, portMAX_DELAY);
       // xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
       // UARTprintf("Reached ISR");
       // xSemaphoreGive(g_pUARTSemaphore);

    	//
    	// Wait until data type is received.
    	//
    	while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ));

    	data_type = MAP_I2CSlaveDataGet(I2C0_BASE);

		// Disable context switching
		//taskENTER_CRITICAL();

    	switch(data_type) { //Switch statement for sending different data points
        //switch(g_ui32DataRx) {
    		case TEMPERATUREDATA :
    			// Disable context switching
    			//taskENTER_CRITICAL();
    			//xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    			//UARTprintf("\nTemp = %d", adc_value[0]);
    			//xSemaphoreGive(g_pUARTSemaphore);

    			//Ensure Temperature is not being read during conversion
    			xSemaphoreTake(g_pTemperatureSemaphore, portMAX_DELAY);
    	    	//Convert adc reading into two byte array
    	    	adc_i2c[0] = (*adc_value & 0xff00) >> 8;
    	    	adc_i2c[1] = (*adc_value & 0x00ff);      //Lowest 8 bits
    	    	xSemaphoreGive(g_pTemperatureSemaphore);
    	        //
    	        // Wait until slave data is requested
    	        //
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	    	//while(!(error))
    	        //{
    	        //	error = MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ;
    	        //}
    	        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[0]); //Send back the temperature
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	        MAP_I2CSlaveDataPut(I2C0_BASE, adc_i2c[1]); //Send the lowest 8 bits
    			// Re-enable context switching
    			//taskEXIT_CRITICAL();
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

    		case BATTDATA :
    			//Expressions
    			//xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    			//UARTprintf("\nBatt = %d", i32VoltageValue);
    			//xSemaphoreGive(g_pUARTSemaphore);
    			xSemaphoreTake(g_pBatterySemaphore, portMAX_DELAY);
    			//Convert adc reading into two byte array
    	    	batt_i2c[0] = (i32VoltageValue & 0xff00) >> 8;
    	    	batt_i2c[1] = (i32VoltageValue & 0x00ff);      //Lowest 8 bits
    	    	xSemaphoreGive(g_pBatterySemaphore);

    	        //
    	        // Wait until slave data is requested
    	        //
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	    	//while(!(error))
    	        //{
    	        //	error = MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ;
    	        //}
    	        MAP_I2CSlaveDataPut(I2C0_BASE, batt_i2c[0]); //Send back the temperature
    	        while(!(MAP_I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_TREQ));
    	        MAP_I2CSlaveDataPut(I2C0_BASE, batt_i2c[1]); //Send the lowest 8 bits
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
    			//Change PWM outputs accordingly
    			break;
    		default :
    			xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    			UARTprintf("\nYou have messed up");
    			xSemaphoreGive(g_pUARTSemaphore);
    	}

		// Re-enable context switching
		//taskEXIT_CRITICAL();
    	vTaskDelay(500);
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
