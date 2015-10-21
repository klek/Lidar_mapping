/*
 * UART_setup.c
 *
 *  Created on: May 2, 2015
 *      Author: orientera
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
#include "UART_setup.h"
#include "tobbeQueue.h"

void bluetooth_init(void) {
    // Enable UART5 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

    // First of all -- TESTING PURPOSE
//	UARTDisable(UART5_BASE);

    // Enable GPIO port E which is used for UART5 pins.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure the pin muxing for UART5 functions on port E4 and E5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);

    // Use the internal 16MHz oscillator as the UART clock source.
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Setting up the FIFOs
    UARTFIFODisable(UART5_BASE);
    UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX2_8, UART_FIFO_RX2_8);

    // Initialize the UART over bluetooth
    // Clock, Baudrate, data bits, stop bit, parity
    UARTConfigSetExpClk(UART5_BASE, SYSCLOCK, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Remember to enable the interrupt for the UART-config
    UARTIntEnable(UART5_BASE, UART_INT_RX);
    UARTEnable(UART5_BASE);
    // Also enable FIFO interrupt
    UARTFIFOEnable(UART5_BASE);

    // Initialize the UART for console I/O.
    //UARTStdioConfig(0, 115200, 16000000);
}


void sendErrorMSG(char* data, int size) {
        int i = 0;
        for (; i < size; i++) {
                UARTCharPut(UART5_BASE,data[i]);
        }
}


// This function should send the first node in the queue provided
void sendMSG(Queue* q1) {

	// Test if the msg queue is empty or not
	if ( !isEmpty(q1) ) {
		// Retrieve the first msg in the queue
		Queue_node data = getFront(q1);

		// T
		int i = 0;
		// Send the msg stored at the node
		for ( ; i < MAX_MSG_SIZE; i++) {
			char x = data.data[i];
			UARTCharPut(UART5_BASE, x);
			UARTCharPut(UART5_BASE, ' ');
		}

		// **** HERE WE SHOULD IMPLEMENT SOME SORT OF CHECKING THAT THE MSG **** //
		// **** WAS SENT PROPERLY OR NOT, OTHERWISE WE SHOULD RESEND IT 	**** //
		UARTCharPut(UART5_BASE, '\n');

		// Finally we deque the node
		dequeue(q1);
	}
	// Finally deque the node

}

// This function should read an incomming message and append it to the que
//*** RE-CONFIGURE THIS ONE ***//
int8_t readMSG(Queue* q1) {
        // Create a counter
        int i = 0;
        // Read first char available
        uint8_t size = UARTCharGet(UART5_BASE);

        // Now allocate memory for an array with the number of slots from size
        //uint8_t data[MAX_MSG_SIZE];
        //uint8_t* data = (uint8_t*)calloc(size,sizeof(uint8_t));
        Queue_node calib_mess;
        calib_mess.data[0] = 4;
        calib_mess.data[1] = 'C';
        calib_mess.data[2] = 0;
        calib_mess.data[3] = 0;
        Queue_node data;

        // Save the length of the message at the first slot
        data.data[i] = size;
        i++;

        // Read five chars from the FIFO-buffer
        while ( i < MAX_MSG_SIZE && UARTCharsAvail(UART5_BASE) ) {
        	// Read the byte stored in the fifo
        	data.data[i] = UARTCharGet(UART5_BASE);


			// Increment the counter
			i++;
        }

        for (i = 0; i < MAX_MSG_SIZE; i++) {
            // Return the the message we just got
            UARTCharPut(UART5_BASE, data.data[i]);
        }

        // Now we test so that all data recieved corresponds to the size specified in the messages
        // otherwise we should issue an error message
        if ( data.data[0] != (i + 0x30) ) {
                char msg[] = "ERROR_MSG_SIZE_NOT_CORRECT\n";
                sendErrorMSG(msg,sizeof(msg));
        }

        // Test to see if the message is a halt messages
        // This should simply override all other messages
        // and should not be placed in the queue
        if ( data.data[MSG] == 'h' ) {
        	// Well then we should return a zero
        	return 0;
        }
        // Test if the messages is a start messages
        // This messages should not be placed in queue
        else if ( data.data[MSG] == 'G' ) {
        	// Then we should return a -1
        	return (-1);
        }
        else {
			// Now we have data saved into array
			// Lets enque it
			enqueue(q1,data);
			// Also enqueue a calibration message
			enqueue(q1,calib_mess);
			// And return a 1 for success
			return 1;
        }
}

int8_t readUARTMessage(uint8_t * dataArr, uint8_t size) {
	// Initialize a counter for the messages
	uint8_t i = 0;
	// Read the buffer and
	while ( i < size && UARTCharsAvail(UART5_BASE) ) {
		dataArr[i++] = UARTCharGet(UART5_BASE);
	}

	// Test if there is still data in the register
	if ( UARTCharsAvail(UART5_BASE) && i >= size ) {
		// There is more data than we can read in, return -1
		return MORE_DATA_AVAIL;
	}
	return i;
}
