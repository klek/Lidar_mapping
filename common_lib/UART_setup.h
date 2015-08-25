/*
 * UART_setup.h
 *
 *  Created on: May 2, 2015
 *      Author: orientera
 */

#ifndef UART_SETUP_H_
#define UART_SETUP_H_

#include "tobbeQueue.h"

// Defines for the UART communication
#define MAX_MSG_SIZE    		4
#define SYSCLOCK				80000000


// Prototype to init the bluetooth communication over UART
void bluetooth_init(void);

// Prototype for sending error msg over bluetooth
void sendErrorMSG(char* data, int size);

// Prototype for sending data over bluetooth
void sendMSG(Queue* q1);

// Prototype for recieving data over bluetooth
int8_t readMSG(Queue* q1);



#endif /* UART_SETUP_H_ */
