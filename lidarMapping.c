/******************************************************************
 * Title: 		Lidar_mapping
 * Author:		Tobias Pettersson & Jerry Sundin
 * Course:		TNG016
 * Date:		25/8-15
 *
 * Note:		This file contains the main-function and some other
 * 				initialization functions in order to start the
 * 				program for Lidar_mapping
 *
 */

/*
 * 	TODO:
 * 			Add stepper-task - CHECK
 * 			Add timer-interrupt to stepper task
 * 			Add timer-interrupt to lidar measurement
 * 			Add lidar-task, to handle measurements
 * 			Add UART-interrupt to handle incomming bl-messages
 * 			Add order-task to handle bl-messages
 * 			Add movement-task to
 */


/******************************************************************
 * 			Includes
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// RTOS Header files
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Tivaware includes
#include "inc/hw_memmap.h"

// Project specific includes
#include "lidarMapping.h"
#include "Lidar_setup.h"
#include "UART_setup.h"
#include "drive_control.h"
#include "tobbeQueue.h"
#include "setupTimers.h"

/******************************************************************
 * 			Macros
 */
// Stepper task
#define STEPPER_STACK_SIZE				512			// Sets the maximum task size
#define PRIORITY_STEPPER_TASK			10			// Sets the priority this task run at

// Order task
#define ORDER_STACK_SIZE				512			// Sets the maximum task size
#define PRIORITY_ORDER_TASK				1			// Sets the priority this task run at



/******************************************************************
 * 			Globals
 */
// Stepper variables
uint8_t stepCount = 0;
uint8_t step = 0;

// Semaphores
xSemaphoreHandle stepperSemHandle;


/******************************************************************
 * 			Declarations
 */
// Inits the ISRs used
int32_t initInts(void) {

	//

	return true;
}

// UART interrupt handler
void uartIntHandler(void) {

}

// Timer A0 interrupt handler
void timerA0IntHandler(void) {

}

// Timer A1 interrupt handler
void timerA1IntHandler(void) {

}

// Inits the tasks used
int32_t initTasks(void) {

	// Initialiaze the stepper task and stepper mutex
	stepperSemHandle = xSemaphoreCreateMutex();
	if ( xTaskCreate(stepperTask, (signed char *)"STEPPER", STEPPER_STACK_SIZE, NULL,
            tskIDLE_PRIORITY + PRIORITY_STEPPER_TASK, NULL) != pdTRUE )
	{
		return FAILED_CREATE_STEPPER;
	}

	if ( xTaskCreate(orderTask, (signed char *)"ORDER", ORDER_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + PRIORITY_ORDER_TASK, NULL) != pdTRUE )
	{
		return FAILED_CREATE_ORDER;
	}

	return true;
}

/******************************************************************
 * 			Tasks
 */
// Stepper task
static void stepperTask(void *pvParameters) {
	// Init variables
	uint8_t canWeStep = 0;
	uint8_t dir = 0;

	// Task main-loop
	while (1) {
		// Wait for semaphore before going on
		//Semaphore_pend(sem_stepper,BIOS_WAIT_FOREVER);
		xSemaphoreTake(stepperSemHandle, 1000);

		// This is the standard case
		if ( canWeStep == 1 ) {
			// We want to keep the step_count between zero and MAX_STEPS
			if ( stepCount > 0 && stepCount < 100 ) {
				if ( dir == 0 ) {
					stepCount++;
				}
				else {
					stepCount--;
				}
			}
			else {
				// Are we at endpoints?
				if ( stepCount == 0 ) {
					// Setting direction to clockwise
					dir = 0;
					stepCount++;
				}
				else if ( stepCount == 100 ) {
					// Setting direction to counter-clockwise
					dir = 1;
					stepCount--;
				}
			}
			// Set up cases depending on the STATUS-vector
			// Per std we take 1 at a time
			step = takeStep(step,dir);
		}

		// Give back semaphore
		xSemaphoreGive(stepperSemHandle);

		// Tell the Lidar-unit that we want a reading
		I2CSend(LID_WRITE_ADDR,0x00,0x04); // Prepare Lidar for reading

		// We must start some sort of timer here in order to be able to make a reading
		// after a specified delay

		// We alse disable Timer 2 so we won't take any step
		// during this time
		disable_Timer(TIMER2_BASE);

		// We enable Timer 1 for an delay of 20 ms before we
		// actually read the lidar-unit
		enable_Timer(TIMER1_BASE, (SYSCLOCK / MEAS_DELAY));			// enable Timer 1

		// Initialize the timer for an 5 ms delay between steps
		//enable_Timer(TIMER2_BASE, (SYSCLOCK / ( TIME_BET_STEP )));

		// Allow steps to happen
		canWeStep = 1;

	}
}

// Order task
static void orderTask(void *pvParameters) {

}
