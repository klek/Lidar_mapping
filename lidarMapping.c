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

// Project specific includes
#include "lidarMapping.h"
#include "Lidar_setup.h"
#include "UART_setup.h"
#include "drive_control.h"
#include "tobbeQueue.h"

/******************************************************************
 * 			Macros
 */
// Stepper task
#define STEPPER_STACK_SIZE				512			// Sets the maximum task size
#define PRIORITY_STEPPER_TASK			1			// Sets the priority this task run at


/******************************************************************
 * 			Declarations
 */
int32_t initTasks(void) {

	// Initialiaze the stepper task
	if ( xTaskCreate(stepperTask, (signed char *)"STEPPER", STEPPER_STACK_SIZE, NULL,
            tskIDLE_PRIORITY + PRIORITY_STEPPER_TASK, NULL) != pdTRUE )
	{
		return FAILED_CREATE_STEPPER;
	}


	return true;
}

/******************************************************************
 * 			Tasks
 */
// Stepper task
static void stepperTask( void *pvParameters ) {

	// Task main-loop
	while (1) {

	}
}
