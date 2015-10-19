/******************************************************************
 * Title: 		Lidar_mapping
 * Author:		Tobias Pettersson & Jerry Sundin
 * Course:		TNG016
 * Date:		25/8-15
 *
 * Note:		This file contains protypes and structs needed
 * 				for this project
 *
 */

#ifndef LIDARMAPPING_H
#define LIDARMAPPING_H


/******************************************************************
 * 			Includes
 */

/******************************************************************
 * 			Macros
 */
// Macros regarding the measurement of the lidar-unit
//#define MEAS_DELAY					40			// This is the measurement time specified in ms


/******************************************************************
 * 			Typedefs
 */
// Task creation error codes
typedef enum {
	FAILED_CREATE_STEPPER = -1,
	FAILED_CREATE_ORDER = -2,
	FAILED_CREATE_REVERSE = -3,
	FAILED_CREATE_TURNLEFT = -4,
	FAILED_CREATE_TURNRIGHT = -5
}taskCreateError;


/******************************************************************
 * 			Globals
 */

/******************************************************************
 * 			Prototypes
 */
// UART interrupt handler
void uartIntHandler(void);
// Timer A0 interrupt handler
void timerA0IntHandler(void);
// Timer A1 interrupt handler
void timerA1IntHandler(void);
// Function for initialization of ISRs used
int32_t initInts(void);
// Function for initialization of tasks used in this project
int32_t initTasks(void);
// Stepper task function
static void stepperTask(void *pvParameters);
// Order task function
static void orderTask(void *pvParameters);


/******************************************************************
 * 			Declarations
 */

#endif /* LIDARMAPPING_H */
