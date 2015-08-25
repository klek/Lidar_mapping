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


/******************************************************************
 * 			Includes
 */

/******************************************************************
 * 			Macros
 */

/******************************************************************
 * 			Typedefs
 */
// Task creation error codes
typedef enum {
	FAILED_CREATE_STEPPER = -1,
	FAILED_CREATE_FORWARD = -2,
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
// Function for initialization of tasks used in this project
int32_t initTasks(void);
// Stepper task function
static void stepperTask( void *pvParameters );


/******************************************************************
 * 			Declarations
 */
