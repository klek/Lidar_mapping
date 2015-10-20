/******************************************************************
 * Title: 		setupTimers
 * Author:		Tobias Pettersson
 * Course:		TNG016
 * Date:		27/8-15
 *
 * Note:		This file should initialize the timers specified
 * 				by the user
 *
 */

#ifndef SETUPTIMERS_H
#define SETUPTIMERS_H


/******************************************************************
 * 			Includes
 */

/******************************************************************
 * 			Macros
 */

/******************************************************************
 * 			Typedefs
 */

/******************************************************************
 * 			Globals
 */

/******************************************************************
 * 			Prototypes
 */
void setupTimers(void);

void disableTimer(uint32_t Timer);

void enableTimer(uint32_t Timer, uint32_t Load);

/******************************************************************
 * 			Declarations
 */


#endif /* SETUPTIMERS_H */
