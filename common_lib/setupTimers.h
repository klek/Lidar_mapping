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

void disable_Timer(uint32_t Timer);

void enable_Timer(uint32_t Timer, uint32_t Load);

/******************************************************************
 * 			Declarations
 */


#endif /* SETUPTIMERS_H */
