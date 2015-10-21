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


/******************************************************************
 * 			Includes
 */
#include <stdint.h>
#include <stdbool.h>

// Tivaware includes
#include "inc/hw_timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "Lidar_setup.h"


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

/******************************************************************
 * 			Declarations
 */
/*************************************************
 *	Initializes timers
 */
void setupTimers(void) {
	// Timer 1 setup code
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);			// enable Timer 1 periph clks
	TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT_UP);		// cfg Timer 1 mode - one-shot up

	uint32_t ui32Period;
	ui32Period = (SYSCLOCK / MEAS_DELAY);					// period = CPU clk div 50 (20ms)
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);			// set Timer 1 period
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);		// enables Timer 1 to interrupt CPU

	// Timer 2 setup code
	// Timer 2 is used for the Lidar-unit
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);			// enable Timer 2 periph clks
	TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT_UP);

	// Initialize the timer for an 5 ms delay between steps
	ui32Period = ( SYSCLOCK / TIME_BET_STEP );
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);			// set Timer 2 period
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);		// enables Timer 2 to interrupt CPU

}

/*************************************************
 *	Disables the specified timer
 */
void disableTimer(uint32_t Timer) {
	TimerDisable(Timer, TIMER_A);						// disable Timer
}

/*************************************************
 * 	Enables the specified timer with the specified period
 * 	Usually this period is calculated from the specified SYSCLOCK
 * 	Ex:
 * 		#define SYSCLOCK	80000000
 * 		#define MEAS_DELAY	40
 *
 * 		SYSCLOCK / MEAS_DELAY would then give a period of about
 * 		20 ms
 */
void enableTimer(uint32_t Timer, uint32_t Load) {
	// Resest the current value
	HWREG(Timer + TIMER_O_TAV) = 0;
	TimerLoadSet(Timer, TIMER_A, Load);
	TimerEnable(Timer, TIMER_A);
}
