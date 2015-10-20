/******************************************************************
 * Title: 		Lidar_mapping
 * Author:		Tobias Pettersson & Jerry Sundin
 * Course:		TNG016
 * Date:		20/10-15
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

// Tivaware Header files
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"

// Project specific headers
#include "lidarMapping.h"
#include "Lidar_setup.h"
#include "UART_setup.h"
#include "drive_control.h"
#include "setupTimers.h"


/******************************************************************
 * 			Macros
 */

/******************************************************************
 * 			Globals
 */
// Stepper variables
uint8_t stepCount = 0;
uint8_t step = 0;

// Defined bits in the status vector
#define TAKE_STEP						(1 << 0)
#define TAKE_MEAS						(1 << 1)

// The status vactor
static uint8_t stat_vec = 0;

/******************************************************************
 * 			Declarations
 */


/******************************************************************
 * 			Main-function
 */
int main(void) {

	/**************************************
	 * 	Set the clock to run at 80 MHz
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	/**************************************
	 * 	Enable the floating-point-unit
	 */
	FPUEnable();
	// We also want Lazystacking, so enable that too
	FPULazyStackingEnable();
	// We also want to put numbers close to zero to zero
	FPUFlushToZeroModeSet(FPU_FLUSH_TO_ZERO_EN);


	/**************************************
	 * 	Init peripherals used
	 */
	bluetooth_init();
	init_stepper();							// Init the GPIOs used for the stepper and loading LED
	InitI2C1();								// Init the communication with the lidar-unit through I2C
	InitPWM();
	setupTimers();

	/**************************************
	 * 	State 2
	 */
	// Init HW-interrupts

	/**************************************
	 * 	State 3
	 */
	// Indicate we should start with a scan regardless of what other things we have already got
	// from UART-interrupt
	// This means setting the appropriate bit in the status vector
	stat_vec |= TAKE_MEAS;

	/**************************************
	 * 	State 4
	 */
	// Contains main-loop where decisions should be made
	for ( ; ; ) {
		/**********************************
		 * 	Decision tree
		 */
		// Highest priority case first
		switch ( stat_vec ) {
			case
		}
	}
}
