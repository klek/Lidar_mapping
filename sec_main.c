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
#define MAP_DATA_SIZE					200
#define MEAS_DELAY						40
#define SYSCLOCK						80000000

/******************************************************************
 * 			Globals
 */
// Stepper variables
static uint8_t stepCount = 0;
static uint8_t step = 0;

// Defined bits in the status vector
#define TAKE_STEP						(1 << 0)
#define TAKE_MEAS						(1 << 1)
#define DRIVE							(1 << 2)
#define BUSY							(1 << 7)

// Defined bits in the interrupt vector
#define UART_INT						(1 << 0)
#define TIMER1_INT						(1 << 1)
#define TIMER2_INT						(1 << 2)

// The status vactor
static uint8_t stat_vec = 0;
// The interrupt vector
static uint8_t int_vec = 0;

/******************************************************************
 * 			Declarations
 */
void uartIntHandler(void);
void timerA1IntHandler(void);
void timerA2IntHandler(void);


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
	 * 	Init variables
	 */
	uint16_t mapData[MAP_DATA_SIZE];
	uint8_t stepDir = 0;

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
	disableTimer(TIMER1_BASE);
	disableTimer(TIMER2_BASE);

	// TEST
	UARTDisable(UART5_BASE);
	UARTIntDisable(UART5_BASE, UART_INT_RX);

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


		// Check both interrupts each iteration in the loop
		if ( int_vec & UART_INT ) {
			// Reset the indication
			int_vec &= ~UART_INT;
			// Collect the message
		}
		/*
		 * This is not needed
		 */
/*		// Checking stepper interrupt
		if ( int_vec & TIMER2_INT ) {
			int_vec &= ~TIMER2_INT;
			// Indicate stepper to step
		}*/
		// Checking measure interrupt
		if ( int_vec & TIMER1_INT ) {
			int_vec &= ~TIMER1_INT;
			// Disable TIMER1
			disableTimer(TIMER1_BASE);

			// Take reading from LIDAR
			mapData[stepCount++] = readLidar();

			// Take step
			// Note: We need to take double meas at randvillkor (100) !!!!!
			if ( stepCount > 0 && stepCount < 100 ) {
				stepDir = 0;
			}
			else if ( stepCount >= 100 && stepCount < 200) {
				stepDir = 1;
			}
			else {
				stepDir = 0;
				stepCount = 0;

				// Reset busy-flag
				stat_vec &= ~TAKE_MEAS;
			}
			step = takeStep(step, stepDir);

			// Request reading from LIDAR
			reqLidarMeas();

			if ( stat_vec & TAKE_MEAS ) {
				// Restart TIMER1
				enableTimer(TIMER1_BASE, (SYSCLOCK / MEAS_DELAY));
			}
			else {
				stat_vec &= ~BUSY;
			}
		}

		if ( !(stat_vec & BUSY) ) {
			// Tasks
			if ( stat_vec & DRIVE ) {
				// Call drive function
			}
			else if ( stat_vec & TAKE_MEAS ) {
				// Request reading from LIDAR
				reqLidarMeas();
				// Start TIMER1
				enableTimer(TIMER1_BASE, (SYSCLOCK / MEAS_DELAY));
				// We are busy
				stat_vec |= BUSY;
			}
/*			switch ( stat_vec ) {
				case ((uint8_t)DRIVE) :
					// Call drive function
					break;

				case ((uint8_t)TAKE_MEAS) :
					// Request reading from LIDAR
					reqLidarMeas();
					// Start TIMER1
					enableTimer(TIMER1_BASE, (SYSCLOCK / MEAS_DELAY));
					// We are busy
					stat_vec |= BUSY;
					break;

				default:
					break;
			}*/
		}
	}
}

/******************************************************************
 * 			Definitions
 */
// UART-interrupt for bluetooth communication
void uartIntHandler(void) {
	// Clear the interrupt
    UARTIntClear(UART5_BASE, UARTIntStatus(UART5_BASE, true));

    // Set the corresponding flag in vector
    int_vec |= UART_INT;
}

/*
 * Not currently used
 */
void timerA2IntHandler(void) {
	// Do random stuff to get debug
	if ( 1 == 1 ){
		int asd = 0;
		asd += 4;
	}
}


// Timer2 interrupt for time between measures
void timerA1IntHandler(void) {
	// Clear the interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Set the corresponding flag in the bit
	int_vec |= TIMER1_INT;
}
