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
#define MAX_UART_MSG_SIZE				4
#define MEASUREMENT_DELAY				30
#define SYSCLOCK						80000000

#define DRIVE_FORWARD_TIME				SYSCLOCK * 10		// This should represent a delay of 2 seconds
#define DRIVE_TURN_180_TIME				SYSCLOCK * 2		// This should represent a delay of 1 seconds
#define DRIVE_TURN_TIME					SYSCLOCK / 2		// This should represent a delay of 0.5 seconds

/******************************************************************
 * 			Globals
 */
// Stepper variables
static uint8_t stepCount = 0;
static uint8_t step = 0;

// Defined bits in the status vector
#define DRIVE							(1 << 0)
#define TAKE_MEAS						(1 << 1)
#define DRIVE_F							(1 << 2)
#define DRIVE_L							(1 << 3)
#define DRIVE_R							(1 << 4)
#define DRIVE_LL						(1 << 5)
#define DRIVE_STOP						(1 << 6)
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
void parseMsg(uint8_t * dataArr, uint8_t size);

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
	// Disable the timers that are used
	disableTimer(TIMER1_BASE);
	disableTimer(TIMER2_BASE);

	// Enable all interrupts
	IntMasterEnable();
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


		// Check both interrupts at each iteration in the loop
		if ( int_vec & UART_INT ) {
			// Reset the indication
			int_vec &= ~UART_INT;

			// Remove drive-stop flag to enable movement
			stat_vec &= ~DRIVE_STOP;

			// Init data array
			uint8_t dataArr[MAX_UART_MSG_SIZE];

			// Collect the message
			if ( readUARTMessage(dataArr, MAX_UART_MSG_SIZE) < SUCCESS ) {
				// If we have recieved more data than fits in the vector we should simply
				// go in here again and grab data
				int_vec |= UART_INT;
			}
			// We have gathered a message
			// and now need to determine what the message is
			parseMsg(dataArr, MAX_UART_MSG_SIZE);
		}
		// Checking drive (movement) interrupt
		if ( int_vec & TIMER2_INT ) {
			int_vec &= ~TIMER2_INT;
			// Disable TIMER2
			disableTimer(TIMER2_BASE);
			// Set drive-stop in status vector
			stat_vec |= DRIVE_STOP;
		}
		// Checking measure interrupt
		if ( int_vec & TIMER1_INT ) {
			int_vec &= ~TIMER1_INT;
			// Disable TIMER1
			disableTimer(TIMER1_BASE);

			// Take reading from LIDAR
			mapData[stepCount++] = readLidar();
			SysCtlDelay(2000);

			// Take step
			// Note: We need to take double meas at randvillkor (100) !!!!!
			if ( stepCount > 0 && stepCount < 100 ) {
				stepDir = 1;
			}
			else if ( stepCount >= 100 && stepCount < 200) {
				stepDir = 0;
			}
			else {
				stepDir = 1;
				stepCount = 0;

				// Reset busy-flag
				stat_vec &= ~TAKE_MEAS;
			}
			step = takeStep(step, stepDir);

			// Request reading from LIDAR
			reqLidarMeas();

			if ( stat_vec & TAKE_MEAS ) {
				// Restart TIMER1
				enableTimer(TIMER1_BASE, (SYSCLOCK / MEASUREMENT_DELAY));
			}
			else {
				sendUARTDataVector(mapData, MAP_DATA_SIZE);
				stat_vec &= ~BUSY;
			}
		}

		// Check the drive_stop flag, which always should be set unless we should move
		if ( stat_vec & DRIVE_STOP ) {
			// Stop all movement
			SetPWMLevel(0,0);
			halt();

			// MAKE SURE all drive-flags are not set
			stat_vec &= ~(DRIVE_F | DRIVE_L | DRIVE_R | DRIVE_LL | BUSY);
		}
		// Should we drive?
		else if ( stat_vec & DRIVE ) {
			// Remove drive flag
			stat_vec &= ~DRIVE;
			// Increase PWM
			increase_PWM(0,MAX_FORWARD_SPEED,0,MAX_FORWARD_SPEED);
			if ( stat_vec & DRIVE_F ) {
				enableTimer(TIMER2_BASE, DRIVE_FORWARD_TIME);
			}
			else if ( stat_vec & DRIVE_LL ) {
				enableTimer(TIMER2_BASE, DRIVE_TURN_180_TIME);
			}
			else {
				enableTimer(TIMER2_BASE, DRIVE_TURN_TIME);
			}
		}
		if ( !(stat_vec & BUSY) ) {
			// Tasks
			switch ( stat_vec ) {
				case ((uint8_t)DRIVE_F) :
					// Call drive function
					go_forward();
					// Set the drive flag & BUSY
					stat_vec |= DRIVE | BUSY;
					break;
				case ((uint8_t)DRIVE_L) :
					// Call drive-left function
					go_left();
					// Set the drive flag
					stat_vec |= DRIVE | BUSY;
					break;
				case ((uint8_t)DRIVE_R) :
					// Call drive-right function
					go_right();
					// Set the drive flag
					stat_vec |= DRIVE | BUSY;
					break;
				case ((uint8_t)DRIVE_LL) :
					// Call turn 180-degrees function
					go_back();
					// Set the drive flag
					stat_vec |= DRIVE | BUSY;
					break;
				case ((uint8_t)TAKE_MEAS) :
					// Request reading from LIDAR
					reqLidarMeas();
					// Start TIMER1
					enableTimer(TIMER1_BASE, (SYSCLOCK / MEASUREMENT_DELAY)); // if sysclock = 1 s, 1/120 = 8.3 ms
					// We are busy
					stat_vec |= BUSY;
					break;

				default:
					break;
			}
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
	// Clear the interrupt
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	// Set the corresponding flag in the bit
	int_vec |= TIMER2_INT;
}


// Timer2 interrupt for time between measures
void timerA1IntHandler(void) {
	// Clear the interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Set the corresponding flag in the bit
	int_vec |= TIMER1_INT;
}

void parseMsg(uint8_t * dataArr, uint8_t size) {
	uint8_t i = 0;
	for ( ; i < size; i++ ) {
		switch ( dataArr[i] ) {
			case 'w': stat_vec |= DRIVE_F;
				break;
			case 'd': stat_vec |= DRIVE_R;
				break;
			case 'a': stat_vec |= DRIVE_L;
				break;
			case 's': stat_vec |= DRIVE_LL;
				break;
			case 'r': stat_vec |= TAKE_MEAS;
				break;
			case 'q': stat_vec |= DRIVE_STOP;
				break;

			default:
				break;
		}
	}
}
