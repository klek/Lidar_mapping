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

// XDCtools Header files
//#include <xdc/std.h>
//#include <xdc/cfg/global.h>
//#include <xdc/runtime/Log.h>				// Needed for any Log_info() call, not currently used

// BIOS Header files
//#include <ti/sysbios/BIOS.h>

// RTOS Header files
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


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


/******************************************************************
 * 			Macros
 */

/*****************************************************************
 * 			Error hooks
 */
// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


// This hook is called when an assert happens
void vAssertCalled( const char *pcFile, unsigned long ulLine ) {
    //Handle Assert here
    while(1)
    {
    }
}

// This hook is called by FreeRTOS when idle happens
void vApplicationIdleHook(void) {
    //Handle Idle Hook for Profiling, Power Management etc
}

// This hook is called by FreeRTOS when a malloc fails
void vApplicationMallocFailedHook() {
    //Handle Memory Allocation Errors
    while(1)
    {
    }
}

// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}



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

	/**************************************
	 * 	Init HW-interrupts
	 */
	// The stepper timer


	/**************************************
	 *  Creation of SW-interrupts
	 */
	//


	/**************************************
	 * 	Creation of tasks
	 */
	// Call for the initTasks functions
	if ( initTasks() != true ) {
		// Something went wrong, loop forever
		while (1){

		}
	}

	// Start the scheduler
	vTaskStartScheduler();

	// The scheduler should never return but if it do, then we should stop in a forever-loop
	// also perhaps print some error message.
	while (1) {

	}
}
