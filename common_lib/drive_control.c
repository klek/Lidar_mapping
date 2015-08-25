#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "UART_setup.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"

#include "drive_control.h"

// Statics
//static uint32_t tickLeft = 0;
//static uint32_t tickRight = 0;


void InitPWM(void)
{
    // Set PWM clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // Enable PWM-peripherial
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    // Enable GPIO-peripherial
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure pins PD0 and PD1 as PWM and PD2 and PD3 as GPIOs output
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    // Set PWM module clock
    uint32_t ui32PWMClock = SYSCLOCK / 64;
    uint32_t ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    // Configure PWM Mode
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC/*|PWM_GEN_MODE_DBG_RUN*/);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    // Turn the PWM-generator on
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);


}

// Function to set the PWM-duty
void SetPWMLevel(uint16_t Left, uint16_t Right) {
        // Set the PWM-freq
        uint32_t ui32PWMClock = SYSCLOCK / 64;
        uint32_t ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

        // Set PWM-duty to left register
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Right * ui32Load / 1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Left * ui32Load / 1000);
        // Turn the PWM-generator on
        //PWMGenEnable(PWM1_BASE, PWM_GEN_0);
        //PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}

// Function to increase the PWM from start-val to end-val
void increase_PWM(uint16_t startValLeft, uint16_t endValLeft, uint16_t startValRight, uint16_t endValRight) {
    // Set Pulsewidth to start value
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, startValLeft * ui32Load / 1000);
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, startValRight * ui32Load / 1000);
    SetPWMLevel(startValLeft, startValRight);

    // Enable output
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

    // Calculate

    // Loop to increase the
    while ( startValLeft <= endValLeft && startValRight <= endValRight ) {
    	startValLeft += 2;
    	startValRight += 2;
        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, startValLeft * ui32Load / 1000);
        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, startValRight * ui32Load / 1000);
        SetPWMLevel(startValLeft, startValRight);

        // This function simply delays for 10 us
        SysCtlDelay(SYSCLOCK/100000);
    }
}

// Function to decrease the PWM from start-val to end-val
void decrease_PWM(uint16_t startValLeft, uint16_t endValLeft, uint16_t startValRight, uint16_t endValRight) {
    // Set Pulsewidth to start value
    SetPWMLevel(startValLeft, startValRight);

    // Enable output
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

    // Calculate

    // Loop to increase the
    while ( startValLeft > endValLeft && startValRight > endValRight ) {
    	startValLeft -= 2;
    	startValRight -= 2;
        SetPWMLevel(startValLeft, startValRight);

        // This function simply delays for 10 us
        SysCtlDelay(SYSCLOCK/100000);
    }
}

// Should travell the distance specified as distance, distance comes in cm
void go_forward(void) {
	// Set direction pins to high = FORWARD
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_PIN_3);

}

void go_back(void) {
	// Set direction pins to low = BACK
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0);

	// Set PWM output to true
	//PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT|PWM_OUT_1_BIT, true);
}

// Should turn the robot to the right the specified number of degrees
void go_right(void) {
	// Set direction pins to turn right
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,GPIO_PIN_3);

}

void go_left(void) {
	// Set direction pins to turn left
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0);

}

// This function should simply halt the movement of the robot
void halt(void) {
	// Set PWM levels to zero
	//SetPWMLevel(0, 0);
	// Set PWM output to false
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT|PWM_OUT_1_BIT, false);
}

void resume(void) {
	// Get the current PWMLevel from register and increase to this
	// level
	//increase_PWM(0, PWMPulseWidthGet(PWM1_BASE, PWM_OUT_0), 0, PWMPulseWidthGet(PWM1_BASE, PWM_OUT_1));
	// Set PWM output to true
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}


/*
 * 	Functions regarding the ADC to the robot
 */

void InitADC(void)
{
	// Voltage references
	uint32_t LowRef = 2400;
	uint32_t HighRef = 3500;

	//Enables the perpherials for ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	// Enable AIN2 fon PE1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

	//Configure ADC0 to trigger always on sequence 3, sequence 3 is always 1 samples
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_ALWAYS, 0);

	// Configure ADC0, sequence 3, channel 3 to compare-unit and single ended
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2| ADC_CTL_CMP0 | ADC_CTL_END);

	//enables the sequence
	ADCSequenceEnable(ADC0_BASE, 3);

	//configure the compare unit to interrupt on high levels.
	ADCComparatorConfigure(ADC0_BASE, 0, ADC_COMP_TRIG_NONE| ADC_COMP_INT_HIGH_ALWAYS);//ADC_COMP_INT_LOW_ALWAYS

	// Set the low and high regions for the comparator
	ADCComparatorRegionSet(ADC0_BASE, 0, LowRef, HighRef);

	// Reset the comparator
	ADCComparatorReset(ADC0_BASE, 0, true, true);

	// Enable interrupt for
	IntEnable(INT_ADC0SS3);
	// Enable comparator interrupt
	ADCComparatorIntEnable(ADC0_BASE, 3);

	// Clear the interrupt flags
	ADCComparatorIntClear(ADC0_BASE, ADCComparatorIntStatus( ADC0_BASE ));
	// Clear interrupt flag on sequence 3
	ADCIntClear(ADC0_BASE, 3);

	// Finally disable the Sequence
	ADCSequenceDisable(ADC0_BASE, 3);

	// Set priority
	//IntPrioritySet(INT_ADC0SS3, 2);
}


/*
 *	Functions regarding the regulation of the robot
 */

void InitQEI(void) {
	// -------------------------------
	//	QEI0 configuration
	// -------------------------------
	// Enable port D peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Unlock the PD7 GPIO
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;		// Unlocking the port
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = GPIO_PIN_7;			// Unlock the pin
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;					// Lock the port again

	// Configure the pin to be QEI
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	//SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_QEI0);

	// Wait to make sure the peripheral is enabled
	SysCtlDelay(SYSCLOCK / 100000);
	QEIDisable(QEI0_BASE);

	// Configure QEI-pins
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);

	GPIOPinTypeQEI(GPIO_PORTD_BASE, (GPIO_PIN_6 | GPIO_PIN_7));

	// Configure the operation

	// Enable software filter, state must be kept for 3 cycles
	HWREG(QEI0_BASE) = 0xf2000;
	SysCtlDelay(SYSCLOCK / 100000);

	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), REG_SIZE);
	QEIEnable(QEI0_BASE);
	QEIPositionSet(QEI0_BASE, 0);

	// -------------------------------
	//	QEI1 configuration
	// -------------------------------
	// Enable port D peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	// Configure the pin to be QEI
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	//SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_QEI1);

	// Wait to make sure the peripheral is enabled
	SysCtlDelay(SYSCLOCK / 100000);
	QEIDisable(QEI1_BASE);

	// Configure QEI-pins
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	GPIOPinTypeQEI(GPIO_PORTC_BASE, (GPIO_PIN_5 | GPIO_PIN_6));

	// Configure the operation

	// Enable software filter, state must be kept for 3 cycles
	HWREG(QEI1_BASE) = 0xf2000;
	SysCtlDelay(SYSCLOCK / 100000);

	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), REG_SIZE);
	QEIEnable(QEI1_BASE);
	QEIPositionSet(QEI1_BASE, 0);
}

// This function should return the distance traveled since last time in mm
int16_t travel_dist() {
	int16_t distance_travelled = 0;
	int32_t diff_in_ticks = 0;
	// We need to read the current tick from the registers
	int32_t tickLeft = QEIPositionGet(QEI0_BASE);
	int32_t tickRight = QEIPositionGet(QEI1_BASE);

	// Reset the register we just did read
	QEIPositionSet(QEI0_BASE,0);
	QEIPositionSet(QEI1_BASE,0);

	// If the direction of leftDir is 1 and rightDir is -1, then we are turning right
	//if (leftDir == 1 && rightDir == -1) {
	if ( tickRight > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickRight = REG_SIZE - tickRight + 1;
	}
	// If the direction of leftDir is -1 and rightDir is 1, then we are turning left
	//else if (leftDir == -1 && rightDir == 1) {
	if ( tickLeft > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickLeft = REG_SIZE - tickLeft + 1;
	}


	diff_in_ticks = tickLeft - tickRight;

	// If the difference is very big, something is wrong
	//
	if ( diff_in_ticks >  MAX_DIFF_IN_TICKS ) {
		// We take the small value
		distance_travelled = TICK_LENGTH * tickRight;
		//char msg[] = "tickLeft_just_did_jump\n";
		//sendErrorMSG(msg, sizeof(msg));
	}
	else if ( diff_in_ticks < (-MAX_DIFF_IN_TICKS) ) {
		// We take the small value
		distance_travelled = TICK_LENGTH * tickLeft;
		//char msg[] = "tickRight_just_did_jump\n";
		//sendErrorMSG(msg, sizeof(msg));
	}
	else {
		// Calculate the distance travelled in mm
		distance_travelled = TICK_LENGTH * ((tickLeft + tickRight) / 2);
	}

	// Return the distance travelled in mm
	return (distance_travelled);
}

int16_t travel_degrees() {
	int16_t deg_travelled = 0;
	int32_t diff_in_ticks = 0;
	// Read the direction of each side to determine which case to use
	//int8_t leftDir = QEIDirectionGet(QEI0_BASE);
	//int8_t rightDir = QEIDirectionGet(QEI1_BASE);

	// We need to read the current tick from the registers
	int32_t tickLeft = QEIPositionGet(QEI0_BASE);
	int32_t tickRight = QEIPositionGet(QEI1_BASE);

	// Reset the register we just did read
	QEIPositionSet(QEI0_BASE,0);
	QEIPositionSet(QEI1_BASE,0);


	// If the direction of leftDir is 1 and rightDir is -1, then we are turning right
	//if (leftDir == 1 && rightDir == -1) {
	if ( tickRight > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickRight = REG_SIZE - tickRight + 1;
	}
	// If the direction of leftDir is -1 and rightDir is 1, then we are turning left
	//else if (leftDir == -1 && rightDir == 1) {
	if ( tickLeft > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickLeft = REG_SIZE - tickLeft + 1;
	}

	// Calculate the difference
	diff_in_ticks = tickLeft - tickRight;

	// If the difference is very big, something is wrong
	if ( diff_in_ticks > MAX_DIFF_IN_TICKS ) {
		deg_travelled = TICK_LENGTH * TICK_DEG_LENGTH * tickLeft;
		//char msg[] = "tickLeft is used\n";
		//sendErrorMSG(msg, sizeof(msg));
	}
	else if ( diff_in_ticks < (-MAX_DIFF_IN_TICKS) ){
		deg_travelled = TICK_LENGTH * TICK_DEG_LENGTH * tickRight;
		//char msg[] = "tickRight is used\n";
		//sendErrorMSG(msg, sizeof(msg));
	}
	else {
		deg_travelled = TICK_LENGTH * TICK_DEG_LENGTH * ((tickLeft + tickRight) / 2);
	}

	// Return the distance travelled in mm
	return (deg_travelled / 10);

}

int16_t travel_ticks( uint8_t *tickLeft_save, uint8_t *tickRight_save ) {
	int16_t ticks_travelled = 0;
	int32_t diff_in_ticks = 0;
	// Read the direction of each side to determine which case to use
	//int8_t leftDir = QEIDirectionGet(QEI0_BASE);
	//int8_t rightDir = QEIDirectionGet(QEI1_BASE);

	// We need to read the current tick from the registers
	int32_t tickLeft = QEIPositionGet(QEI0_BASE);
	int32_t tickRight = QEIPositionGet(QEI1_BASE);

	// Reset the register we just did read
	QEIPositionSet(QEI0_BASE,0);
	QEIPositionSet(QEI1_BASE,0);


	// If the direction of leftDir is 1 and rightDir is -1, then we are turning right
	//if (leftDir == 1 && rightDir == -1) {
	if ( tickRight > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickRight = REG_SIZE - tickRight + 1;
	}
	// If the direction of leftDir is -1 and rightDir is 1, then we are turning left
	//else if (leftDir == -1 && rightDir == 1) {
	if ( tickLeft > (REG_SIZE - 10000) ) {
		// tickRight will be very big, we need the exact value
		tickLeft = REG_SIZE - tickLeft + 1;
	}

	diff_in_ticks = tickLeft - tickRight;

	// If tickLeft is larger than tickRight
	if ( diff_in_ticks > MAX_DIFF_IN_TICKS ) {
		// Subtract the difference from tickLeft
		tickLeft -= diff_in_ticks;
	}
	if ( diff_in_ticks < (-MAX_DIFF_IN_TICKS) ) {
		tickRight += diff_in_ticks;
	}

	*tickLeft_save += tickLeft;
	*tickRight_save += tickRight;
	ticks_travelled = ((tickLeft + tickRight) / 2);

	// Return the distance travelled in mm
	return (ticks_travelled);

}


void ResetQEI() {
	QEIPositionSet(QEI0_BASE, 0);
	QEIPositionSet(QEI1_BASE, 0);
}
