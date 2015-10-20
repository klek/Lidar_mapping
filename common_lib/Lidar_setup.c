#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "Lidar_setup.h"
#include "inc/tm4c123gh6pm.h"

uint16_t I2CReceive(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg) {
	// Integer to store data.
	uint16_t Data = 0;

	//Initialize the i2c for writnign to slave device
	I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

	//Register to be read
	I2CMasterDataPut(I2C1_BASE, reg);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//Wait until slave device finished it's transaction.
	while(I2CMasterBusy(I2C1_BASE));

	//Initialize the i2c to read from slave device
	I2CMasterSlaveAddrSet(I2C1_BASE, ReadAddr>>1, true);

	//Read first Byte
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

	//Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C1_BASE));

	// Read 8 MSB
	Data = I2CMasterDataGet(I2C1_BASE);

	// Shift Data
	Data = Data << 8;

	// Read last 8 Byte
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C1_BASE));

	// Shift in LSB
	Data |= I2CMasterDataGet(I2C1_BASE);

	// If error occur store messg
	char test = I2CMasterErr(I2C1_BASE);

	// Return Data
	return Data;
}

void InitI2C1(void){

	//Enable i2c module 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

	//Reset the i2c module1
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

	//Enable the GPIO periph that contains i2c1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//Configure the pin muxing for i2c1, functions on port A6,A7.
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	//Select i2c funciton on pins
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	//Initialize the i2c master block, I2C module data 1 pin PA(7).
	I2CMasterInitExpClk(I2C1_BASE, SYSCLOCK, false);

	//Enable Pull-up on SDA pin.
	GPIO_PORTA_PUR_R = GPIO_PIN_7;

	//Clear i2c FIFOs
	HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;

}


uint16_t I2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value) {
	//Writing register adress to slave device
	I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

	//Register to be read
	I2CMasterDataPut(I2C1_BASE, reg);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C1_BASE));

	//Register adress to slave device
	I2CMasterSlaveAddrSet(I2C1_BASE, WriteAddr >> 1, false);

	// Value to send.
	I2CMasterDataPut(I2C1_BASE, value);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	// //Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C1_BASE));

	// If error occur return messg.
	return I2CMasterErr(I2C1_BASE);
}

// This function should simply read the value from the Lidar-unit
uint16_t readLidar(/*uint16_t RegL,uint16_t RegH*/void) {
        uint16_t Dist = 0; // Integer to store data
        //I2CSend(L_SLAVE_ADDR,0x00,0x04); // Prepare Lidar for reading
        //SysCtlDelay(200000);
        Dist =  I2CReceive(LID_READ_ADDR, LID_WRITE_ADDR, LID_2_BYTE_READ); // Reads the 8 LSB
        //SysCtlDelay(200000);
        //Dist = Dist + (I2CReceive(L_SLAVE_ADDR, RegH)<< 8); //Reads the 8 MSB

    return Dist;
}

// This function tells Lidar to start measure
void reqLidarMeas(void) {
	I2CSend(L_SLAVE_ADDR,0x00,0x04);
}

// This function should initialize the GPIOs used to control the steppper
void init_stepper(void) {
	// Specify the pins we are gonna use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // We have too unlock PF0 which defaults to NMI-pin
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= PF0;
	//HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= 0x400;
	//HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // PF0 for LED and PF1 + PF2 for the stepper
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, PF0 | PF1 | PF2);

    // Put the pins low
    GPIOPinWrite(GPIO_PORTF_BASE, PF0 | PF1 | PF2, 0x00);
}

// This function takes one step and returns the new position
int takeStep(int step, int DIR) {
        if ( DIR == 1 ) {
                if (step == 0) {
                        step = 2;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 2) {
                        step = 6;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 4) {
                        step = 0;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 6) {
                        step = 4;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
        }
        else if ( DIR == 0 ) {
                if (step == 0) {
                        step = 4;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 2) {
                        step = 0;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 4) {
                        step = 6;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
                else if (step == 6) {
                        step = 2;
                        GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
                }
        }
        return step;

}


// This functions takes the specified amount of steps in the specified direction
// and returns the new current position
int goSteps(int howMany, int step, int DIR) {
	int i = 0;
	for (; i < howMany; i++) {
		if ( DIR == 1 ) {
				if (step == 0) {
						step = 2;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 2) {
						step = 6;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 4) {
						step = 0;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 6) {
						step = 4;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
		}
		else if ( DIR == 0 ) {
				if (step == 0) {
						step = 4;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 2) {
						step = 0;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 4) {
						step = 6;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
				else if (step == 6) {
						step = 2;
						GPIOPinWrite(GPIO_PORTF_BASE, PF1 | PF2, step);
				}
		}

		// **** Some delay between steps may be necessary **** //
		// **** How do we solve this?					  **** //
		// **** Maybe solve with a task instead?		  **** //
	}

    return step;
}

// This function should simply calculate measurepoints depending on given information
// and the value of step. It returns one if we are at a measurepoint and zero otherwise
uint8_t measPoint(uint16_t step) {
	// Half the steps is always a measurepoint
	if ( step == (MAX_STEPS / 2) ) {
		return 1;
	}
	// End points all always measurepoints
	else if ( step == MAX_STEPS || step == 0 ) {
		return 1;
	}
	else {
		// What are the possible measurepoints we can have?
		// How do we calculate these?
		// This depends on MAX_STEPS
		// This depends on the NR_OF_MEAS
		// And the MIN_STEPS_BET_MEAS

		// If we only have MAX_STEPS and NR_OF_MEAS
		// We can only take odd number of measures, so test whether
		// the NR_OF_MEAS is odd or not
		if ( NR_OF_MEAS % 2 ) {
			// Then NR_OF_MEAS is odd, so we will simply subtract one measurement

		}


		return 1;
	}
}

