

#ifndef LIDAR_SETUP_H_
#define LIDAR_SETUP_H_

// Defines for the LidarLite-unit
#define L_SLAVE_ADDR    		0x62
#define LID_WRITE_ADDR			0xc5
#define LID_READ_ADDR			0xc4
#define REGLOW                  0x10
#define REGHIGH                 0x0f
#define LID_2_BYTE_READ			0x8f

#define PF0						GPIO_PIN_0
#define PF1                     GPIO_PIN_1
#define PF2                     GPIO_PIN_2

#define MAX_STEPS				100
#define MIN_STEPS_BET_MEAS		15
#define NR_OF_MEAS				7
#define MEAS_DELAY				40
#define TIME_BET_STEP			200				// This value should represent the divider
												// for the SysCtlClockGet()
#define SYSCLOCK				80000000

//Initialising function for I2C1
void InitI2C1(void);

// Read function for I2C1
uint16_t I2CReceive(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg);

// Write to I2C1
uint16_t I2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value);

// Reading the values from the lidar-unit
uint16_t readLidar(/*uint16_t RegL, uint16_t RegH*/void);

// This function tells Lidar to start measure
void reqLidarMeas(void);

// Prototype for the stepper
void init_stepper(void);

// Stepper should take a step
int takeStep(int step, int DIR);

// Prototype for taking several steps at a time
int goSteps(int howMany, int step, int DIR);

// Prototype for evaluation measure-points
uint8_t measPoint(uint16_t step);


#endif /* LIDAR_SETUP_H_ */
