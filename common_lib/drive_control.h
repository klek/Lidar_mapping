
#ifndef DRIVE_CONTROL_H_
#define DRIVE_CONTROL_H_

/*
 *	Defines specific to the drive-control
 */
#define PWM_FREQUENCY				55
#define MAX_FORWARD_SPEED			350				// Indicates the maximum forward speed we can accept
#define CRAWL_SPEED					250
#define BREAK_DIST					1000			// We need 10 cm to break the robot down
#define MAX_TURN_SPEED				500				// Indicates the maximum turn speed of the robot
#define MAX_PWM_DIFF				500
#define STATIC_ERROR				100

#define REG_SIZE					80000
#define MAX_DIFF_IN_TICKS			5				// Defines the maximum number diff in ticks from the encoders
#define TICK_LENGTH					9				// This is the length of a tick in m⁻⁴
#define TICK_DEG_LENGTH				12				// This is the length of one tick in degrees

#define STD_TURN					114				// Define for 90 degrees turn	// 115 original
#define STD_LEFT_TICKS_R			113				// 120 original // 110 Senaste
#define STD_RIGHT_TICKS_R			108				// 115 original // 100 Senaste

#define STD_LEFT_TICKS_L			110
#define STD_RIGHT_TICKS_L			110
#define SYSCLOCK					80000000

/*
 *	Prototypes
 */
void InitPWM(void);
void SetPWMLevel(uint16_t Left, uint16_t Right);
void increase_PWM(uint16_t startValLeft, uint16_t endValLeft, uint16_t startValRight, uint16_t endValRight);
void decrease_PWM(uint16_t startValLeft, uint16_t endValLeft, uint16_t startValRight, uint16_t endValRight);
void go_forward(void);
void go_back(void);
void go_left(void);
void go_right(void);
void halt(void);
void resume(void);

// ADC stuff
void InitADC(void);

void InitQEI(void);
int16_t travel_dist();
int16_t travel_degrees();
int16_t travel_ticks(uint8_t *tickLeft_save, uint8_t *tickRight_save);
void ResetQEI();

#endif /* DRIVE_CONTROL_H_ */
