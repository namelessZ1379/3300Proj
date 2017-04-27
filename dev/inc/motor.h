#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "hall.h"

#define MOTOR_USE_TIMER PWMD8
#define REVERSE true
#define NO_REVERSE false
#define  NUM_OF_MOTOR 2U
#define   motor_stop()    pwmStop(&MOTOR_USE_TIMER)

typedef enum {
	PWMA_Channel_1 = 0,
	PWMA_Channel_2 = 1,
	PWMA_Channel_3 = 2,
	PWMA_Channel_4 = 3,
} PWMChannelA;

typedef enum {
	PWMB_Channel_1 = 0,
	PWMB_Channel_2 = 1,
	PWMB_Channel_3 = 2,
	PWMB_Channel_4 = 3,
} PWMChannelB;

typedef struct {
	HallStruct* Hall_Encoder;
	uint8_t PWMChannelA;
	uint8_t PWMChannelB;
	uint8_t reverse;
	float input;
	float input_temp;
	uint16_t pwmA;
	uint16_t pwmB;
	uint16_t speedRaw[SPEED_COUNT];
	float speed;
	float speed_filtered
} MotorStruct;

MotorStruct* getMotors(void);
inline void motor_init(void);
void motor_pwmUpdate(void);

#endif
