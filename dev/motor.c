#include "motor.h"

#define MOTOR_USE_TIMER PWMD8
#define PWM_PERIOD 4000U

#define REVERSE true
#define NO_REVERSE false

static const PWMConfig motor_pwmcfg = {
  84000000,
  PWM_PERIOD,     /* PWM period (1/18000 s) in ticks
               for center-aligned mode.      */
  NULL,     /* Callback disabled.            */
  {         /* PWM channel configuration:    */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},  /* CH1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},  /* CH2 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},  /* CH3 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}   /* CH4 */
  },
  0, /* CR2 register value. */
#if STM32_PWM_USE_ADVANCED
  0, /* BDTR register value. Not used for TIM4. */
#endif
  0  /* DIER register value. */
};

static THD_WORKING_AREA(Hall_thread_wa, 1024);
static THD_FUNCTION(Hall_thread, p);

static HallStruct hall_encoder[NUM_OF_MOTOR] = {
  {&GPTD3, HALL_USE_CHANNEL1_2 ,NO_REVERSE,0, 0},
  {&GPTD4, HALL_USE_CHANNEL1_2 ,NO_REVERSE,0, 0}
};

static MotorStruct motors[NUM_OF_MOTOR] = {
  {hall_encoder, PWMA_Channel_1, PWMB_Channel_2,NO_REVERSE, 0.0f, 0.0f, 0, 0},
  {hall_encoder + 1, PWMA_Channel_3, PWMB_Channel_4,REVERSE, 0.0f, 0.0f, 0, 0}
};

MotorStruct* getMotors(void)
{
  return motors;
}

inline void motor_init(void)
{
  chThdCreateStatic(Hall_thread_wa, sizeof(Hall_thread_wa),
  NORMALPRIO + 5,
                    Hall_thread, NULL);

  pwmStart(&MOTOR_USE_TIMER, &motor_pwmcfg);
  motor_pwmUpdate();
}

void motor_pwmUpdate(void)
{
  motors[0].input_temp = (motors[0].reverse)?-motors[0].input : motors[0].input;
  motors[1].input_temp = (motors[1].reverse)?-motors[1].input : motors[1].input;

  motors[0].pwmA =
    (uint16_t)(PWM_PERIOD*((motors[0].input_temp > 0.0f)? motors[0].input_temp : 0.0f));
  motors[0].pwmB =
    (uint16_t)(PWM_PERIOD*((motors[0].input_temp < 0.0f)? -motors[0].input_temp : 0.0f));

  motors[1].pwmA =
    (uint16_t)(PWM_PERIOD*((motors[1].input_temp > 0.0f)? motors[1].input_temp : 0.0f));
  motors[1].pwmB =
    (uint16_t)(PWM_PERIOD*((motors[1].input_temp < 0.0f)? -motors[1].input_temp : 0.0f));

  MOTOR_USE_TIMER.tim->CCR[motors[0].PWMChannelA] = motors[0].pwmA;
  MOTOR_USE_TIMER.tim->CCR[motors[0].PWMChannelB] = motors[0].pwmB;

  MOTOR_USE_TIMER.tim->CCR[motors[1].PWMChannelA] = motors[1].pwmA;
  MOTOR_USE_TIMER.tim->CCR[motors[1].PWMChannelB] = motors[1].pwmB;
}

static THD_FUNCTION(Hall_thread, p)
{
  (void)p;
  chRegSetThreadName("Hall Sensor");

  Hall_init(hall_encoder);
  Hall_init(hall_encoder + 1);

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick+=US2ST(HALL_COUNT);

    if(tick>chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
      tick=chVTGetSystemTimeX();

    Hall_update(hall_encoder);
    Hall_update(hall_encoder + 1);
  }
}
