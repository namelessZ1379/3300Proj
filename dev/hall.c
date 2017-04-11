#include "hall.h"

#define HALL_INITIAL_VALUE 32767U
#define HALL_RANGE 30000U

#define HALL_UPPER_LIMIT HALL_INITIAL_VALUE + HALL_RANGE
#define HALL_LOWER_LIMIT HALL_INITIAL_VALUE - HALL_RANGE

/*
 * GPT3 configuration.
 */
static const GPTConfig hall_cfg = {
  84000000,    /* 10kHz timer clock.*/
  NULL,   /* Timer callback.*/
  0,
  0
};

void Hall_init(HallStruct* Hall)
{
  GPTDriver* Hall_GPT = Hall->Timer;

  gptStart(Hall_GPT, &hall_cfg);

  if (HALL_USE_CHANNEL1_2)
  {
    Hall_GPT->tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1);
    Hall_GPT->tim->CCMR1 |= STM32_TIM_CCMR1_CC2S(1);
    Hall_GPT->tim->CCMR1 |= STM32_TIM_CCMR1_IC1F(8);
    Hall_GPT->tim->CCMR1 |= STM32_TIM_CCMR1_IC2F(8);
  }
  else
  {
    Hall_GPT->tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1);
    Hall_GPT->tim->CCMR2 |= STM32_TIM_CCMR2_CC4S(1);
    Hall_GPT->tim->CCMR2 |= STM32_TIM_CCMR2_IC3F(8);
    Hall_GPT->tim->CCMR2 |= STM32_TIM_CCMR2_IC4F(8);
  }

  Hall_GPT->tim->SMCR |= STM32_TIM_SMCR_SMS(3);
  Hall_GPT->tim->CNT = HALL_INITIAL_VALUE;
  Hall->count_prev = HALL_INITIAL_VALUE;

  Hall_GPT->tim->CR1 |= STM32_TIM_CR1_CEN;
}

void Hall_update(HallStruct* Hall)
{
  uint16_t count = Hall->Timer->tim->CNT;

  Hall->count += ((Hall->reverse)?
      (count - Hall->count_prev):(Hall->count_prev - count));

  Hall->count_prev = count;

  if(count > HALL_UPPER_LIMIT)
  {
    Hall->Timer->tim->CNT = count - HALL_RANGE;
    Hall->count_prev -= HALL_RANGE;
  }
  else if (count < HALL_LOWER_LIMIT)
  {
    Hall->Timer->tim->CNT = count + HALL_RANGE;
    Hall->count_prev += HALL_RANGE;
  }
}
