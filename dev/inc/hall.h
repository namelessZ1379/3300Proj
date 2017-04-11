
#ifndef __HALL_H_
#define __HALL_H_

#include "ch.h"
#include "hal.h"

#define HALL_FREQ                   50U  //Read Hall sensor @ 50Hz
#define HALL_COUNT    1000000U/HALL_FREQ

typedef int32_t hall_cnt_t;

typedef enum {
  HALL_USE_CHANNEL1_2 = 1,
  HALL_USE_CHANNEL3_4 = 0
} Hall_UseChannel;

typedef struct{
  GPTDriver* Timer;
  uint8_t Channel;
  uint8_t reverse;

  hall_cnt_t count;
  uint16_t count_prev;

} HallStruct;

void Hall_init(HallStruct* Hall);
void Hall_update(HallStruct* Hall);

#endif
