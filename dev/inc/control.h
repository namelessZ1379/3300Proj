#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "3300misc.h"

#define  BALANCE_POS 0.0f
#define  CONTROL_PARAMETERS_SECTOR        4U
#define  CONTROL_PARAMETERS_FLASH 0x08010000
#define  CONTROL_FAIL 15.0f
#define  PARAM_NUM 11U
#define  DEAD_ZONE 0.11f
//#define  CONTROL_DEAD_ZONE 0.002f
#define  DISPLAY_NUM 4U


typedef struct {
  thread_t *thread;
  uint8_t status;
  uint8_t display_status;
  float Kp_s;
  float Kd_s;
  float Kp_r;
  float Ki_r;
  float int_max_r;
  float Kp_v;
  float Ki_v;
  float int_max_v;
  float Kp_p;
  float Ki_p;
  float int_max_p;
  int32_t display[DISPLAY_NUM];
} Control_Param;

//static THD_FUNCTION(Controller_thread, p);
void control_param_init(void);

#endif
