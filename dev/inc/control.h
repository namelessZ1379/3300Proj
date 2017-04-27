#ifndef _CONTROL_H_
#define _CONTROL_H_

#define  BALANCE_POS 0.0f
#define  CONTROL_PARAMETERS_FLASH 0x08020000
#define  CONTROL_FAIL 15.0f
#define  PARAM_NUM 2U
#define  DEAD_ZONE 0.11f
//#define  CONTROL_DEAD_ZONE 0.002f

typedef struct {
  float Kp_s;
  float Kd_s;
  //float Kp_v;
  //float Ki_v;
  //float int_max
} Control_Param;

//static THD_FUNCTION(Controller_thread, p);
void control_param_init(void);

#endif
