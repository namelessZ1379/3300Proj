#include "main.h"

static MotorStruct* motors;
static rc_channel_t* rc_channel;

static THD_WORKING_AREA(Shell_thread_wa, 1024);
static THD_WORKING_AREA(Controller_thread_wa, 8192);
static THD_WORKING_AREA(Printing_thread_wa, 128);
static THD_FUNCTION(Controller_thread, p);
static THD_FUNCTION(Printing_thread, p);

static Control_Param controller;

static void cmd_print(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;

  if(!controller.display_status)
  {
    palTogglePad(GPIOD,GPIOD_LED4);
    chprintf(chp,"display on\n");
    controller.display_status = 1;
  }
  else
  {
    palTogglePad(GPIOD,GPIOD_LED4);
    chprintf(chp,"display off\n");
    controller.display_status = 0;
  }
}

static void cmd_Kp(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Kp_s = (float)(finalNum/10000.0f);

  chprintf(chp,"Kp: %d\n", finalNum);
}

static void cmd_Kd(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Kd_s = (float)(finalNum/10000.0f);

  chprintf(chp,"Kd: %d\n", finalNum);
}

static void cmd_Kpr(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Kp_r = (float)(finalNum/10000.0f);

  chprintf(chp,"Kp_r: %d\n", finalNum);
}

static void cmd_Kir(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Ki_r = (float)(finalNum/10000.0f);

  chprintf(chp,"Ki_r: %d\n", finalNum);
}

static void cmd_Kpv(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Kp_v = (float)(finalNum/10000.0f);

  chprintf(chp,"Kp_v: %d\n", finalNum);
}

static void cmd_Kiv(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Ki_v = (float)(finalNum/10000.0f);

  chprintf(chp,"Ki_v: %d\n", finalNum);
}

static void cmd_Kpp(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Kp_p = (float)(finalNum/10000.0f);

  chprintf(chp,"Kp_p: %d\n", finalNum);
}

static void cmd_Kip(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint32_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  controller.Ki_p = (float)(finalNum/10000.0f);

  chprintf(chp,"Ki_p: %d\n", finalNum);
}

static void cmd_control_update(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  uint32_t parameters[PARAM_NUM] = {(uint32_t)(controller.Kp_s * 10000.0f),
                                    (uint32_t)(controller.Kd_s * 10000.0f),
                                    (uint32_t)(controller.Kp_r * 10000.0f),
                                    (uint32_t)(controller.Ki_r * 10000.0f),
                                    (uint32_t)(controller.int_max_r),
                                    (uint32_t)(controller.Kp_v * 10000.0f),
                                    (uint32_t)(controller.Ki_v * 10000.0f),
                                    (uint32_t)(controller.int_max_v),
                                    (uint32_t)(controller.Kp_p * 10000.0f),
                                    (uint32_t)(controller.Ki_p * 10000.0f),
                                    (uint32_t)(controller.int_max_p)};
  flashSectorErase(CONTROL_PARAMETERS_SECTOR);
  flashWrite(CONTROL_PARAMETERS_FLASH, (char*)parameters, PARAM_NUM*4);

  control_param_init();
  chprintf(chp, "Kp_s = %d, Kd_s = %d\n",(uint32_t)(controller.Kp_s * 10000.0f),
                                         (uint32_t)(controller.Kd_s * 10000.0f));
  chprintf(chp, "Kp_r = %d, Ki_r = %d\n",(uint32_t)(controller.Kp_r * 10000.0f),
                                         (uint32_t)(controller.Ki_r * 10000.0f));
  chprintf(chp, "Kp_v = %d, Ki_v = %d\n",(uint32_t)(controller.Kp_v * 10000.0f),
                                         (uint32_t)(controller.Ki_v * 10000.0f));
  chprintf(chp, "Kp_p = %d, Ki_p = %d\n",(uint32_t)(controller.Kp_p * 10000.0f),
                                         (uint32_t)(controller.Ki_p * 10000.0f));
}

void cmd_control_start(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  if(!controller.status)
  {
    chThdCreateStatic(Controller_thread_wa, sizeof(Controller_thread_wa),
      NORMALPRIO + 2,
                      Controller_thread, NULL);

    chThdCreateStatic(Printing_thread_wa, sizeof(Printing_thread_wa),
      NORMALPRIO - 5,
                      Printing_thread, NULL);

    controller.status = 1;
  }
}

static const ShellCommand commands[] =
{

  {"print",cmd_print},
  {"ps" , cmd_Kp},
  {"ds" , cmd_Kd},
  {"pr" , cmd_Kpr},
  {"ir" , cmd_Kir},
  {"pv" , cmd_Kpv},
  {"iv" , cmd_Kiv},
  {"pp" , cmd_Kpp},
  {"ip" , cmd_Kip},
  {"up", cmd_control_update},
  {"s", cmd_control_start}
};

static const ShellConfig shell_cfg1 =
{
  CHP,
  commands
};

void shellStart(void)
{
  sdStart(BLE, NULL);
  motors = getMotors();

  controller.status = 0;
  controller.display_status = 1;

  rc_channel = rc_init();

  control_param_init();
  motor_init();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}

void control_param_init(void)
{
  uint32_t parameters[PARAM_NUM];
  flashRead(CONTROL_PARAMETERS_FLASH, (char*)parameters, PARAM_NUM*4);

  controller.Kp_s = (float)(parameters[0]/10000.0f);
  controller.Kd_s = (float)(parameters[1]/10000.0f);
  controller.Kp_r = (float)(parameters[2]/10000.0f);
  controller.Ki_r = (float)(parameters[3]/10000.0f);
  controller.int_max_r = (float)(parameters[4]);
  controller.Kp_v = (float)(parameters[5]/10000.0f);
  controller.Ki_v = (float)(parameters[6]/10000.0f);
  controller.int_max_v = (float)(parameters[7]);
  controller.Kp_p = (float)(parameters[8]/10000.0f);
  controller.Ki_p = (float)(parameters[9]/10000.0f);
  controller.int_max_p = (float)(parameters[10]);
}

#define POSITION_CONTROL 40.0f
static THD_FUNCTION(Controller_thread, p)
{
  (void)p;
  chRegSetThreadName("Controller");

  static float position_error, position_error_int = 0.0f, position_output;
  static float speed_cmd, speed_error, speed_error_int = 0.0f, speed_output;
  static float error,temp,output;
  static float diff_cmd = 0.0f, diff_error, diff_error_int = 0.0f, diff_output;
  static float encoder_diff = 0.0f, encoder_norm = 0.0f;
  static float rc_cmd;
  static uint8_t stand_up = 0;

  encoder_norm = (float)(motors[0].Hall_Encoder->count
     + motors[1].Hall_Encoder->count);

  diff_cmd = (float)(motors[0].Hall_Encoder->count - motors[1].Hall_Encoder->count);

  error =  g_IMU1.theta - BALANCE_POS;
  if(error > CONTROL_FAIL || error < -CONTROL_FAIL)
    stand_up = 0;
  else
    stand_up = 30;

  while(true)
  {
    rc_cmd = (float)((int16_t)(rc_channel[0]) - 1500);

    if(rc_cmd < POSITION_CONTROL && rc_cmd > -POSITION_CONTROL)
    {
/*      position_error = (float)(motors[0].Hall_Encoder->count
        + motors[1].Hall_Encoder->count) - encoder_norm;
      position_error_int += position_error;

      if(position_error_int > controller.int_max_p)
        position_error_int = controller.int_max_p;
      else if(position_error_int < -controller.int_max_p)
        position_error_int = -controller.int_max_p;

      position_output = controller.Kp_p * position_error;
            + controller.Ki_p * position_error_int;
*/
      speed_cmd = 0.0f;
    }
    else
    {
      encoder_norm = (float)(motors[0].Hall_Encoder->count
        + motors[1].Hall_Encoder->count);
      position_output = 0.0f;
      speed_cmd = rc_cmd/8.0f;
    }

    speed_error = speed_cmd - motors[0].speed;
    speed_error_int += (speed_error * controller.Ki_v);

    if(speed_error_int > controller.int_max_v)
      speed_error_int = controller.int_max_v;
    else if(speed_error_int < -controller.int_max_v)
      speed_error_int = -controller.int_max_v;

    speed_output = atan((controller.Kp_v * speed_error)/1000.0f) * 180.0f/M_PI;

    error =  g_IMU1.theta - BALANCE_POS;

    if((error > CONTROL_FAIL || error < -CONTROL_FAIL) && stand_up >= 30)
    {
      motor_stop();
      palTogglePad(GPIOD, GPIOD_LED5);
    }
    else if(stand_up < 30)
      stand_up++;

    error -= speed_output;

    output = controller.Kp_s * error -
            controller.Kd_s * g_IMU1.gyroFiltered[2] / 150.0f;

    if(stand_up < 2)
      temp = ( g_IMU1.theta - BALANCE_POS > 0 ? -0.17f : 0.062f);
    else if(stand_up < 30)
      temp += output/4.25f;
    else
      temp += output;

    if(temp > 0.6f)
      temp = 0.6f;
    else if(temp < -0.6f)
      temp = -0.6f;

    if(temp > 0.0f)
    {
      motors[0].input = temp + DEAD_ZONE;
      motors[1].input = temp + DEAD_ZONE;
    }
    else
    {
      motors[0].input = temp - DEAD_ZONE;
      motors[1].input = temp - DEAD_ZONE;
    }

    encoder_diff = (float)(motors[0].Hall_Encoder->count - motors[1].Hall_Encoder->count);

    diff_cmd += (float)((int16_t)(rc_channel[1]) - 1500)/30.0f;

    diff_error = diff_cmd - encoder_diff;

    diff_error_int += diff_error;

    if(diff_error_int > controller.int_max_r)
      diff_error_int = controller.int_max_r;
    else if(diff_error_int < -controller.int_max_r)
      diff_error_int = -controller.int_max_r;

    diff_output = (controller.Kp_r * diff_error
            + controller.Ki_r * diff_error_int)/100.0f;

    tft_printf(5,2,"max:%5d", (int16_t)(speed_error_int));
    tft_printf(5,4,"speed+:%5d", (int16_t)(speed_output * 100.0f));
    tft_printf(5,3,"error:%5d", (int16_t)(speed_error));

    motors[0].input_diff = motors[0].input - diff_output;
    motors[1].input_diff = motors[1].input + diff_output;

    if(controller.display_status)
    {
      //controller.display[0] = (int32_t)(position_error);
      controller.display[0] = (int32_t)(speed_error*100.0f);
      controller.display[1] = (int32_t)(speed_output*100.0f);
      controller.display[2] = (int32_t)(error);
      controller.display[3] = (int32_t)(g_IMU1.theta * 100.0f);
    }

    chThdSleepMilliseconds(20);
  }
}

static THD_FUNCTION(Printing_thread, p)
{
  (void)p;
  chRegSetThreadName("Printing Data");

  sdStart(BLE2, NULL);

  static uint8_t i;
  for (i = 1; i < DISPLAY_NUM; i++)
    controller.display[i] = 0;

  while(true)
  {
    if(controller.display_status)
      for (i = 0; i < DISPLAY_NUM + 1; i++)
      {
        if(i == 1)
          chprintf(CHP2,"a.");
        else
          chprintf(CHP2,"%d.",controller.display[i - 2]);

        chThdSleepMilliseconds(200);
      }
    else
      chThdSleepMilliseconds(500);
  }
}
