#include "main.h"

static MotorStruct* motors;

static THD_WORKING_AREA(Shell_thread_wa, 1024);
//static THD_WORKING_AREA(Controller_thread_wa, 4096);

//static Control_Param controller;

static void cmd_print(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;

  uint8_t i;
  for (i = 0; i < 3U; i++)
    chprintf(chp,"Accl:%4d   Gyro:%4d\n",
      (int32_t)(g_IMU1.accelBias[i] * 100),
      (int32_t)(g_IMU1.gyroBias[i] * 100));
}

/*
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

  controller.Kp_s = (float)(finalNum/10000.0f);

  chprintf(chp,"Kd: %d\n", finalNum);
}

static void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  while(!imuCalibrate(&g_IMU1, true));
  while(!imuCalibrate(&g_IMU1, false));

  flashSectorErase(6);
  size_t i = 0;
  for (; i < 3U; i++)
  {
     MPUFlash[i] = g_IMU1.accelBias[i];
     MPUFlash[i + 3] = g_IMU1.gyroBias[i];
     chprintf(chp,"AccelBias: %4d   GyroBias: %4d\n",
          (int32_t)(MPUFlash[i] * 100),
          (int32_t)(MPUFlash[i + 3] * 100));
  }

  chprintf(chp,"\n\n");

  float result[6];

  flashWrite(MPU_FLASH_ADDR, (char*)MPUFlash, 24);
  chThdSleepMilliseconds(500);
  flashRead(MPU_FLASH_ADDR,(char*)result, 24);

  for (i = 0; i < 6U; i++)
    chprintf(chp,"FlashData %d: %4d\n",i + 1,(int32_t)(result[i] * 100));

  chprintf(chp,"\n\n");
}

static void cmd_control_update(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  uint32_t parameters[PARAM_NUM] = {(uint32_t)(controller.Kd_s * 10000.0f),
                                    (uint32_t)(controller.Kd_s * 10000.0f)};

  flashSectorErase(5);
  flashWrite(CONTROL_PARAMETERS_FLASH, (char*)parameters, PARAM_NUM*4);

  control_param_init();
  chprintf(chp, "Kp = %d, Kd = %d\n",controller.Kp_s,controller.Kd_s);
}
*/
static void cmd_control_start(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
/*
  chThdCreateStatic(Controller_thread_wa, sizeof(Controller_thread_wa),
    NORMALPRIO + 2,
                    Controller_thread, NULL);*/
}

static const ShellCommand commands[] =
{

  {"print",cmd_print},
  //{"ps" , cmd_Kp},
  //{"ds" , cmd_Kd},
  //{"calibrate",cmd_calibrate},
  //{"up", cmd_control_update},
  //{"s", cmd_control_start}
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

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}

/*
static THD_FUNCTION(Controller_thread, p)
{
  (void)p;
  chRegSetThreadName("Controller");

  control_param_init();
  while(true)
  {
    float error = BALANCE_POS - g_IMU1.theta, output;

    if(abs(error) > CONTROL_FAIL)
      motor_stop();

    output = controller.Kp_s * error;
    if(output > 1.0f)
      output = 1.0f;
    else if(output < -1.0f)
      output = -1.0f;

    motors[0].input = output;
    motors[1].input = output;

    chThdSleepMilliseconds(50);
  }
}
*/
