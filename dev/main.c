/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

static BaseSequentialStream* chp = CHP;
static MotorStruct* motors;
static rc_channel_t* rc_channel;
static PIMUStruct pIMU;

static THD_WORKING_AREA(Test_thread_wa, 64);
static THD_FUNCTION(Test_thread, p)
{
  (void)p;
  chRegSetThreadName("Test");

  palSetPad(GPIOD,GPIOD_LED4);
  palSetPad(GPIOD,GPIOD_LED5);
  palClearPad(GPIOD,GPIOD_LED3);

  rc_channel = rc_init();

  while (true)
  {
    palTogglePad(GPIOD,GPIOD_LED6);
    chThdSleepMilliseconds(200);
  }
}

static THD_WORKING_AREA(MPU_thread_wa, 32768);
static THD_FUNCTION(MPU_thread, p)
{
  (void)p;
  chRegSetThreadName("MPU6050");

  pIMU = getIMU();
  if(MPU_Init())
    while(true)
    {
      palTogglePad(GPIOD,GPIOD_LED3);
      chThdSleepMilliseconds(200);
    }

  chThdSleepMilliseconds(1000);

  //pIMU->init_stat = mpu_dmp_init();

  while (true)
  {
    //if(mpu_dmp_get_data(pIMU->P_R_Y))
    //  palTogglePad(GPIOD,GPIOD_LED3);
    MPU_test();
    chThdSleepMilliseconds(250);
  }
}

static THD_WORKING_AREA(TFT_thread_wa, 4096);
static THD_FUNCTION(TFT_thread, p)
{
  (void)p;
  chRegSetThreadName("TFT Display");

  tft_init(1, CYAN,BLACK,BLACK);
  while(true)
  {
    tft_printf(5,1,"Stat:%2d",pIMU->init_stat);
    tft_printf(5,2,"P:%6d",(int)(pIMU->P_R_Y[0]*100));
    tft_printf(5,3,"R:%6d",(int)(pIMU->P_R_Y[1]*100));
    tft_printf(5,4,"Y:%6d",(int)(pIMU->P_R_Y[2]*100));
    //tft_printf(5,4,"Gyro:%6d",pIMU->Gyro[0]);
    tft_printf(5,5,"Accl:%6d",pIMU->Accel[0]);
    tft_printf(5,6,"Gyro:%6d",pIMU->Gyro[0]);
    tft_update();
    chThdSleepMilliseconds(50);
  }
}

static THD_WORKING_AREA(Motor_thread_wa, 4096);
static THD_FUNCTION(Motor_thread, p)
{
  (void)p;
  chRegSetThreadName("Motor Control");

  motors = getMotors();
  motor_init();
  while(true)
  {
    motor_pwmUpdate();
    chThdSleepMilliseconds(20);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellStart();

  chThdCreateStatic(Test_thread_wa, sizeof(Test_thread_wa),
  NORMALPRIO,
                    Test_thread, NULL);

  chThdCreateStatic(MPU_thread_wa, sizeof(MPU_thread_wa),
  NORMALPRIO + 5,
                    MPU_thread, NULL);

  chThdCreateStatic(TFT_thread_wa, sizeof(TFT_thread_wa),
  NORMALPRIO - 5,
                    TFT_thread, NULL);

  chThdCreateStatic(Motor_thread_wa, sizeof(Motor_thread_wa),
  NORMALPRIO + 1,
                    Motor_thread, NULL);

  while (true) {
    //chprintf(chp,"Serial OK\n");
    chThdSleepMilliseconds(500);
  }
  return 0;
}
