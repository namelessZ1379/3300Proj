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

static rc_channel_t* rc_channel;

static THD_WORKING_AREA(Test_thread_wa, 64);
static THD_FUNCTION(Test_thread, p)
{
  (void)p;
  chRegSetThreadName("Test");

  palSetPad(GPIOD,GPIOD_LED4);
  palSetPad(GPIOD,GPIOD_LED5);
  palClearPad(GPIOD,GPIOD_LED3);

  rc_channel = rc_getChannels();

  while (true)
  {
    palTogglePad(GPIOD,GPIOD_LED6);

    if(palReadPad(GPIOA,GPIOA_BUTTON))
    {
      cmd_control_start(CHP, 0, NULL);

      palTogglePad(GPIOD,GPIOD_LED4);
      chThdSleepMilliseconds(1000);
      palTogglePad(GPIOD,GPIOD_LED4);
    }

    chThdSleepMilliseconds(200);
  }
}

static THD_WORKING_AREA(MPU_thread_wa, 32768);
static THD_FUNCTION(MPU_thread, p)
{
  (void)p;
  chRegSetThreadName("MPU6050");
  chThdSleepMilliseconds(200);

  if(!mpu6050Init(MPU6050_I2C_ADDR_A0_LOW))
    while(true)
    {
      palTogglePad(GPIOD,GPIOD_LED3);
      chThdSleepMilliseconds(200);
    }

  imuStructureInit(&g_IMU1, false);

  uint32_t tick = chVTGetSystemTimeX();
  uint32_t count = 0;
  while (true)
  {
    tick+=US2ST(MPU_COUNT);

    if(tick>chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
      tick=chVTGetSystemTimeX();

    mpu6050update(&g_IMU1);

    if(count % 8 == 7)
      mpu_calc_theta(&g_IMU1);

    count++;
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
    tft_printf(5,1,"Theta:%6d",(int16_t)(g_IMU1.theta * 100.0f));
    tft_printf(5,5,"RC1:%5d", rc_channel[0]);
    tft_printf(5,6,"RC2:%5d", rc_channel[1]);
    tft_printf(5,7,"RC3:%5d", rc_channel[2]);
    tft_update();

    chThdSleepMilliseconds(100);
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

  while (true) {
    //chprintf(chp,"Serial OK\n");
    chThdSleepMilliseconds(500);
  }
  return 0;
}
