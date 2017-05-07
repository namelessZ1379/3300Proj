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

static Control_Param controller;
static BaseSequentialStream* chp = CHP;
static MotorStruct* motors;
static rc_channel_t* rc_channel;

void control_param_init(void)
{
  uint32_t parameters[PARAM_NUM];
  flashRead(CONTROL_PARAMETERS_FLASH, (char*)parameters, PARAM_NUM*4);

  controller.Kp_s = (float)(parameters[0]/10000.0f);
  controller.Kd_s = (float)(parameters[1]/10000.0f);
  //controller.Kp_v = (float)(parameters[2]/10000.0f);
  //controller.Ki_v = (float)(parameters[3]/10000.0f);
  //controller.int_max = (float)(parameters[4]);
}

static THD_WORKING_AREA(Controller_thread_wa, 4096);
static THD_FUNCTION(Controller_thread, p)
{
  (void)p;
  chRegSetThreadName("Controller");

  //float speed_cmd = 0.0f, speed_error, speed_error_int;
  //float speed_output;

  control_param_init();
  while(true)
  {
/*
    speed_error = (speed_cmd - motors[0].speed)/1000.0f;
    speed_error_int += speed_error;

    if(speed_error_int > controller.int_max)
      speed_error_int = controller.int_max;
    else if(speed_error_int < -controller.int_max)
      speed_error_int = -controller.int_max;

    speed_output = controller.Kp_v * speed_error +
          controller.Ki_v * speed_error_int;

  //  tft_printf(5,2,"Error:%6d",(int16_t)(speed_output * 10000.0f));
*/
    float error =  g_IMU1.theta - BALANCE_POS, output;

    if(error > CONTROL_FAIL || error < -CONTROL_FAIL)
    {
      motor_stop();
      palTogglePad(GPIOD, GPIOD_LED4);
    }

    error += ((float)rc_channel[2] - 1500.0f)/100.0f;
    //error += speed_output;

    output = controller.Kp_s * error -
             controller.Kd_s * g_IMU1.gyroFiltered[2] / 150.0f;

    float temp;

    if (motors[0].input > 0.0f)
      temp =  motors[0].input - DEAD_ZONE;
    else
      temp =  motors[0].input + DEAD_ZONE;

    temp += output;

    if(temp > 0.5f)
      temp = 0.5f;
    else if(temp < -0.5f)
      temp = -0.5f;


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

    float diff = (float)(rc_channel[1] - 1500.0f)/15000.0f;
    motors[0].input_diff = motors[0].input + diff;
    motors[1].input_diff = motors[1].input - diff;

    chThdSleepMilliseconds(20);
  }
}

static THD_WORKING_AREA(Test_thread_wa, 64);
static THD_FUNCTION(Test_thread, p)
{
  (void)p;
  chRegSetThreadName("Test");

  palSetPad(GPIOD,GPIOD_LED4);
  palSetPad(GPIOD,GPIOD_LED5);
  palClearPad(GPIOD,GPIOD_LED3);

  rc_channel = rc_init();
  uint8_t started = 0;

  while (true)
  {
    palTogglePad(GPIOD,GPIOD_LED6);
    if(!started && palReadPad(GPIOA , 0))
    {
      chThdCreateStatic(Controller_thread_wa, sizeof(Controller_thread_wa),
        NORMALPRIO + 2,
                        Controller_thread, NULL);

      started = 1;
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

  //uint32_t count = 0;
  while(true)
  {
    //if(count%80 < 8)
    tft_printf(5,1,"Theta:%6d",(int16_t)(g_IMU1.theta * 100.0f));
    tft_printf(5,5,"RC1:%5d", rc_channel[0]);
    tft_printf(5,6,"RC2:%5d", rc_channel[1]);
    tft_printf(5,7,"RC3:%5d", rc_channel[2]);
    tft_update();

    //count++;
    chThdSleepMilliseconds(100);
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
