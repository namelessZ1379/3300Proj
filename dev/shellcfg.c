#include "main.h"

static MotorStruct* motors;

#define  FLASH_TEST_COUNT 8U
#define  FLASH_TEST_ADDR 0x08010000

const uint32_t flashTestData[FLASH_TEST_COUNT] =
{ 1, 1, 2, 3, 5, 8, 13, 21};


static void cmd_print(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc,argv;

  chprintf(chp,"Shell OK\r\n");
}

static void cmd_accelerate(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint16_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  float cmd = motors[0].input + (float)finalNum * 0.01f;
  if(cmd <= 0.9f)
  {
    motors[0].input = cmd;
    motors[1].input = cmd;
    chprintf(chp,"Motor Speed: %d\n", (int8_t)(cmd*100));
  }
}

static void cmd_deccelerate(BaseSequentialStream * chp, int argc, char *argv[]){
  (void) argc;

  char *toNumber = argv[0];
  uint16_t finalNum=0;
  while(*toNumber!='\0')
    finalNum=finalNum*10+*(toNumber++)-'0';

  float cmd = motors[0].input - (float)finalNum * 0.01f;
  if(cmd >= -0.9f)
  {
    motors[0].input = cmd;
    motors[1].input = cmd;
    chprintf(chp,"Motor Speed: %d\n", (int8_t)(cmd*100));
  }
}

static void cmd_flash(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  size_t i;
  for (i = 0; i < FLASH_TEST_COUNT; i++)
    flashWriteData(FLASH_TEST_ADDR + 4*i , flashTestData[i]);

  chprintf(chp,"Flash successful\n");
  chThdSleepMilliseconds(1000);
  chprintf(chp,"Reading flash data...\n");

  for (i = 0; i < FLASH_TEST_COUNT; i++)
    chprintf(chp,"Data1:%d\n",flashReadData(FLASH_TEST_ADDR + 4*i));
}

static const ShellCommand commands[] =
{

  {"print",cmd_print},
  {"a",cmd_accelerate},
  {"d" , cmd_deccelerate},
  {"flash",cmd_flash},
  {NULL, NULL}
};


static const ShellConfig shell_cfg1 =
{
  CHP,
  commands
};

static THD_WORKING_AREA(Shell_thread_wa, 1024);

void shellStart(void)
{
  sdStart(BLE, NULL);

  motors = getMotors();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}
