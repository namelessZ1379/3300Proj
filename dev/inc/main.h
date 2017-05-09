#ifndef _MAIN_H_
#define _MAIN_H_

#include "tft_display.h"
#include "motor.h"
#include "rc.h"
#include "flash.h"
#include "control.h"

#include "mpu6050.h"

void cmd_control_start(BaseSequentialStream * chp, int argc, char *argv[]);

#endif
