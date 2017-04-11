#ifndef _3300MISC_H_
#define _3300MISC_H_

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#define BLE &SD4
#define CHP (BaseSequentialStream*)BLE

void shellStart(void);

#endif
