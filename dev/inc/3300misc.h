#ifndef _3300MISC_H_
#define _3300MISC_H_

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"


#define BLE &SD4
#define BLE2 &SD2
#define CHP (BaseSequentialStream*)BLE
#define CHP2 (BaseSequentialStream*)BLE2

void shellStart(void);

#endif
