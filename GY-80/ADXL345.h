#ifndef __ADXL345_H
#define __ADXL345_H

#include "stm32f10x.h"
#include "lcd.h"
#include "i2c_ee.h" 
#include "delay.h"

#define ADXL345_Addr	0xA6  //Digital Accelerometer
const static float mps2PerDigit=0.00390625f;      

void ADXL345_Init(void);
void ADXL345_setOffset(int8_t x, int8_t y, int8_t z);
void ADXL345_updateAcceleration(void);
void getAccel(float* tmp_dat);  //m/s^2











#endif