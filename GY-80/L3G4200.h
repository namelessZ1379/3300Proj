#ifndef __L3G4200_H
#define __L3G4200_H

#include "stm32f10x.h"
#include "lcd.h"
#include "i2c_ee.h" 
#include "delay.h"

#define L3G4200_Addr	0xD2  //three-axis digital output gyroscope

#define  L3G4200_CTRL_REG1  0x20
#define  L3G4200_CTRL_REG2  0x21
#define  L3G4200_CTRL_REG3  0x22
#define  L3G4200_CTRL_REG4  0x23
#define  L3G4200_CTRL_REG5  0x24
#define  L3G4200_CTRL_REG6  0x25
#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  3         // As recommended 

const static float dpsPerDigit=0.00875f;              // for conversion to degrees per second, Sensitivity 250dps

void getGyroRaw(short* tmp_dat);      //debug
void getGyroDPS(float* tmp_dat);     //debug
void getHeading(float* heading);
void L3G4200_Init(void);
void L3G4200_testCalibration(void);
void L3G4200_update();
void L3G4200_updateGyroValues(void);
void L3G4200_updateHeadings(void);
void L3G4200_calibrateGyro(void);
uint32_t getDeltaTMillis(void);








#endif