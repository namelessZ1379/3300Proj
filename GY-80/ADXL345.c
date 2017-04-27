#include "ADXL345.h"

short accelRaw[3];    //For debug use only
float accelmps2[3] = {0.0f, 0.0f, 0.0f};

//Digital Accelerometer Init
void ADXL345_Init(void) {
  //user-set offset adjustments in twos complement format
  
  ADXL345_setOffset(0, 0, 0);              //for debug only

  I2C_ByteWrite(ADXL345_Addr, 0x2D, 0x00); // normal mode, standby
  I2C_ByteWrite(ADXL345_Addr, 0x31, 0x0B); // full resolution, +/- 16G, 13-bit mode
  I2C_ByteWrite(ADXL345_Addr, 0x38, 0x00); // bypass mode
  I2C_ByteWrite(ADXL345_Addr, 0x2C, 0x0A); // 100Hz
  I2C_ByteWrite(ADXL345_Addr, 0x2D, 0x08); // start measure
}

void ADXL345_setOffset(int8_t x, int8_t y, int8_t z){
  I2C_ByteWrite(ADXL345_Addr, 0x1E, x); // OFSX
  I2C_ByteWrite(ADXL345_Addr, 0x1F, y); // OFSY
  I2C_ByteWrite(ADXL345_Addr, 0x20, z); // OFSZ
}

void ADXL345_updateAcceleration(void){
  u8 temp_L,temp_H;
  u8 reg=0x32;
  for (int j=0; j<3; j++) {
    temp_L = I2C_ByteRead(ADXL345_Addr, reg);
    temp_H = I2C_ByteRead(ADXL345_Addr, reg + 1);
    accelRaw[j] = temp_L | temp_H << 8;
    accelmps2[j] = accelRaw[j] * mps2PerDigit;
    reg+=2;
  }
}

void getAccel(float* tmp_dat){
  for(int i=0;i<3;i++){
    tmp_dat[i] = accelmps2[i];
  }
}