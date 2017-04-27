#include "L3G4200.h"
#include "delay.h"
#include "lcd.h"
#include "math.h"


short gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
float gyroDPS[3];                      // gyro degrees per second, each axis

float heading[3]= {0.0f, 0.0f, 0.0f};               // heading[x], heading[y], heading [z]

short gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
short gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.


void getGyroRaw(short* tmp_dat){
  for(int i=0;i<3;i++){
    tmp_dat[i] = gyroRaw[i];
  }
}

void getGyroDPS(float* tmp_dat){
  for(int i=0;i<3;i++){
    tmp_dat[i] = gyroDPS[i];
  }
}

//three-axis digital output gyroscope init
void L3G4200_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_ByteWrite(L3G4200_Addr, L3G4200_CTRL_REG1, 0x4F);       // 200Hz, Cutoff 12.5, enable 3 axis, and start
  I2C_ByteWrite(L3G4200_Addr, L3G4200_CTRL_REG3, 0x08);       // Enable control ready signal
  I2C_ByteWrite(L3G4200_Addr, L3G4200_CTRL_REG4, 0x80);       // block data update£¡ 250dps

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                   //KEY_1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void L3G4200_calibrateGyro() {
  long int gyroSums[3]= {0};
  long int gyroSigma[3]= {0};

  for (int i=0; i<NUM_GYRO_SAMPLES; i++) {
    L3G4200_updateGyroValues();
    for (int j=0; j<3; j++) {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0; j<3; j++) {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
    double tmp_1 = (double)gyroSigma[j] / NUM_GYRO_SAMPLES;
    double tmp_2 = (averageRate * averageRate) * GYRO_SIGMA_MULTIPLE;
    // Per STM docs, we store the average of the samples for each axis and subtract them when we use the data.
    gyroZeroRate[j]=averageRate;
    // Per STM docs, we create a threshold for each axis based on the standard deviation of the samples times 3.
    gyroThreshold[j]=(int)sqrt(tmp_1 - tmp_2);
  }
}

void L3G4200_testCalibration() {
  L3G4200_calibrateGyro();
  u8 zeroRate[10] = "ZeroRate:";
  u8 threshold[11] = "threshold:";
  u8 prompt[16] = "KEY_1--continue";
  int ZeroRate, Threshold;
  LCD_DrawString(2, 0, zeroRate, 9);
  LCD_DrawString(4, 0, threshold, 10);
  LCD_DrawString(6, 0, prompt, 15);
  for (int j=0; j<3; j++) {
    if(gyroZeroRate[j] < 0){
      ZeroRate = -gyroZeroRate[j];
      LCD_DrawChar(2, 88, '-');
    }else{
      ZeroRate = gyroZeroRate[j];
    }
    
    LCD_DrawChar(2, 120, HexValueOffset[ZeroRate%10]);
    ZeroRate /= 10;
    LCD_DrawChar(2, 112, HexValueOffset[ZeroRate%10]);
    ZeroRate /= 10;
    if(ZeroRate%10 != 0) {
      LCD_DrawChar(2, 104, HexValueOffset[ZeroRate%10]);
      ZeroRate /= 10;
      if(ZeroRate%10 != 0) {
        LCD_DrawChar(2, 96, HexValueOffset[ZeroRate%10]);
        ZeroRate /= 10;
      }
    }
    if(gyroThreshold[j] < 0){
      Threshold = -gyroThreshold[j];
      LCD_DrawChar(4, 88, '-');
    }else{
      Threshold = gyroThreshold[j];
    }
    LCD_DrawChar(4, 120, HexValueOffset[Threshold%10]);
    Threshold /= 10;
    LCD_DrawChar(4, 112, HexValueOffset[Threshold%10]);
    Threshold /= 10;
    if(ZeroRate%10 != 0) {
      LCD_DrawChar(4, 104, HexValueOffset[Threshold%10]);
      Threshold /= 10;
      if(ZeroRate%10 != 0) {
        LCD_DrawChar(4, 96, HexValueOffset[Threshold%10]);
      }
    }
//    while(!(GPIOA->IDR&GPIO_Pin_0));//wait KEY_1
//    while(GPIOA->IDR&GPIO_Pin_0);//wait KEY_1
  }

  LCD_ClearLine(1);
  LCD_ClearLine(2);
  LCD_ClearLine(3);
}

void L3G4200_updateGyroValues() {
  while ((!I2C_ByteRead(L3G4200_Addr, 0x27) & 0x08));     // XYZ data ready test
  u8 temp_L,temp_H;
  u8 reg=0x28;
  delay_ms(1);
  for (int j=0; j<3; j++) {
    temp_L = I2C_ByteRead(L3G4200_Addr, reg);
    temp_H = I2C_ByteRead(L3G4200_Addr, reg + 1);
    gyroRaw[j] = temp_L | temp_H << 8;
    reg+=2;
  }

  
  int deltaGyro[3];
  for (int j=0; j<3; j++) {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (fabs((double)deltaGyro[j]) < gyroThreshold[j]) {
      deltaGyro[j]=0;
    }
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
  
  
}

void L3G4200_updateHeadings() {
  int deltaT=getDeltaTMillis();
  if(deltaT > 300){
    deltaT = 0;   //adjustment
  }
  for (int j=0; j<3; j++) {
    heading[j] += (gyroDPS[j]*deltaT)/1000.0f;
  }
}

uint32_t getDeltaTMillis() {
  static uint32_t lastTime=0;
  unsigned long currentTime=millis();

  unsigned long deltaT=currentTime-lastTime;
//  if (deltaT < 0.0) {
//    deltaT=currentTime+(0xFFFFFFFF-lastTime);
//  }
  lastTime=currentTime;

  return deltaT;
}

void L3G4200_update() {
  L3G4200_updateGyroValues();
  L3G4200_updateHeadings();
}

void getHeading(float* GyroHeading) {
  for(int i=0; i<3; i++) {
    GyroHeading[i] = heading[i];
  }
}