#include "mpu6050.h"

#define MPU_I2C &I2CD1

#define MPU6050_I2C_ADDR_A0_LOW   0x68 //todo:modify
#define MPU6050_I2C_ADDR_A0_HIGH  0x69

#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x10
#define MPU_ERROR			0xff
/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

#define MPU6050_I2C_ADDR_USER MPU6050_I2C_ADDR_A0_LOW


static uint8_t mpu6050TXData[MPU6050_RX_BUF_SIZE];

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    200000,
    FAST_DUTY_CYCLE_2
};

static IMUStruct IMU =
{
  10,
  {0,0,0},
  {0,0,0},
  {0.0f,0.0f,0.0f}
};

uint8_t MPU_Init(void)
{
	uint8_t res;
	MPU_i2c_init();

	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050

  chThdSleepMilliseconds(100);

	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(100);						//���ò�����50Hz
	//MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);

	if(res==MPU6050_I2C_ADDR_USER)//����ID��ȷ
	{
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(100);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}

void MPU_i2c_init(void)
{
  i2cStart(MPU_I2C, &i2cfg);
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 *//*
uint8_t MPU_Init(void)
{
  msg_t status = MSG_OK;

  MPU_i2c_init();

  // Reset all MPU6050 registers to their default values
  mpu6050TXData[0] = MPU_PWR_MGMT1_REG;  // Start register address;
  mpu6050TXData[1] = 0xC0;          // Register value 0b11000000

  i2cAcquireBus(MPU_I2C);

  status = i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK)
    return 0;

  // Wait 100 ms for the MPU6050 to reset
  chThdSleepMilliseconds(100);

  // Clear the SLEEP flag, set the clock and start measuring.
  mpu6050TXData[0] = MPU_PWR_MGMT1_REG;  // Start register address;
  mpu6050TXData[1] = 0x03;         // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK)
    return 0;

  mpu6050TXData[0] = MPU_SAMPLE_RATE_REG;  // Start register address;
  mpu6050TXData[1] = 11;                  // SAMPLE_RATE_REG register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0x00;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);

  //MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g

  status = i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));


  i2cReleaseBus(MPU_I2C);

  if (status != MSG_OK)
    return 0;

  return 1;
}*/

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_I2C_ADDR_USER,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z����ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,��������
uint8_t MPU_Get_Gyroscope(int16_t* Gyro)
{
  uint8_t buf[6],res;
	res=MPU_Read_Len(MPU6050_I2C_ADDR_USER,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		Gyro[0]=(int16_t)((buf[0]<<8)|buf[1]);
		Gyro[1]=(int16_t)((buf[2]<<8)|buf[3]);
		Gyro[2]=(int16_t)((buf[4]<<8)|buf[5]);
	}
  return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z����ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,��������
uint8_t MPU_Get_Accelerometer(int16_t* Accel)
{
  uint8_t buf[6],res;
	res=MPU_Read_Len(MPU6050_I2C_ADDR_USER,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		Accel[0]=(int16_t)((buf[0]<<8)|buf[1]);
		Accel[1]=(int16_t)((buf[2]<<8)|buf[3]);
		Accel[2]=(int16_t)((buf[4]<<8)|buf[5]);
	}
  return res;
}
//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,��������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;
  i2cAcquireBus(MPU_I2C);

  uint8_t txbuf[len + 1];
  txbuf[0] = reg;

  for (i = 0; i < len; i++)
    txbuf[i + 1] = buf[i];

  if(i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, txbuf, len + 1,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS)) != MSG_OK)
  {
    i2cReleaseBus(MPU_I2C);
    return 1;
  }

  i2cReleaseBus(MPU_I2C);
	return 0;
}

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,��������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
 	i2cAcquireBus(MPU_I2C);

	uint8_t txbuf = reg;

	if(i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, &txbuf, 1,
    buf, len, MS2ST(MPU6050_READ_TIMEOUT_MS)) != MSG_OK)	//�ȴ�Ӧ��
	{
		i2cReleaseBus(MPU_I2C);
		return 1;
	}
  i2cReleaseBus(MPU_I2C);	//����һ��ֹͣ����
	return 0;
}
//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,��������
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	uint8_t txbuf[2] = {reg,data};

	i2cAcquireBus(MPU_I2C);

	if(i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, txbuf, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS)))	//�ȴ�Ӧ��
	{
		i2cReleaseBus(MPU_I2C);
		return 1;
	}

	i2cReleaseBus(MPU_I2C);
	return 0;
}
//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
  i2cAcquireBus(MPU_I2C);

	uint8_t txbuf = reg;

	if(i2cMasterTransmitTimeout(MPU_I2C, MPU6050_I2C_ADDR_USER, &txbuf, 1,
    &res, 1, MS2ST(MPU6050_READ_TIMEOUT_MS)))	//�ȴ�Ӧ��
	{
		i2cReleaseBus(MPU_I2C);
		return MPU_ERROR;
	}

	i2cReleaseBus(MPU_I2C);			//����һ��ֹͣ����
	return res;
}

PIMUStruct getIMU(void)
{
  return &IMU;
}

uint8_t MPU_test(void)
{
  uint8_t result;
  result = MPU_Get_Accelerometer(IMU.Accel);
  result = MPU_Get_Gyroscope(IMU.Gyro);

  return result;
}
