#ifndef __GY_87__
#define __GY_87__

#include "stm32f0xx_hal.h"
#include "main.h"
//#include "delay_lib.h"
#include "app_angle.h"

//MPU 6050
#define MPU6050_ADDR        		0xD0	//register 8 bit, default #define MPU6050_ADDR 0x68
#define MPU9250_ID							0x68	//check WHO_AM_I

#define MPU_SAMPLE_RATE_REG			0x19
#define MPU_CFG_REG							0x1A
#define MPU_GYRO_CFG_REG				0x1B	//Gyroscope full scale
#define MPU_ACCEL_CFG_REG				0x1C	//Accelerate full scale
#define MPU_ACCEL_CFG_2_REG     0x1D

#define MPU_FIFO_EN_REG					0x23

#define MPU_INTBP_CFG_REG				0X37
#define MPU_INT_EN_REG					0x38

#define MPU_USER_CTRL_REG				0x6A
#define MPU_PWR_MGMT1_REG				0x6B	//Power management 1
#define MPU_PWR_MGMT2_REG				0x6C	//Power management 2

#define WHO_AM_I_REG 						0x75

#define MPU_ACCEL_XOUTH_REG		0X3B	
#define MPU_ACCEL_XOUTL_REG		0X3C	
#define MPU_ACCEL_YOUTH_REG		0X3D	
#define MPU_ACCEL_YOUTL_REG		0X3E	
#define MPU_ACCEL_ZOUTH_REG		0X3F	
#define MPU_ACCEL_ZOUTL_REG		0X40


#define MPU_GYRO_XOUTH_REG			0x43		
#define MPU_GYRO_XOUTL_REG			0x44	
#define MPU_GYRO_YOUTH_REG			0x45	
#define MPU_GYRO_YOUTL_REG			0x46	
#define MPU_GYRO_ZOUTH_REG			0x47	
#define MPU_GYRO_ZOUTL_REG			0x48	

//HMC5883L

#define HMC5883L_ADDRESS      (0x1E << 1)
#define HMC5883L_CONFIG_A     0x00
#define HMC5883L_CONFIG_B     0x01
#define HMC5883L_MODE         0x02
#define HMC5883L_OUT_X_H      0x03
#define HMC5883L_OUT_X_L      0x04
#define HMC5883L_OUT_Z_H      0x05
#define HMC5883L_OUT_Z_L      0x06
#define HMC5883L_OUT_Y_H      0x07
#define HMC5883L_OUT_Y_L      0x08
#define HMC5883L_STATUS       0x09
#define HMC5883L_IDA          0x0A  // should return 0x48
#define HMC5883L_IDB          0x0B  // should return 0x34
#define HMC5883L_IDC          0x0C  // should return 0x33


#define MEASUREMENT_CONTINUOUS 					0x00
#define MEASUREMENT_SINGLE_SHOT 				0x01
#define MEASUREMENT_IDLE 								0x03



typedef struct
{
	short accel[3];
	short gyro[3];
	short compass[3];
    float roll;
    float pitch;
    float yaw;
} mpu_data_t;

typedef struct{
		short XAxis;
    short YAxis;
    short ZAxis;
} MagnetometerScaled;

typedef struct{
		short XAxis;
    short YAxis;
    short ZAxis;
} MagnetometerRaw;

void MPU_Delay_ms(uint16_t time);
uint8_t MPU_Basic_Init(I2C_HandleTypeDef *I2Cx);
uint8_t Gy87_Init(I2C_HandleTypeDef *I2Cx);
uint8_t MPU_Get_Gyroscope(I2C_HandleTypeDef *I2Cx, short *gyro);
uint8_t MPU_Get_Accelerometer(I2C_HandleTypeDef *I2Cx, short *accel);
void Gy_87_Read_Data_Handle(I2C_HandleTypeDef *I2Cx);
float MPU_Get_Yaw_Now(void);

uint8_t HMC5883L_setScale(I2C_HandleTypeDef *I2Cx, float gauss);
uint8_t MPU_Get_Magnetometer(I2C_HandleTypeDef *I2Cx, short *mag);

#endif 
