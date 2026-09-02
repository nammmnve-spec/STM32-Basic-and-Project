#include "Gy_87.h"
#include <math.h>
#include "main.h"


#define CONTROL_DELAY           10
#define DELAY_I2C								100

uint8_t res;
uint8_t res_2;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t g_system_init;


int Deviation_Count = 0;
mpu_data_t mpu_data;
short Deviation_gyro[3], Original_gyro[3];

mpu_data_t mpu_data;

short raw_mag[3], scaled_mag[3];
float m_Scale;

void MPU_Delay_ms(uint16_t time)
{
	if (g_system_init == 0)
	{
		delay_ms(time);
	}
	else
	{
		App_Delay_ms(time);
	}
}

uint8_t MPU_Basic_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;
	uint8_t Rate;
	
	
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &res, 1, DELAY_I2C);

	if (res == MPU9250_ID) // The device ID is correct 
	{	
		//PLL with X axis gyroscope reference 
		MPU_Delay_ms(100);
		Data = 0x01;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_PWR_MGMT1_REG, 1, &Data, 1, DELAY_I2C); 

		// Acceleration and gyroscope both work 
		MPU_Delay_ms(100);
		Data = 0X00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_PWR_MGMT2_REG, 1, &Data, 1, DELAY_I2C);
		
			//Set Gyroscope full scale register
		MPU_Delay_ms(100);
		Data = 0x08; //Set the third bit
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_GYRO_CFG_REG, 1, &Data, 1, DELAY_I2C);   // Gyroscope sensor   //�500dps=�500�/s �32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
		
		//Set Accelerate full scale register
		MPU_Delay_ms(100);
		Data = 0x00; 
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_ACCEL_CFG_REG, 1, &Data, 1, DELAY_I2C);	// Acceleration sensor //�2g=�2*9.8m/s^2 �32768 accel/32768*19.6=accel/1671.84
		
		//Set the Low-pass-filter and sampling rate of GY-87
		//sampling rate = 2 x LPF rate (Nyquist-Shannon theory) 
		//sampling rate = 50 -> max of LPF is 25
		MPU_Delay_ms(100);
		Rate = 50; // sampling rate of GY-87
		//Set the LPF
		Data = 0x04; //accelerate LPF = 21, gyro LPF = 20
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_CFG_REG, 1, &Data, 1, DELAY_I2C);
		//Set the sampling rate
		MPU_Delay_ms(100);
		Data = 1000/ Rate - 1;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_SAMPLE_RATE_REG, 1, &Data, 1, DELAY_I2C);
		
		// Turn off all interrupts
		MPU_Delay_ms(100);
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_INT_EN_REG, 1, &Data, 1, DELAY_I2C);
		
		// The I2C main mode is off 
		MPU_Delay_ms(100);
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_USER_CTRL_REG, 1, &Data, 1, DELAY_I2C);
		
		// Close the FIFO 
		MPU_Delay_ms(100);
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_FIFO_EN_REG, 1, &Data, 1, DELAY_I2C);
		
		// The INT pin is low, enabling bypass mode to read the magnetometer directly
		MPU_Delay_ms(100);
		Data = 0X82;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_INTBP_CFG_REG, 1, &Data, 1, DELAY_I2C);
		
	}
	else
		return 1;
	
	//Init HMC5883L
	HAL_I2C_Mem_Read(&hi2c2, HMC5883L_ADDRESS, HMC5883L_IDA, 1, &res, 1, DELAY_I2C);
	if (res == 0x48)
	{
		HAL_I2C_Mem_Read(&hi2c2, HMC5883L_ADDRESS, HMC5883L_IDB, 1, &res, 1, DELAY_I2C);
	}
	else
		return 2;
	
	if (res == 0x34)
	{
		HAL_I2C_Mem_Read(&hi2c2, HMC5883L_ADDRESS, HMC5883L_IDC, 1, &res, 1, DELAY_I2C);
	}
	else
		return 3;
	
	if (res == 0x33)
	{
		//Set average sampler = 4
		Data = 0X50;
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 1, &Data, 1, DELAY_I2C);
		
		//"Setting scale to +/- 1.3 Ga"
		HMC5883L_setScale(I2Cx, 1.3);

		//"Setting measurement mode to continous."
		Data = MEASUREMENT_CONTINUOUS; //0x00
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS, HMC5883L_MODE, 1, &Data, 1, DELAY_I2C);
	}
	else
		return 3;
	
	return 0;
}

uint8_t HMC5883L_setScale(I2C_HandleTypeDef *I2Cx, float gauss) {
    uint8_t regValue = 0x00;

    /*  Some of these values; e.g. 1.3 - cause comparison
        issues with the compiler that the Arduino IDE uses.
    */
	#define CLOSEENOUGH(x,y) (fabs(x-y)<0.001)

    if (CLOSEENOUGH(gauss, 0.88)) {
        regValue = 0x00;
        m_Scale = 0.73;
    } else if (CLOSEENOUGH(gauss, 1.3)) {
        regValue = 0x01;
        m_Scale = 0.92;
    } else if (CLOSEENOUGH(gauss, 1.9)) {
        regValue = 0x02;
        m_Scale = 1.22;
    } else if (CLOSEENOUGH(gauss, 2.5)) {
        regValue = 0x03;
        m_Scale = 1.52;
    } else if (CLOSEENOUGH(gauss, 4.0)) {
        regValue = 0x04;
        m_Scale = 2.27;
    } else if (CLOSEENOUGH(gauss, 4.7)) {
        regValue = 0x05;
        m_Scale = 2.56;
    } else if (CLOSEENOUGH(gauss, 5.6)) {
        regValue = 0x06;
        m_Scale = 3.03;
    } else if (CLOSEENOUGH(gauss, 8.1)) {
        regValue = 0x07;
        m_Scale = 4.35;
    } else {
        return 4;
    }

    // Setting is in the top 3 bits of the register.
    regValue = regValue << 5;
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 1, &regValue, 1, DELAY_I2C);
		
		return 0;
}


uint8_t Gy87_Init(I2C_HandleTypeDef *I2Cx)
{
	
	uint8_t count = 0;
	res_2 = MPU_Basic_Init(I2Cx);
	while (1)
	{
		if (res_2 == 0)
		{
			while (Deviation_Count < CONTROL_DELAY)
			{
				MPU_Get_Gyroscope(I2Cx, mpu_data.gyro);
				MPU_Delay_ms(100);
			}
			break;
		}
		else
		{
			count++;
			if (count > 5)
			{
				return 1;
			}
		}
		MPU_Delay_ms(200);
	}
	return 0;
}

uint8_t MPU_Get_Gyroscope(I2C_HandleTypeDef *I2Cx, short *gyro)
{
	uint8_t buf[6];
	HAL_StatusTypeDef status;
	//res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU_GYRO_XOUTH_REG, 1, buf, 6, DELAY_I2C);
	if(status==HAL_OK)
	{
		//
		if(Deviation_Count<CONTROL_DELAY)
		{
			Deviation_Count++;
			//Read the gyroscope zero 
			Deviation_gyro[0] = (((uint16_t)buf[0]<<8)|buf[1]);  
			Deviation_gyro[1] = (((uint16_t)buf[2]<<8)|buf[3]);  
			Deviation_gyro[2] = (((uint16_t)buf[4]<<8)|buf[5]);	
		}
		else
		{
			
			//Save the raw data
			Original_gyro[0] = (((uint16_t)buf[0]<<8)|buf[1]);
			Original_gyro[1] = (((uint16_t)buf[2]<<8)|buf[3]);
			Original_gyro[2] = (((uint16_t)buf[4]<<8)|buf[5]);
			
			//Removes zero drift data
			gyro[0] = Original_gyro[0]-Deviation_gyro[0];
			gyro[1] = Original_gyro[1]-Deviation_gyro[1];
			gyro[2] = Original_gyro[2]-Deviation_gyro[2];
		}
	}
	return 0;
}

uint8_t MPU_Get_Accelerometer(I2C_HandleTypeDef *I2Cx, short *accel)
{
	uint8_t buf[6];
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU_ACCEL_XOUTH_REG, 1, buf, 6, DELAY_I2C);
	if (status == 0)
	{
		accel[0] =((uint16_t)buf[0]<<8)|buf[1];
		accel[1] =((uint16_t)buf[2]<<8)|buf[3];
		accel[2] =((uint16_t)buf[4]<<8)|buf[5];
	}
	return status;
}

uint8_t MPU_Get_Magnetometer(I2C_HandleTypeDef *I2Cx, short *mag)
{
	uint8_t buf[6];
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(I2Cx, HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 1, buf, 6, DELAY_I2C);
	if (status == 0)
	{
		mag[0] =(((uint16_t)buf[0]<<8)|buf[1]) * m_Scale;
		mag[1] =(((uint16_t)buf[2]<<8)|buf[3]) * m_Scale;
		mag[2] =(((uint16_t)buf[4]<<8)|buf[5]) * m_Scale;
	}
	return status;
}


void Gy_87_Read_Data_Handle(I2C_HandleTypeDef *I2Cx)
{
	if (Deviation_Count >= CONTROL_DELAY)
	{
		MPU_Get_Gyroscope(I2Cx, mpu_data.gyro);
		MPU_Get_Accelerometer(I2Cx, mpu_data.accel);
		MPU_Get_Magnetometer(I2Cx, mpu_data.compass);
		
		g_imu_data.accX = mpu_data.accel[0];
		g_imu_data.accY = mpu_data.accel[1];
		g_imu_data.accZ = mpu_data.accel[2];
		g_imu_data.gyroX = mpu_data.gyro[0];
		g_imu_data.gyroY = mpu_data.gyro[1];
		g_imu_data.gyroZ = mpu_data.gyro[2];
		
		//get_attitude_angle(&g_imu_data, &g_attitude, DT);
	}
	else
	{
		MPU_Get_Gyroscope(I2Cx, mpu_data.gyro);
		MPU_Get_Accelerometer(I2Cx, mpu_data.accel);
		MPU_Get_Magnetometer(I2Cx, mpu_data.compass);
	}
}

// Get the current yaw angle
float MPU_Get_Yaw_Now(void)
{
	return mpu_data.yaw;           // ????
	// return mpu_data.yaw*RtA;    // ????
}


