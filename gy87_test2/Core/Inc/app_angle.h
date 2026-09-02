

#ifndef __APP_ANGLE_H__
#define __APP_ANGLE_H__


#define DT                  (0.01)

#include "main.h"


/* 向量 */
typedef struct _vector_t
{
    float x;
    float y;
    float z;
} vector_t;

/* 姿态角 */
typedef struct _attitude_t
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct _quaternion_t
{ //四元数
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;

typedef struct _imu_data_t
{
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;

    float Offset[6];
} imu_data_t;

extern attitude_t g_attitude;
extern imu_data_t g_imu_data;
extern quaternion_t NumQ;
extern vector_t Gravity, Acc, Gyro, AccGravity;

//函数声明
void get_attitude_angle(imu_data_t *p_imu, attitude_t *p_angle, float dt);
void reset_quaternion(void);


#endif /* __APP_ANGLE_H__ */
