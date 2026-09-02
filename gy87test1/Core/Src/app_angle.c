#include "app_angle.h"
#include "app_math.h"



#define squa(Sq) (((float)Sq) * ((float)Sq))
// Angular velocity converted to radians. This parameter corresponds to gyro 500 degrees per second 0.00026646f
const float Gyro_Gr = 0.00013323f * 2; 
vector_t Gravity, Acc, Gyro, AccGravity;

quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ, vecyZ, veczZ;
float wz_acc_tmp[2];

attitude_t g_attitude;
imu_data_t g_imu_data;


/* Calculate 1/sqrt(x) */
float q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));
    return y;
}


/* Reset the four elements */
void reset_quaternion(void)
{
    NumQ.q0 = 1.0;
    NumQ.q1 = 0.0;
    NumQ.q2 = 0.0;
    NumQ.q3 = 0.0;
}


/* Get four elements dt: 10MS */
void get_attitude_angle(imu_data_t *p_imu, attitude_t *p_angle, float dt)
{
    
    static vector_t GyroIntegError = {0};
    static float KpDef = 0.25;
    static float KiDef = 0.000001f;
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

		//quaternion -> gravity
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // Acceleration normalization,
    NormQuat = q_rsqrt(squa(p_imu->accX)+ squa(p_imu->accY) +squa(p_imu->accZ)); 

    //After normalization, it can be transformed into the downward component of the unit vector
    Acc.x = p_imu->accX * NormQuat;
    Acc.y = p_imu->accY * NormQuat;
    Acc.z = p_imu->accZ * NormQuat;

    //The value obtained by vector cross multiplication can be used to obtain the deviation of the gravity component of the rotation matrix on the new acceleration component.
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
		
		

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    //The angular velocity is integrated with the acceleration ratio compensation value, which together with the above three sentences forms the PI compensation to obtain the corrected angular velocity value.
    Gyro.x = p_imu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; //In radians, the drift of angular velocity is compensated here.
    Gyro.y = p_imu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = p_imu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // Update quaternion
    // Integrate the corrected angular velocity value to obtain the changes in the real part Q0 and the three imaginary parts Q1~3 of the quaternion in the two attitude calculations.
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    //积分后的值累加到上次的四元数中，即新的四元数
    NumQ.q0 += q0_t; 
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 重新四元数归一化，得到单位向量下
    NormQuat = q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //得到四元数的模长
    NumQ.q0 *= NormQuat;                                                               //模长更新四元数值
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* 计算姿态角 */
    vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3; /*矩阵(3,1)项*/                                 //地理坐标系下的X轴的重力分量
    vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1; /*矩阵(3,2)项*/                                 //地理坐标系下的Y轴的重力分量
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3; /*矩阵(3,3)项*/ //地理坐标系下的Z轴的重力分量

    
    p_angle->pitch = asin(vecxZ);             //俯仰角
    p_angle->roll = atan2f(vecyZ, veczZ);     //横滚角

    p_angle->yaw = atan2(2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3), 
        NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 - NumQ.q3 * NumQ.q3); //偏航角

}
