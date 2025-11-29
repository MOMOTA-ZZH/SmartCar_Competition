/*
 * imu.c
 *
 *  Created on: 2024年10月19日
 *      Author: 554.
 */
#include "imu.h"

// 变量
//============================================姿态解算======================================================//

float my_gyro_x, my_gyro_y, my_gyro_z;
float my_acc_x, my_acc_y, my_acc_z;
LowPassFilter filter_gyro;
//============================================mahony滤波====================================================//

// volatile防止数据更改
float twoKp = twoKpDef;                                           // twoKp比例增益的两倍
float twoKi = twoKiDef;                                           // twoKi积分增益的两倍
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // 传感器坐标系相对于辅助坐标系的四元数
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 通过积分项计算得到的误差累积量
float half_error;                                                          // 误差的一半

// 函数

//============================================姿态解算======================================================//


//============================================mahony滤波====================================================//
float my_absf(float x)
{
    if (x < 0)
    {
        return -x;
    }
    else
    {
        return x;
    }
}

float invSqrt(float x) // 快速平方根倒数
{

    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;              // 归一化系数
    float halfvx, halfvy, halfvz; // 估计的重力方向和磁通的垂直向量的一半
    float halfex, halfey, halfez; // 误差的一半在x、y、z轴上的分量
    float qa, qb, qc;             // 四元数的临时变量

    static int count = 0; // 计数器
    // 动态参数调节
    /********************************参数规划************************************/
    if (count < 2000)
    {
        count += 2;
        twoKp = 20; // 比例增益的两倍设置为20
    }
    else
    {
        twoKp = 0.5; // 如果误差的一半乘以1000的绝对值大于50，将比例增益的两倍设置为0.1

    }
    /***************************************************************************/

    // 仅在加速度计测量有效时计算反馈（避免在加速度计归一化时出现NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 加速度计测量归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向和磁通的垂直向量
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差是估计重力方向和测量重力方向的叉乘之和
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        half_error = sqrtf(halfex * halfex + halfey * halfey + halfez * halfez);

        // 如果开启了积分反馈，计算并应用积分反馈
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // 积分误差乘以Ki再乘以采样周期的比例
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // 应用积分反馈
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // 防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 对四元数的变化率进行积分
    gx *= (0.5f * (1.0f / sampleFreq));
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 计算姿态角
    
    Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll                                    // 俯仰角
    Pitch =  Pitch6;
    Yaw   =  Yaw6;
}
//============================================imu进程====================================================//


// 初始化滤波器
void init_lowpass_filter(LowPassFilter *filter, float cutoff_freq, float sample_rate) {
    float RC = 1.0 / (2 * M_PI * cutoff_freq); // 时间常数
    float dt = 1.0 / sample_rate;             // 采样周期
    filter->alpha = dt / (RC + dt);            // 计算滤波器系数
    filter->prev_output = 0.0;                 // 初始输出值设为 0
}

// 实时低通滤波器
float apply_lowpass_filter(LowPassFilter *filter, float input) {
    // 计算当前输出值
    float output = filter->prev_output + filter->alpha * (input - filter->prev_output);
    // 更新滤波器状态
    filter->prev_output = output;
    return output;
}

void imu_process(void)
{

    if(gyro1[0]>0.1)gyro1[0]-=0.1;
    else if(gyro1[0]<-0.1) gyro1[0]+=0.1;
    else gyro1[0]=0;

    if(gyro1[1]>0.1)gyro1[1]-=0.1;
    else if(gyro1[1]<-0.1) gyro1[1]+=0.1;
    else gyro1[1]=0;

    if(gyro1[2]>0.1)gyro1[2]-=0.1;
    else if(gyro1[2]<-0.1) gyro1[2]+=0.1;
    else gyro1[2]=0;

    gyro6[0]=apply_lowpass_filter(&filter_gyro, gyro1[0]*ch100gain);

    MahonyupdateIMU(gyro1[0]*gyro_gain, gyro1[1]*gyro_gain, gyro1[2]*gyro_gain, acc[0], acc[1], acc[2]); // mahony姿态解算

}
