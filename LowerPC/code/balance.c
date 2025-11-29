/*
 * balance.c
 * 车辆平衡控制相关
 *  Created on: 2024年1月18日
 *      Author: 554.
 */
#include "balance.h"
float Dynamic_zero_roll;
float eulerAngle_roll_max=0, eulerAngle_roll_min, eulerAngle_pitch_max, eulerAngle_pitch_min = 0;
float BLDC_YAW_ANGLE_Value=0,set_yaw=0;
int8_t image=0;
float ROLL_SPEED_I=2200;
float Z_Coefficient=0.012,Y_Coefficient=0.012;

//左右平衡
float ROLL_ANGLESPEED_LOOP(float value) {
    float SetValue = value;
#if CH100_ON
    float ActualValue = gyro6[0];
#else
    float ActualValue = -gyro[0];
#endif
    float out = Positional_PID(&roll_anglespeed_ring, SetValue, ActualValue, 10, 0.7);//0.7
    return out;
}

#if CH100_ON
    float Zero_roll=0.0f;//5.2
#else
    float Zero_roll=1.4f;//5.2
#endif

float ZHONGZHI_roll=0;
float ROLL_ANGLE_LOOP(float Set_Roll) {
    float SetValue = Set_Roll;
    //动态零点
    float ActualValue = Roll - ZHONGZHI_roll- Dynamic_zero_roll;//
    float out = Positional_PID(&roll_angle_ring, SetValue, ActualValue, 10, 0.7);//0.7

    return out;
}

float ROLL_SPEED_LOOP(float value) {
    float SetValue = value;
    float ActualValue = encoder_bldc;
    float out = Positional_PID(&roll_speed_ring, SetValue, ActualValue,ROLL_SPEED_I, 0);
    return out;
}

//动态零点计算

float low_pass1 = 0.25;//上一次数据占比
float Dynamic_zero_cale(void) {
    float zero_set=0,zero_get;
    if (BLDC_YAW_ANGLE_Value != 0 )
    {
        if(BLDC_YAW_ANGLE_Value>0){
           //压弯
             zero_get = BLDC_YAW_ANGLE_Value * Z_Coefficient;

        }
        else {
            //压弯
            zero_get = BLDC_YAW_ANGLE_Value * Y_Coefficient;

        }

        zero_set= zero_set * low_pass1 + zero_get * (1 - low_pass1);

        if(bdc_speed<0) zero_set=-zero_set;
    }
    else {
        return zero_set = 0;
    }
    return zero_set;
}


//姿态是否稳定判断函数
//0稳定，1不稳定
uint8 Attitude_stability_judgment(float roll, float pitch) {
    uint8 flag_pose = 0;//0表示稳定，1表示不稳定，车体姿态标志位
    if (eulerAngle_roll_min > roll)
        eulerAngle_roll_min = roll;
    else if (eulerAngle_roll_max < roll)
        eulerAngle_roll_max = roll;
    if (eulerAngle_pitch_min > pitch)
        eulerAngle_pitch_min = pitch;
    else if (eulerAngle_pitch_max < pitch)
        eulerAngle_pitch_max = roll;
    //姿态不稳定，输出标志位，取消转向环
    if ((eulerAngle_pitch_max - eulerAngle_pitch_min) > 14 || (eulerAngle_roll_max - eulerAngle_roll_min) > 4.2) {
        flag_pose = 1;//不稳定
    }
    else if ((eulerAngle_pitch_max - eulerAngle_pitch_min) < 9 && (eulerAngle_roll_max - eulerAngle_roll_min) < 3)
        flag_pose = 0;//稳定
    else if ((eulerAngle_pitch_max - eulerAngle_pitch_min) > 14 || (eulerAngle_roll_max - eulerAngle_roll_min) > 5)
        flag_pose = 2;//极度不稳定
    eulerAngle_roll_max = -ZHONGZHI_roll +  Dynamic_zero_roll;
    eulerAngle_roll_min = -ZHONGZHI_roll + Dynamic_zero_roll;
    return flag_pose;
}

