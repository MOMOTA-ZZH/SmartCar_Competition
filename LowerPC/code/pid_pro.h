/*
 * pid_pro.h
 *
 *  Created on: 2024年11月18日
 *      Author: 554.
 */
#ifndef CODE_PID_PRO_H_
#define CODE_PID_PRO_H_
#include "zf_common_headfile.h"

#define pid           (0)
#define torque_ctl    (1)

typedef struct {
    float Kp;                       //比例系数Proportional
    float Ki;                       //积分系数Integral
    float Kd;                       //微分系数Derivative
    float Ek;                       //当前误差
    float Ek1;                      //前一次误差 e(k-1)
    float Ek2;                      //累计误差
    float Last_Measure;             // 上一次测量值
    float OUT;                      //PID输出
    uint8_t Improve;                    // 增强选项的标志位
    float ScalarA;                      // 动态积分调整系数A
    float ScalarB;                      // 动态积分调整系数B
} PID_IncTypeDef;

// PID 增强功能枚举定义，便于选择不同的控制策略
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        // 无增强功能
    Derivative_On_Measurement = 0x01,   // 基于测量值的微分项  微分先行
    Trapezoid_Intergral = 0x02,         // 梯形积分
    ChangingIntegralRate = 0x04,        // 动态积分调整
} PID_Improvement_e;

//typedef struct {
//    PID_IncTypeDef bdc_angle_wheel;
//    PID_IncTypeDef bdc_speed_wheel;
//    PID_IncTypeDef bdc_anglespeed_wheel;
//} PID_Piecewise;
//
//extern PID_Piecewise bdc_pid[3];

extern PID_IncTypeDef roll_angle_ring, roll_anglespeed_ring, roll_speed_ring;
extern PID_IncTypeDef image_Error_wheel,lane_change;

extern int xjlun,dllun,yawh;

void PID_Inc_Init(PID_IncTypeDef* sptr, float kp, float ki, float kd);
float Positional_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue, float Max_I,float low_pass);
void PID_param_init(void);
void pid_Improvement_on(PID_IncTypeDef* PID,float A,float B,uint8_t improve);

void PID_Change(void);
void PID_clear(PID_IncTypeDef* sptr);
float limit(float val, float lower, float upper);

#endif
