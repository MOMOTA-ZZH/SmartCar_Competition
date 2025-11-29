/*
 * pid_pro.c
 * pid算法
 *  Created on: 2024年11月18日
 *      Author: 554.
 */
#include <pid_pro.h>

PID_IncTypeDef roll_angle_ring,roll_anglespeed_ring,roll_speed_ring;
PID_IncTypeDef image_Error_wheel,lane_change;

//PID_Piecewise bdc_pid[3];//分段pid

int xjlun=1,dllun=1,yawh=1;//一些标志位，拼音

/**
  * @brief  PID参数初始化
  * @note   无
  * @retval 无
  */
void PID_param_init(void) {

    // 初始化平衡环
#if torque_ctl

#if CH100_ON
    PID_Inc_Init(&roll_speed_ring, -0.035f, -0.035, 0);//   15  8
    pid_Improvement_on(&roll_speed_ring,17,10,Trapezoid_Intergral|ChangingIntegralRate);//Trapezoid_Intergral|ChangingIntegralRate
    PID_Inc_Init(&roll_angle_ring,   48, 0, 50);//45
    pid_Improvement_on(&roll_angle_ring,0,0,Derivative_On_Measurement);//Trapezoid_Intergral ChangingIntegralRate
    PID_Inc_Init(&roll_anglespeed_ring, 0.083, 0, 0);  //动量轮 0.08
#else
    PID_Inc_Init(&roll_speed_ring, -0.035f, -0.035, 0);
    PID_Inc_Init(&roll_angle_ring,   45, 0, 50);//45
    PID_Inc_Init(&roll_anglespeed_ring, 0.08, 0, 0);  //动量轮 0.08
#endif

#else
    PID_Inc_Init(&roll_speed_ring, -0.035f, -0.035, 0);
    PID_Inc_Init(&roll_angle_ring,   80, 0, 80);//90
    PID_Inc_Init(&roll_anglespeed_ring, 0.6, 0, 0);  //动量轮 0.4
#endif

    PID_Inc_Init(&image_Error_wheel, 1.6, 0.0f, 0.6f);//摄像头偏差  1.5   0.5
    pid_Improvement_on(&image_Error_wheel,0,0,Derivative_On_Measurement);

    PID_Inc_Init(&lane_change, 4, 0.0f, 0.5f);//变道偏差

}

void PID_Change(void) // 切换参数(后期完善)
{
//    if (abs(encoder_bdc) < 100){
//        bdc_angle_wheel.Kp=bdc_pid[0].bdc_angle_wheel.Kp;
//        bdc_angle_wheel.Kd=bdc_pid[0].bdc_angle_wheel.Kd;
//        bdc_anglespeed_wheel.Kp=bdc_pid[0].bdc_anglespeed_wheel.Kp;
//        bdc_speed_wheel.Kp=bdc_pid[0].bdc_speed_wheel.Kp;
//    }
//    else if (abs(bdc_speed)<200){
//        bdc_angle_wheel.Kp=bdc_pid[1].bdc_angle_wheel.Kp;
//        bdc_angle_wheel.Kd=bdc_pid[1].bdc_angle_wheel.Kd;
//        bdc_anglespeed_wheel.Kp=bdc_pid[1].bdc_anglespeed_wheel.Kp;
//        bdc_speed_wheel.Kp=bdc_pid[1].bdc_speed_wheel.Kp;
//    }
//    else {
//        bdc_angle_wheel.Kp=bdc_pid[2].bdc_angle_wheel.Kp;
//        bdc_angle_wheel.Kd=bdc_pid[2].bdc_angle_wheel.Kd;
//        bdc_anglespeed_wheel.Kp=bdc_pid[2].bdc_anglespeed_wheel.Kp;
//        bdc_speed_wheel.Kp=bdc_pid[2].bdc_speed_wheel.Kp;
//    }

}


void PID_Inc_Init(PID_IncTypeDef* sptr, float kp, float ki, float kd) {
    sptr->Ek1 = 0;        //上次偏差值初始化
    sptr->Ek2 = 0;        //上上次偏差值初始化
    sptr->Kp = kp;                //比例常数
    sptr->Ki = ki;                //积分常数
    sptr->Kd = kd;                //微分常数
    sptr->OUT = 0;
    sptr->Last_Measure = 0;
    sptr->Improve = 0;
    sptr->ScalarA = 0;
    sptr->ScalarB = 0;
}

void pid_Improvement_on(PID_IncTypeDef* PID,float A,float B,uint8_t improve){
// 动态积分系数A   A-B之间按比例调整积分项
    PID->ScalarA =A;
    PID->ScalarB =B;
    PID->Improve =improve;
}

float Positional_PID(PID_IncTypeDef* PID, float SetValue, float ActualValue, float Max_I,float low_pass) {
    float ITerm=0;                        // 当前积分项
    PID->Ek = SetValue - ActualValue;
    PID->Ek = PID->Ek * (1-low_pass) + PID->Ek1 * low_pass; //低通滤波

    if(fabs(roll_angle_ring.Ek)<0.6 && fabs(pidout_3f)<0.8 && fabs(gyro6[0])<15 ){//0.6   && fabs(gyro6[0])<10
    if (PID->Improve & Trapezoid_Intergral)
        ITerm= (PID->Ek+PID->Ek1)/2;// 梯形积分计算
    else
        ITerm= PID->Ek;
    }

    if (PID->Improve & ChangingIntegralRate){ // 若增强选项包含动态积分率
        if (PID->Ek * PID->Ek2 > 0) { // 判断积分项和误差符号是否相同

            if (fabs(PID->Ek) <= PID->ScalarB)// 若误差小于动态积分系数B
                ITerm+=0;
            else if (fabs(PID->Ek) <= (PID->ScalarA + PID->ScalarB) && fabs(PID->Ek) >= PID->ScalarB)
                ITerm *= (PID->ScalarA - fabs(PID->Ek) + PID->ScalarB) / PID->ScalarA;
            else
                ITerm = 0; // 积分项置零
        }
    }

    PID->Ek2+=ITerm;

    if (PID->Ek2 > Max_I)  PID->Ek2 = Max_I;
    if (PID->Ek2 < -Max_I) PID->Ek2 = -Max_I;

    float PIDInc;

    if (PID->Improve & Derivative_On_Measurement)// 进行测量值微分处理 微分先行
        PIDInc = (PID->Kp * PID->Ek) +(PID->Ki * PID->Ek2 *0.01) +(PID->Kd * (PID->Last_Measure - ActualValue));
    else
        PIDInc = (PID->Kp * PID->Ek) +(PID->Ki * PID->Ek2 *0.01) +(PID->Kd * (PID->Ek - PID->Ek1));

    PID->Ek1 = PID->Ek;
    PID->Last_Measure = ActualValue;            // 更新上次测量值

    if(go_flag == 0 || go_flag == 1){
    PID->Ek2 = 0;
    PID->Ek1 = 0;
    PID->Last_Measure=0;
    }
    return PIDInc;
}


//pid输出清零函数
void PID_clear(PID_IncTypeDef* sptr) {
    sptr->Ek1 = 0;        //上次偏差值初始化
    sptr->Ek2 = 0;        //上上次偏差值初始化
    //    sptr->OUT = 0;
}


/*!
  * @brief    限幅函数
  *
  * @param    val   ： 参数
  * @param    lower   ： 最低值
  * @param    upper  ： 最高值
  *
  */
//限幅
float limit(float val, float lower, float upper) {
    return val < lower ? lower : (val > upper ? upper : val);
}
