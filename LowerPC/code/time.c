/*
 * time.c
 * 中断定时器用
 *  Created on: 2024年1月16日
 *      Author: 554.
 */
#include "time.h"
Timer_0 Flag;

float pidout_1f = 0, pidout_2f = 0, pidout_3f = 0,//动量轮pid输出值
pidout_1a = 0, pidout_2a = 0, pidout_3a = 0;//行进轮pid输出值

float increase_xj=0.05,increase_dj=0.45,limit_yaw=40;// xj 0.1
int i=0,y=0,o=0;//标志位
//float V_mediator=0;//
int increasing = 0;//增加标志位
int decreasing = 0;//减小标志位

void Fuse_result(void) {

#if odrive_encoder2

    // 30ms控制
     if (Flag.T_30ms == 1) {
         odrive_feedback(1);
         Flag.T_30ms = 0;
     }

#endif

    // 20ms控制  速度环
    if (Flag.T_20ms == 1) {
        odrive_feedback(0);

        if ((go_flag == 2 || go_flag == 3 )&& xjlun==1){//缓慢加速

        if((cross_flag<1) && flag_state==1) {
            cross_flag=1;
            direct_bdc_speed(0);
            beep_flag=1;
            i=0;
        }

        if(bdc_speed_max==0){//缓慢提速
            bdc_speed=0;
        }
        else{
            bdc_speed=Speed_Limit(bdc_speed_max,bdc_speed,increase_xj);
        }
        }else bdc_speed=0;

#if pid

        PID_Change();     //参数切换

#endif

        //左右平衡速度环
        pidout_1f = ROLL_SPEED_LOOP(0);

        Flag.T_20ms = 0;
    }


#if CH100_ON
    imu_process();
#else
    DMP_Read();    //获取姿态信息 (2ms)
#endif

    // 10ms控制 角度环
     if (Flag.T_10ms == 1) {

         if(abs(Bias_Line)<16 && flag_state !=2 && flag_state !=3 &&cross_flag>7){
             image_Error_wheel.Kp=2.2;
             image_Error_wheel.Kd=1.2;
         }else{
             image_Error_wheel.Kp=1.6;
             image_Error_wheel.Kd=0.6;
         }

         if(ele_flag==0)set_yaw = Positional_PID(&image_Error_wheel, 0, (float)Bias_Line, 0, 0.8);//0.6

         set_yaw = limit(set_yaw,-limit_yaw,limit_yaw);
         BLDC_YAW_ANGLE_Value = Speed_Limit(set_yaw,BLDC_YAW_ANGLE_Value,increase_dj);

         Dynamic_zero_roll = Dynamic_zero_cale();//动态零点获取

         //左右平衡角度环
         pidout_2f = ROLL_ANGLE_LOOP(pidout_1f);

         Flag.T_10ms = 0;
     }

    // 2ms控制 imu姿态结算 角速度
///////////////////////////////
        //姿态异常急停
        if ((go_flag == 2 || go_flag == 3)&& (Roll > 6  || Roll < -5)) {
            set_yaw=0;
            direct_bdc_speed(0);
            go_flag = 0;
        }

        //左右平衡角速度环
        pidout_3f = ROLL_ANGLESPEED_LOOP(pidout_2f);//

        set_angle(BLDC_YAW_ANGLE_Value);

        if (go_flag == 2 || go_flag == 3) {

        if (dllun==1){
            // 打印动量轮控制数据
            printf("动量轮控制 - 模式: %s, 输出值: %.2f, go_flag: %d, dllun: %d\n", 
#if torque_ctl
                   "力矩控制", pidout_3f, go_flag, dllun
#else  
                   "速度控制", pidout_3f, go_flag, dllun
#endif
            );
            
#if torque_ctl
            odrive_torque_ctl(0,pidout_3f);
#else
            odrive_speed_ctl(0,pidout_3f);
#endif

        }

        if(xjlun==1){
            // 打印行进轮控制数据
            printf("行进轮控制 - 速度: %.2f, bdc_speed_max: %.2f, bdc_speed_max_set: %.2f, xjlun: %d\n", 
                   bdc_speed, bdc_speed_max, bdc_speed_max_set, xjlun);
            odrive_speed_ctl(1,bdc_speed);
        }

            }else{
            set_yaw=0;
            bdc_speed_max_set=0;
            odrive_speed_ctl(1,0);
            o=0;

#if torque_ctl

            if(encoder_bldc>1)
            odrive_torque_ctl(0,-1.5);
            else if(encoder_bldc<-1)
            odrive_torque_ctl(0,1.5);
            else odrive_torque_ctl(0,0);

#else
            odrive_speed_ctl(0,0);
#endif

            }
//////////////////////////////////////

        if (Flag.T_100ms == 1) {

            switch (cross_flag)
            {
              case -1:
                switch (flag_state)
                {
                  case 0:
                      if(abs(Bias_Line)<8){
                      if(o++>10)
                      bdc_speed_max_set=5;
                      }
                      break;
                  case 2:
                      bdc_speed_max_set=3;
                      o=0;
                      break;
                  case 3:
                      bdc_speed_max_set=4;
                      o=0;
                      break;
                }
                  break;
              case 1:
              ////////////////////////
//              ele_flag=1;
              limit_yaw=40;
//              lane_change_Start(R_L_Flag);

              if(i++==110){//停车10s
                 i=0;
                 cross_flag=2;// 2
                 bdc_speed_max_set=5;//3 5

              }

                  break;
              case 2:

              if(i++>10){//正常开1s
                 if(abs(Bias_Line)<9){
                 i=0;
                 cross_flag=3;
                 beep_flag=1;
                 lane_change_Start(R_L_Flag);
                 ele_flag=1;
                 limit_yaw=30;
                 }
              }

                  break;

              case 3:
              if(i++==40){//斜开4s
                 i=0;
                 cross_flag=4;
                 beep_flag=1;

                  }
                  break;

              case 4:
              if(R_L_Flag==1?Bias_Line<0:Bias_Line>0)
              {

              if(i++==2){
                 i=0;

                 if(R_L_Flag==1)
                 {
                    set_yaw= -6;
                }else set_yaw=2;

                 cross_flag++;
                 beep_flag=1;
                 bdc_speed_max_set=4;
                }

              }else i=0;
                  break;

              case 5:
                  if(R_L_Flag==1?Bias_Line>0:Bias_Line<4){//6

                  set_yaw=0;
                  cross_flag++;
                  beep_flag=1;
                  ele_flag=0;
                  limit_yaw=30;
                  y=0;
                  }

                  break;

              case 6:

              if(i++==30){//保持3s
                 i=0;
                 cross_flag++;
              }
                  break;

              case 7:
                  if(abs(BLDC_YAW_ANGLE_Value)<15){
                      y++;
                      if(y>15&&y<28)
                      bdc_speed_max_set=6;
                      else if(y>28){
                      bdc_speed_max_set=8;//9
                      cross_flag++;
                      }
                  }
                  break;
            }

            if(fabs(BLDC_YAW_ANGLE_Value)>30 && bdc_speed_max>1.6 && cross_flag!=0) bdc_speed_max=3;
            else bdc_speed_max=bdc_speed_max_set;//偏差过大保护

            if(beep_flag==1||fabs(encoder_bldc)>18){//蜂鸣器响
                Beep_ON
                beep_flag=0;
            }else
            Beep_OFF

            Flag.T_100ms = 0;
        }

}
