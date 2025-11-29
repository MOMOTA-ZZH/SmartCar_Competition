/*
 * init.c
 * 初始化及屏幕按键控制
 *  Created on: 2024年1月16日
 *      Author: 554.
 */
#include "init.h"
int8_t go_flag = 0;//启动标志位
float ADCbattery;//电池电压adc
uint8 beep_flag=0;
uint8 ele_flag=0;
int8 cross_flag=-2;

#define dmp_wait     (1)

void Init(void) {

      // LED灯初始化
      GPIO_LED_Init();

      gpio_init(Beep, GPO, 0, GPO_PUSH_PULL);//蜂鸣器初始化

      init_lowpass_filter(&filter_gyro, 4, 1000);
#if CH100_ON
//      uart_init(UART_1,460800,UART1_TX_P11_12,UART1_RX_P11_10);//ch100
//      uart_rx_interrupt(UART_1, 1);// 打开串口1接收中断
#else
      // 6050陀螺仪初始化
      soft_iic_6050_init();

      for (int i = 0; i < 10; i++){
          system_delay_ms(5);
          DMP_Read();
      }

      if(Roll==0 && Pitch==0 && Yaw==0)
      {
      mpu6050_init();
      DMP_Init();                  //DMP初始化
      system_delay_ms(100);

#if dmp_wait
      for (int i = 0; i < 35000; i++)
      {
      DMP_Read();
      }
      do{
      DMP_Read();
      }while(!(abs(gyro[0])<=1 && abs(gyro[1])<=1 && abs(gyro[2])<=1));                    //等待dmp稳定
#endif
      }

#endif

      pwm_init(ATOM1_CH1_P33_9,SERVO_MOTOR_FREQ,0);
      set_angle(0);

      PID_param_init();//pid初始化

      ZHONGZHI_roll=Zero_roll;

}

void Init_win(void) {

    pit_ms_init(CCU61_CH1, 2);//2ms

    Beep_ON;
    system_delay_ms(500);
    Beep_OFF;

}

uint8 Directory=0;//目录数值
uint8 TFT_Directory=0;//TFT目录数值
uint8 ele=0;//蓝牙标志位

char Set[20];//1设定数值
char Set2[20];//2设定数值
char Set3[20];//3设定数值
char Set4[20];//4设定数值

void Directory_Init(void){//菜单控制的一些初始化
for (int var = 0; var < 20; var++) {
   Set[var]=' ';
   Set2[var]=' ';
   Set3[var]=' ';
   Set4[var]=' ';
}
    Set[1]='>';
    Set2[1]='>';
    Set3[1]='>';
    Set4[1]='>';
}

//按键控制
void Key_ControlM(void)
{

    if(KEY_Read(KEY0) == 0)
    {
        if(go_flag==1){
              while(1){
                  LED_Ctrl(LED0,ON);
                  if(fabs(pidout_2f)<1)
                      break;
            }
              LED_Ctrl(LED0,OFF);
        }
        go_flag++;

        if(go_flag==2){//一些标志位清空
            cross_flag=-2;
            flag_state=0;
#if R_L_ON
            R_L_Flag=0;
#endif
            Serial_init();
        }

        if(go_flag==3){
            go_flag=0;
            set_yaw=0;
            bdc_speed_max=0;
            bdc_speed_max_set=0;
        }

        while(KEY_Read(KEY0) == 0);
    }

    if(KEY_Read(KEY1) == 0)
    {
//        ele_flag=1-ele_flag;
        while(KEY_Read(KEY1) == 0);
    }

    if(KEY_Read(KEY2) == 0)
    {
        cross_flag=-1;
        flag_state=0;
#if R_L_ON
        R_L_Flag=0;
#endif
        Serial_init();
        bdc_speed_max_set=5;
        while(KEY_Read(KEY2) == 0);
    }

    if(KEY_Read(KEY3) == 0)
    {
        Directory++;
        if(Directory==3)Directory=0;
        while(KEY_Read(KEY3) == 0);
    }

}


/* @param    x:0 - 15 (行)
* @param    y:0 - 10 (列*/
void TFTSPI_P8X8Str(unsigned char x, unsigned char y,const char dat[]){
    tft180_show_string(x*8,y*14,dat);
}

char  txt[40];

void TFT_Task(void){//屏幕显示
    switch (Directory)
    {
    case 0:
        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
        {
            tft180_clear();    //清屏
            TFT_Directory = Directory;   //记录状态
        }

//        sprintf((char*)txt,"Pitch:%.02f  ",Pitch);
//        TFTSPI_P8X8Str(2,0,txt);

        sprintf((char*)txt,"Roll:%.02f  ",Roll);
        TFTSPI_P8X8Str(2,0,txt);


        sprintf((char*)txt,"Yaw:%.02f  ",Yaw);
        TFTSPI_P8X8Str(2,1,txt);

//        sprintf((char*)txt,"GX:%d  ",gyro[0]);
//        TFTSPI_P8X8Str(2,3,txt);
#if CH100_ON
        sprintf((char*)txt,"GY:%.2f  ",gyro6[0]);
        TFTSPI_P8X8Str(2,2,txt);
#else
        sprintf((char*)txt,"GY:%d  ",-gyro[0]);
        TFTSPI_P8X8Str(2,2,txt);
#endif

//        sprintf((char*)txt,"GZ:%d  ",gyro[2]);
//        TFTSPI_P8X8Str(2,5,txt);

        sprintf((char*)txt,"f0:%.2f  ",encoder_bldc);
        TFTSPI_P8X8Str(2,3,txt);

        sprintf((char*)txt,"f1:%.2f  ",encoder_rear);
        TFTSPI_P8X8Str(2,4,txt);

        sprintf((char*)txt,"0V:%.2f  ",pidout_3f);
        TFTSPI_P8X8Str(2,5,txt);

        sprintf((char*)txt,"0:%.2f ",pidout_2f);
        TFTSPI_P8X8Str(2,6,txt);

        sprintf((char*)txt,"V:%.2f ",pidout_1f);
        TFTSPI_P8X8Str(10,6,txt);

        sprintf((char*)txt,"I:%.2f  ",roll_speed_ring.Ek2);
        TFTSPI_P8X8Str(2,7,txt);

        sprintf((char*)txt,"image:%d  ",Bias_Line);
        TFTSPI_P8X8Str(2,8,txt);

        sprintf((char*)txt,"flag:%d  (%d)  ",flag_state,cross_flag);
        TFTSPI_P8X8Str(2,9,txt);

        sprintf((char*)txt,"E :%d  flag:%d  ",ele_flag,go_flag);
        TFTSPI_P8X8Str(2,10,txt);

        break;
    case 1:
        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
        {
            tft180_clear();    //清屏
            TFT_Directory = Directory;   //记录状态
        }
        sprintf((char*)txt,"%cRoll:%.02f  ",Set2[1],Zero_roll);
        TFTSPI_P8X8Str(1,0,txt);

        sprintf((char*)txt,"%cR0V-kP:%.03f!  ",Set2[2],roll_anglespeed_ring.Kp);
        TFTSPI_P8X8Str(1,1,txt);

        sprintf((char*)txt,"%cR0V-ki:%.03f  ",Set2[3],roll_anglespeed_ring.Ki);
        TFTSPI_P8X8Str(1,2,txt);

        sprintf((char*)txt,"%cR0V-kd:%.03f  ",Set2[4],roll_anglespeed_ring.Kd);
        TFTSPI_P8X8Str(1,3,txt);

        sprintf((char*)txt,"%cR0-kP:%.03f!  ",Set2[5],roll_angle_ring.Kp);
        TFTSPI_P8X8Str(1,4,txt);

        sprintf((char*)txt,"%cR0-ki:%.03f  ",Set2[6],roll_angle_ring.Ki);
        TFTSPI_P8X8Str(1,5,txt);

        sprintf((char*)txt,"%cR0-kd:%.03f  ",Set2[7],roll_angle_ring.Kd);
        TFTSPI_P8X8Str(1,6,txt);

        sprintf((char*)txt,"%cRV-kP:%.03f!  ",Set2[8],roll_speed_ring.Kp);
        TFTSPI_P8X8Str(1,7,txt);

        sprintf((char*)txt,"%cRV-ki:%.03f  ",Set2[9],roll_speed_ring.Ki);
        TFTSPI_P8X8Str(1,8,txt);

        sprintf((char*)txt,"%cRV-kd:%.03f  ",Set2[10],roll_speed_ring.Kd);
        TFTSPI_P8X8Str(1,9,txt);

        sprintf((char*)txt,"%cR :%d  flag:%d  ",Set2[11],dllun,go_flag);
        TFTSPI_P8X8Str(1,10,txt);

        break;
    case 2:
            if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
            {
                tft180_clear();    //清屏
                TFT_Directory = Directory;   //记录状态
            }
            sprintf((char*)txt,"%cRoll:%.02f  ",Set3[1],Zero_roll);
            TFTSPI_P8X8Str(1,0,txt);

            sprintf((char*)txt,"%cY-kP:%.03f!  ",Set3[2],image_Error_wheel.Kp);
            TFTSPI_P8X8Str(1,1,txt);

            sprintf((char*)txt,"%cY-ki:%.03f  ",Set3[3],image_Error_wheel.Ki);
            TFTSPI_P8X8Str(1,2,txt);

            sprintf((char*)txt,"%cY-kd:%.03f  ",Set3[4],image_Error_wheel.Kd);
            TFTSPI_P8X8Str(1,3,txt);

            sprintf((char*)txt,"%cDJ++:%.02f  ",Set3[5],increase_dj);
            TFTSPI_P8X8Str(1,4,txt);

            sprintf((char*)txt,"%cXJ++:%.02f  ",Set3[6],increase_xj);
            TFTSPI_P8X8Str(1,5,txt);

            sprintf((char*)txt,"%cMAX-I:%.01f  ",Set3[7],ROLL_SPEED_I);
            TFTSPI_P8X8Str(1,6,txt);

            sprintf((char*)txt,"%cZ:%.02f Y:%.02f  ",Set3[8],Z_Coefficient,Y_Coefficient);
            TFTSPI_P8X8Str(1,7,txt);

            sprintf((char*)txt,"%c0:%.02f  ",Set3[9],Middle);
            TFTSPI_P8X8Str(1,8,txt);

            sprintf((char*)txt,"image:%d  ",Bias_Line);
            TFTSPI_P8X8Str(2,9,txt);

            sprintf((char*)txt,"flag:%d  ",go_flag);
            TFTSPI_P8X8Str(5,10,txt);

            break;
    }

}

//void Directory_Init(void){//菜单控制的一些初始化
//for (int var = 0; var < 20; var++) {
//   Set[var]=' ';
//   Set2[var]=' ';
//   Set3[var]=' ';
//   Set4[var]=' ';
//   Set6[var]=' ';
//}
//Set[1]='>';
//Set2[1]='>';
//Set3[1]='>';
//Set4[1]='>';
//Set6[1]='>';
//}

////屏幕控制
//char  txt[40];
//void TFT_Task(void)
//{
//    switch (Directory)
//    {
//    case 0:
//        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//        {
//            TFTSPI_Init(2);                 //屏幕初始化为竖屏 倒显示
//            TFTSPI_CLS(u16CYAN);    //清屏
//            TFT_Directory = Directory;   //记录状态
//        }
//        //batv = batv * 11 / 25;  // x/4095*3.3*100*5.7
//        //sprintf(txt, "BAT:%d.%02dV ", batv / 100, batv % 100);  // *3.3/4095*3
//            sprintf((char*)txt, "  adcV:%.02fV  ", ADCbattery);
//            TFTSPI_P8X8Str(0, 0, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt,"  Pitch:%.02f  ",Pitch);
//            TFTSPI_P8X8Str(0,1,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  Roll:%.02f  ",Roll);
//            TFTSPI_P8X8Str(0,2,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  Yaw:%.02f  ",Yaw);
//            TFTSPI_P8X8Str(0,3,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  gx:%06d  ",gyro[0]);
//            TFTSPI_P8X8Str(0,4,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  gy:%06d  ",gyro[1]);
//            TFTSPI_P8X8Str(0,5,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  gz:%06d  ",gyro[2]);
//            TFTSPI_P8X8Str(0,6,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  zuo:%.02f  ",encoder_bldc_f);
//            TFTSPI_P8X8Str(0,7,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  you:%.02f  ",encoder_bldc_b);
//            TFTSPI_P8X8Str(0,8,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  lun:%.02f  ",encoder_bdc);
//            TFTSPI_P8X8Str(0,9,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  OV:%.02f  ",pidout_3f);
//            TFTSPI_P8X8Str(0,10,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  O:%.02f  ",pidout_2f);
//            TFTSPI_P8X8Str(0,11,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  V:%.02f  ",pidout_1f);
//            TFTSPI_P8X8Str(0,12,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  BOV:%.02f  ",pidout_3a);
//            TFTSPI_P8X8Str(0,13,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  BO:%.02f  ",pidout_2a);
//            TFTSPI_P8X8Str(0,14,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  BV:%.02f  ",pidout_1a);
//            TFTSPI_P8X8Str(0,15,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  YOV:%.02f  ",pidout_6f);
//            TFTSPI_P8X8Str(0,16,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  YO:%.02f  ",pidout_5f);
//            TFTSPI_P8X8Str(0,17,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  YV:%.02f  ",pidout_4f);
//            TFTSPI_P8X8Str(0,18,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"  go_flag:%d  ",go_flag);
//            TFTSPI_P8X8Str(0,19,txt,u16RED,u16CYAN);
//        break;
//    case 1:
//        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//        {
//            TFTSPI_CLS(u16CYAN);    //清屏
//            TFT_Directory = Directory;   //记录状态
//        }
//            sprintf((char*)txt," %cPitch:%.01f ",Set[1],ZHONGZHI_PITCH);
//            TFTSPI_P8X8Str(0,1,txt,u16RED,Set[1]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0V-kp:%.03f  ",Set[2],bdc_pid[bdc].bdc_anglespeed_wheel.Kp);
//            TFTSPI_P8X8Str(0,2,txt,u16RED,Set[2]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0V-ki:%.03f  ",Set[3],bdc_anglespeed_wheel.Ki);
//            TFTSPI_P8X8Str(0,3,txt,u16RED,Set[3]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0V-kd:%.03f  ",Set[4],bdc_anglespeed_wheel.Kd);
//            TFTSPI_P8X8Str(0,4,txt,u16RED,Set[4]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0-kp:%.03f  ",Set[5],bdc_pid[bdc].bdc_angle_wheel.Kp);
//            TFTSPI_P8X8Str(0,5,txt,u16RED,Set[5]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0-ki:%.03f  ",Set[6],bdc_angle_wheel.Ki);
//            TFTSPI_P8X8Str(0,6,txt,u16RED,Set[6]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP0-kd:%.03f  ",Set[7],bdc_pid[bdc].bdc_angle_wheel.Kd);
//            TFTSPI_P8X8Str(0,7,txt,u16RED,Set[7]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cPV-kp:%.03f  ",Set[8],bdc_pid[bdc].bdc_speed_wheel.Kp);
//            TFTSPI_P8X8Str(0,8,txt,u16RED,Set[8]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cPV-ki:%.03f  ",Set[9],bdc_speed_wheel.Ki*1000);
//            TFTSPI_P8X8Str(0,9,txt,u16RED,Set[9]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cPV-kd:%.03f  ",Set[10],bdc_speed_wheel.Kd);
//            TFTSPI_P8X8Str(0,10,txt,u16RED,Set[10]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cspeedV:%.01f  ",Set[11],bdc_speed);
//            TFTSPI_P8X8Str(0,11,txt,u16RED,Set[11]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %csetV:%.01f  ",Set[11],bdc_speed_max);
//            TFTSPI_P8X8Str(0,12,txt,u16RED,Set[11]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cSq:%d  ",Set[12],xjsiq);
//            TFTSPI_P8X8Str(0,13,txt,u16RED,Set[12]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cP :%d  ",Set[13],xjlun);
//            TFTSPI_P8X8Str(0,14,txt,u16RED,Set[13]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cbdc-pid:(%d) ",Set[14],bdc+1);
//            TFTSPI_P8X8Str(0,15,txt,u16RED,Set[14]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt,"  go_flag:%d  ",go_flag);
//            TFTSPI_P8X8Str(0,19,txt,u16RED,u16CYAN);
//
//        break;
//    case 2:
//        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//        {
//            TFTSPI_CLS(u16CYAN);    //清屏
//            TFT_Directory = Directory;   //记录状态
//        }
//            sprintf((char*)txt," %cRoll:%.03f  ",Set2[1],Zero_roll);
//            TFTSPI_P8X8Str(0,1,txt,u16RED,Set2[1]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0V-kp:%.03f  ",Set2[2],roll_anglespeed_ring.Kp);
//            TFTSPI_P8X8Str(0,2,txt,u16RED,Set2[2]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0V-ki:%.03f  ",Set2[3],roll_anglespeed_ring.Ki);
//            TFTSPI_P8X8Str(0,3,txt,u16RED,Set2[3]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0V-kd:%.03f  ",Set2[4],roll_anglespeed_ring.Kd);
//            TFTSPI_P8X8Str(0,4,txt,u16RED,Set2[4]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0-kp:%.03f  ",Set2[5],roll_angle_ring.Kp);
//            TFTSPI_P8X8Str(0,5,txt,u16RED,Set2[5]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0-ki:%.03f  ",Set2[6],roll_angle_ring.Ki);
//            TFTSPI_P8X8Str(0,6,txt,u16RED,Set2[6]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR0-kd:%.03f  ",Set2[7],roll_angle_ring.Kd);
//            TFTSPI_P8X8Str(0,7,txt,u16RED,Set2[7]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cRV-kp:%.03f  ",Set2[8],roll_speed_ring.Kp);
//            TFTSPI_P8X8Str(0,8,txt,u16RED,Set2[8]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cRV-ki:%.03f  ",Set2[9],roll_speed_ring.Ki);
//            TFTSPI_P8X8Str(0,9,txt,u16RED,Set2[9]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cRV-kd:%.03f  ",Set2[10],roll_speed_ring.Kd);
//            TFTSPI_P8X8Str(0,10,txt,u16RED,Set2[10]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cR :%d  ",Set2[11],dllun);
//            TFTSPI_P8X8Str(0,11,txt,u16RED,Set2[11]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cV++ :%.1f  ",Set2[12],increase_xj);
//            TFTSPI_P8X8Str(0,12,txt,u16RED,Set2[12]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY++ :%.1f  ",Set2[13],increase_dl);
//            TFTSPI_P8X8Str(0,13,txt,u16RED,Set2[13]=='>'?u16GREEN:u16CYAN);
//
//
//
//            sprintf((char*)txt,"  go_flag:%d  ",go_flag);
//            TFTSPI_P8X8Str(0,19,txt,u16RED,u16CYAN);
//
//       break;
//    case 3:
//        if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//        {
//            TFTSPI_CLS(u16CYAN);    //清屏
//            TFT_Directory = Directory;   //记录状态
//        }
//            sprintf((char*)txt," %cYAW:%.02f  ",Set3[1],BLDC_YAW_ANGLE_Value);
//            TFTSPI_P8X8Str(0,1,txt,u16RED,Set3[1]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0V-kp:%.03f  ",Set3[2],bldc_yaw_anglespeed_wheel.Kp);
//            TFTSPI_P8X8Str(0,2,txt,u16RED,Set3[2]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0V-ki:%.03f  ",Set3[3],bldc_yaw_anglespeed_wheel.Ki);
//            TFTSPI_P8X8Str(0,3,txt,u16RED,Set3[3]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0V-kd:%.03f  ",Set3[4],bldc_yaw_anglespeed_wheel.Kd);
//            TFTSPI_P8X8Str(0,4,txt,u16RED,Set3[4]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0-kp:%.03f  ",Set3[5],bldc_yaw_angle_wheel.Kp);
//            TFTSPI_P8X8Str(0,5,txt,u16RED,Set3[5]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0-ki:%.03f  ",Set3[6],bldc_yaw_angle_wheel.Ki);
//            TFTSPI_P8X8Str(0,6,txt,u16RED,Set3[6]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY0-kd:%.03f  ",Set3[7],bldc_yaw_angle_wheel.Kd);
//            TFTSPI_P8X8Str(0,7,txt,u16RED,Set3[7]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cYV-kp:%.03f  ",Set3[8],bldc_yaw_speed_wheel.Kp);
//            TFTSPI_P8X8Str(0,8,txt,u16RED,Set3[8]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cYV-ki:%.03f  ",Set3[9],bldc_yaw_speed_wheel.Ki);
//            TFTSPI_P8X8Str(0,9,txt,u16RED,Set3[9]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cYV-kd:%.03f  ",Set3[10],bldc_yaw_speed_wheel.Kd);
//            TFTSPI_P8X8Str(0,10,txt,u16RED,Set3[10]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cY :%d  ",Set3[11],yawh);
//            TFTSPI_P8X8Str(0,11,txt,u16RED,Set3[11]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cIns.kp :%.2f  ",Set3[12],inertial_Error_wheel.Kp);
//            TFTSPI_P8X8Str(0,12,txt,u16RED,Set3[12]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt," %cIns.lim:%.1f  ",Set3[13],ins_limit);
//            TFTSPI_P8X8Str(0,13,txt,u16RED,Set3[13]=='>'?u16GREEN:u16CYAN);
//
//
//            sprintf((char*)txt,"  go_flag:%d  ",go_flag);
//            TFTSPI_P8X8Str(0,19,txt,u16RED,u16CYAN);
//
//       break;
//    case 4:
//            if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//            {
//                TFTSPI_Init(0);                 //屏幕初始化为竖屏 倒显示
//                TFTSPI_CLS(u16CYAN);    //清屏
//                TFT_Directory = Directory;   //记录状态
//            }
//            if(ele==2){
//
//            OV7725_TFTSPI_Road(0, 0, 60, 80,(const uint16 *)scc8660_image);
//            TFTSPI_Draw_Rectangle(start_col,start_row,end_col-1,end_row-1,u16RED);
//            TFTSPI_Draw_Rectangle(start_col,65+start_row,end_col-1,64+end_row,u16RED);
//
//            sprintf(txt, "L-MAX:%d   ", GET_L_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 1, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "L-MIN:%d   ", GET_L_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 2, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "L(%d)(%d)", L_min,L_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 3, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "H-MAX:%d   ", GET_a_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 4, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "H-MIN:%d   ", GET_a_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 5, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "H(%d)(%d)", h_min,h_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 6, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "S-MAX:%d   ", GET_b_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 7, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "S-MIN:%d   ", GET_b_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 8, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "S(%d)(%d", s_min,s_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 9, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cELE:%d   ", Set4[7],ele);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 10, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            }
//            else{
//
//            OV7725_TFTSPI_Road(0, 0, 60, 80,(const uint16 *)scc8660_image);
//
//            TFTSPI_Draw_Rectangle(result1.x-result1.w/2,result1.y-result1.h/2,result1.x+result1.w/2,result1.y+result1.h/2,u16YELLOW);
//            TFTSPI_Draw_Circle(result1.x,result1.y,4,u16BLUE);
//
//
//#if color_number
//
//            TFTSPI_Draw_Rectangle(result2.x-result2.w/2,result2.y-result2.h/2,result2.x+result2.w/2,result2.y+result2.h/2,u16GREEN);
//            TFTSPI_Draw_Circle(result2.x,result2.y,4,u16BLUE);
//
//#endif
//
//            sprintf(txt, "y:%d y2:%d ", result1.y,result2.y);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 1, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "x:%d x2:%d ", result1.x,result2.x);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 2, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cL-MAX:%d   ", Set4[1],L_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 3, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cL-MIN:%d   ", Set4[2],L_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 4, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cH-MAX:%d   ", Set4[3],h_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 5, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cH-MIN:%d   ", Set4[4],h_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 6, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cS-MAX:%d   ", Set4[5],s_max);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 7, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cS-MIN:%d   ", Set4[6],s_min);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 8, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, "%cELE:%d   ", Set4[7],ele);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 9, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//            sprintf(txt, " 6:%d   ",target_board_number);                    //将变量填充到字符串的对应位置，并将字符串存放到txt[]中
//            TFTSPI_P8X8Str(10, 10, txt, u16RED, u16BLUE);              //将txt中 内容显示出来
//
//            }
//        break;
//    case 5:
//            if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//            {
//                TFTSPI_CLS(u16CYAN);    //清屏
//                TFT_Directory = Directory;   //记录状态
//            }
//            sprintf((char*)txt,"ins:%d  ",ins_flag);
//            TFTSPI_P8X8Str(2,1,txt,u16RED,ins_flag==1?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt,"R:%d  ",speed_reversal_flag);
//            TFTSPI_P8X8Str(2,2,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"x:%.02f  ",navigation.x_cur);
//            TFTSPI_P8X8Str(2,3,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"y:%.02f  ",navigation.y_cur);
//            TFTSPI_P8X8Str(2,4,txt,u16RED,u16CYAN);
//
////            sprintf((char*)txt,"  yaw:%.02f  ",Yaw);
////            TFTSPI_P8X8Str(0,5,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"%d x:%.02f  ",g,set_x[g]);
//            TFTSPI_P8X8Str(2,5,txt,u16RED,u16GREEN);
//
//            sprintf((char*)txt,"%d y:%.02f  ",g,set_y[g]);
//            TFTSPI_P8X8Str(2,6,txt,u16RED,u16GREEN);
//
//            sprintf((char*)txt, ">%d  (%d)  ", ACC_number,ACC_number_max);
//            TFTSPI_P8X8Str(2, 8, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt,"yaw:%.02f  ",navigation.cur_yaw);
//            TFTSPI_P8X8Str(2,9,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt,"set*:%.02f  ",navigation.inertial_navigation_set_yaw_update-navigation.cur_yaw);
//            TFTSPI_P8X8Str(2,10,txt,u16RED,u16CYAN);
//
//            sprintf((char*)txt, "speed1:(%d)  ", (int)inertial_speed_max);
//            TFTSPI_P8X8Str(1, 11, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt, "speed2:(%d)  ", (int)inertial_speed_max2);
//            TFTSPI_P8X8Str(1, 12, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt,"L:%.02f  ",navigation.ins_2distance);
//            TFTSPI_P8X8Str(1,13,txt,u16RED,u16CYAN);
//
//        break;
//    case 6:
//            if(TFT_Directory != Directory)//防止过多清屏，换页才清屏
//            {
//                TFTSPI_CLS(u16CYAN);    //清屏
//                TFT_Directory = Directory;   //记录状态
//            }
//
//            sprintf((char*)txt, "%d x:%.2f  ",g, set_x[g]);
//            TFTSPI_P8X8Str(1, 1, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt, "%d y:%.2f ",g, set_y[g]);
//            TFTSPI_P8X8Str(1, 2, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt, "%d x:%.2f  ",g+1, set_x[g+1]);
//            TFTSPI_P8X8Str(1, 3, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt, "%d y:%.2f ",g+1, set_y[g+1]);
//            TFTSPI_P8X8Str(1, 4, txt, u16RED, u16CYAN);
//
//            sprintf((char*)txt, "%cBX[%d]  (%d) ", Set6[1],bx1+1,BX[bx1-1]);
//            TFTSPI_P8X8Str(1, 6, txt, u16RED, Set6[1]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt, "%c[>%d]++ ", Set6[2],g);
//            TFTSPI_P8X8Str(1, 7, txt, u16RED, Set6[2]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt, "%c[>%d]-- ", Set6[3],g);
//            TFTSPI_P8X8Str(1, 8, txt, u16RED, Set6[3]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt, "%c(read)! ", Set6[4]);
//            TFTSPI_P8X8Str(1, 9, txt, u16RED, Set6[4]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt, "%c(write)! ", Set6[5]);
//            TFTSPI_P8X8Str(1, 10, txt, u16RED, Set6[5]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt,"%cMax1:%d  ",Set6[6],(int)inertial_speed_max);
//            TFTSPI_P8X8Str(1,11,txt,u16RED,Set6[6]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt,"%cMax2:%d  ",Set6[7],(int)inertial_speed_max2);
//            TFTSPI_P8X8Str(1,12,txt,u16RED,Set6[7]=='>'?u16GREEN:u16CYAN);
//
//            sprintf((char*)txt, "%cRtk-S:%.1fM ", Set6[8],ins_switch_distance);
//            TFTSPI_P8X8Str(1, 13, txt, u16RED, Set6[8]=='>'?u16GREEN:u16CYAN);
//        break;
//}
//}
//
//
//void insertElement(int size, int index, int element) {
//    if (index < 0 || index >= size) return; // 索引越界检查
//
//    // 将从index开始到数组末尾的所有元素向后移动一位
//    for (int i = size-1; i > index; i--) {
//        set_x[i] = set_x[i-1];
//        set_y[i] = set_y[i-1];
//    }
//
//    for (int i = 0; i < 10; i++) {
//        if(BX[i]>index)
//        BX[i]++;
//    }
//
//    set_x[index] = (float)element;
//    set_y[index] = (float)element;// 在index位置插入新元素
//}
//
//void deleteElement(int size, int index) {
//    if (index < 0 || index >= size) return; // 索引越界检查
//
//    // 将从index+1开始到数组末尾的所有元素向前移动一位
//    for (int i = index; i < size - 1; i++) {
//        set_x[i] = set_x[i+1];
//        set_y[i] = set_y[i+1];
//    }
//
//    for (int i = 0; i < 10; i++) {
//        if(BX[i]>index)
//        BX[i]--;
//    }
//}


