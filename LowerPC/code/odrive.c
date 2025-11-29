/*
 * odrive.c
 * odrive控制
 *  Created on: 2024年9月15日
 *      Author: 554
 */
#include "odrive.h"
float low_pass = 0.6;//上一次数据占比
uint8_t odrive_encoder=0;
float encoder_bldc=0,encoder_rear=0,bdc_speed,bdc_speed_max,bdc_speed_max_set=0;
float encoder_bldc_get=0,encoder_rear_get=0;

char odrive_data[30];
uint8_t rx_data[50];//数据数组

//odrive速度控制
void odrive_speed_ctl(unsigned char num, float speed)
{
    speed=limit(speed,-28,28);
    memset(odrive_data, 0, sizeof(odrive_data));
    sprintf((char*)odrive_data, "v %d %+.2f 0 \n",num,speed);
    uart_write_string(odrive_UART, odrive_data);
}

//odrive力矩控制
void odrive_torque_ctl(unsigned char num, float torque)
{

    torque=limit(torque,-2.4,2.4);
    memset(odrive_data, 0, sizeof(odrive_data));
    sprintf((char*)odrive_data, "c %d %+.2f \n",num,torque);
    uart_write_string(odrive_UART, odrive_data);
}

//获得odrive电机速度
void odrive_feedback(unsigned char num)
{
    memset(odrive_data, 0, sizeof(odrive_data));

#if odrive_encoder2
    switch (num)
        {
        case 0:
            odrive_encoder=0;
            sprintf((char*)odrive_data, "r axis0.encoder.vel_estimate \n");
            break;
        case 1:
            odrive_encoder=1;
            sprintf((char*)odrive_data, "r axis1.encoder.vel_estimate \n");
            break;
        }
#else
    sprintf((char*)odrive_data, "f %d \n",num);
#endif

    uart_write_string(odrive_UART, odrive_data);
}


uint8_t buf2;
int num2=0,flag=0;

void odrive_Anl(char *p){

#if odrive_encoder2
    switch (odrive_encoder)
    {
      case 0:

          sscanf(p,"%f",&encoder_bldc_get);
          encoder_bldc= encoder_bldc * low_pass + encoder_bldc_get * (1 - low_pass);

      break;

      case 1:

          sscanf(p,"%f",&encoder_rear_get);
          encoder_rear= encoder_rear * low_pass + encoder_rear_get * (1 - low_pass);

      break;
    }

    odrive_encoder=0;
#else

    sscanf(p,"%f",&encoder_bldc_get);
    encoder_bldc= encoder_bldc * low_pass + encoder_bldc_get * (1 - low_pass);

#endif

}

//解析得到odrive速度(串口中断中已调用)
void odrive_analyze_speed(void)
{
    uart_query_byte(odrive_UART, &buf2);
#if odrive_encoder2
    switch (flag)
     {
     case 0:
         if(buf2 == 0x0D){
             flag = 1;
             break;
         }else
             rx_data[num2++]=buf2;
         if(num2>=40){
         num2 = 0;
         memset(rx_data, 0, sizeof(rx_data));
         }
         break;
     case 1:
         if(buf2 == 0x0A)
             odrive_Anl((char *)rx_data);
         num2 = 0;
         flag = 0;
         memset(rx_data, 0, sizeof(rx_data));
         break;
     }
#else
    switch (flag)
    {
    case 0:
        if(buf2 == 0x20)
        {
            num2 = 0;
            flag = 1;
        }else    //不满足，舍弃一帧数据
        {
            num2 = 0;
            flag = 0;
        }
        memset(rx_data, 0, sizeof(rx_data));
        break;
    case 1:
        if(buf2 == 0x20){
            num2 = 0;
            break;
        }
        else if(buf2 == 0x0D){
            flag = 2;
            break;
        }
        rx_data[num2++]=buf2;
        if(num2>=40)flag = 0;
        break;
    case 2:
        if(buf2 == 0x0A)
            odrive_Anl((char *)rx_data);
        flag = 0;
        break;
    }
#endif

}


