/*
 * Serial.c
 * 上位机通信模块
 *  Created on: 2024年9月22日
 *      Author: 554.
 */
#include "Serial.h"

/**************************************************
解析外部数据包
**************************************************/

//解析数据包
int   Bias_Line=0;//偏差
int8  flag_state=0;//目前状态标志

#if R_L_ON
int8  R_L_Flag=0;//变道方向
#else
int8  R_L_Flag=1;//变道方向
#endif



int8  R_Flag=0;//右
int8  L_Flag=0;//左
int8  crossing1_Flag=0;//状态-1
int8  zero_Flag=0;//状态0
int8  crossing_Flag=0;//斑马线1
int8  middleCone_Flag=0;//中间锥桶2
int8  Cone_Flag=0;//普通锥桶3

//左1 右2

int8  Bias_Line_temp=0;
int8 flag_state_temp=0;
uint8 R_L_Flag_temp=0;
int num_2=0;

uint8_t RxState = 0;//接收状态
void Data_Receive_Analysis(uint8_t RxData)
{

        if(RxState==0&&RxData==HEAD1)
        {
            RxState=1;
        }
        else if(RxState==1)//接收有效数据
        {
            switch (num_2)
            {
              case 0:
                  Bias_Line_temp=RxData;
                  num_2++;
                  break;
              case 1:
                  flag_state_temp=RxData;
                  num_2++;
                  break;

              case 2:
                  R_L_Flag_temp=RxData;
                  num_2=0;
                  RxState=2;
                  break;
            }
        }
        else if(RxState==2&&RxData==TAil)
        {
            RxState=0;
            Bias_Line=Bias_Line_temp;

            switch (flag_state_temp)
              {
                case -1:
                if(crossing1_Flag++>1)
                {
                    flag_state=-1;
                    cross_flag=0;
                    direct_bdc_speed(3);//3
                    crossing1_Flag=0;}
                break;
                case 0:
                if(zero_Flag++>3)
                {
                    flag_state=0;
                    zero_Flag=0;}
                break;
                case 1:
                if(crossing_Flag++>1)
                {
                    flag_state=1;
//                    direct_bdc_speed(0);
                    crossing_Flag=0;}
                break;
                case 2:
                if(middleCone_Flag++>3)
                {
                    flag_state=2;
                    middleCone_Flag=0;}
                break;
                case 3:
                if(Cone_Flag++>3)
                {
                    flag_state=3;
                    Cone_Flag=0;}
                break;
              }

#if R_L_ON
            switch (R_L_Flag_temp)
              {
                case 1:
                if(L_Flag++>3)
                    R_L_Flag=1;
                break;
                case 2:
                if(R_Flag++>3)
                    R_L_Flag=-1;
                break;
              }
#endif

        }
        else
        {
            RxState=0;
        }

}


void Serial_init(void){

    R_Flag=0;//右
    L_Flag=0;//左
    zero_Flag=0;//状态0
    crossing_Flag=0;//斑马线1
    middleCone_Flag=0;//中间锥桶2
    Cone_Flag=0;//普通锥桶3
    crossing1_Flag=0;
}

