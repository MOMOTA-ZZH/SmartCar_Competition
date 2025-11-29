/*
 * lane_change.c
 *  ��������߼�
 *  Created on: 2024��10��1��
 *      Author: 554.
 */
#include "lane_change.h"
float start_yaw=0,cur_yaw=0;
float set_yaw_update=0;

float lane_change_L=23,lane_change_R=-15;//26  -18

void lane_change_Start(int x) {
    //���õ�ǰ����,ץȡ��ǰ����ϵΪ��׼����ϵ
    start_yaw =Yaw;
    switch (x)
    {
     case -1:
     set_yaw_update=lane_change_R;
     set_yaw=-8;

     break;
     case 1:
     set_yaw_update=lane_change_L;
     set_yaw=7;//12

     break;
     default:
     set_yaw_update=0;
     cross_flag=5;

     break;
    }
}

void lane_change_ON(void){

    cur_yaw=Yaw-start_yaw;
    // ȷ�������������
    normalizeYaw(&cur_yaw);

    float value=set_yaw_update-cur_yaw;
    normalizeYaw(&value);
    if(fabs(value)<3){
        set_yaw_update=0;
        cross_flag++;
        beep_flag=1;
    }

    set_yaw = Positional_PID(&lane_change, value, 0, 0, 0);

}

// ȷ������Ǳ仯�����·���Ͻ���
void normalizeYaw(float *yaw) {
    while (*yaw > 180) {
        *yaw -= 360;
    }
    while (*yaw < -180) {
        *yaw += 360;
    }
}

