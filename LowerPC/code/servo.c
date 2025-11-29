/*
 * servo.c
 * 舵机控制
 *  Created on: 2024年9月22日
 *      Author: 554.
 */
#include "servo.h"

float Middle=715;

void set_angle(float x)//设置角度
{

//  x = limit(x,-30,30);
  x = x+Middle;
  pwm_set_duty(ATOM1_CH1_P33_9,(uint32)x);

}

float Speed_Limit(float now,float last,float limit){

    if(fabs(now - last)<25)limit=limit*(2-fabs(now - last)/25);

    if ((now - last) >= limit)
        return (last + limit);
    else if ((now - last) <= -limit)
        return (last - limit);
    else
        return now;

    return last;
}
