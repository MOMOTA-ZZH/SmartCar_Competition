/*
 * balance.h
 *
 *  Created on: 2024Äê1ÔÂ18ÈÕ
 *      Author: 554.
 */
#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_
#include "zf_common_headfile.h"
extern float Dynamic_zero_roll;
extern float BLDC_YAW_ANGLE_Value,set_yaw;
extern float ZHONGZHI_roll,Zero_roll;
extern int8_t image;
extern float ROLL_SPEED_I;
extern float Z_Coefficient,Y_Coefficient;

float ROLL_ANGLE_LOOP(float Set_Roll);
float ROLL_ANGLESPEED_LOOP(float value);
float ROLL_SPEED_LOOP(float value);

uint8 Attitude_stability_judgment(float roll, float pitch);
float Dynamic_zero_cale(void);

#endif
