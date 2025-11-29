/*
 * servo.h
 *
 *  Created on: 2024年9月22日
 *      Author: 554.
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"

/////////////////////////舵机转向中值，极限。
extern float Middle;

#define SERVO_MOTOR_FREQ 50  // 定义主板上舵机频率，单位为Hz，范围应在50-300之间
#define SERVO_MOTOR_DUTY(x) ((int)(PWM_DUTY_MAX / (1000.0 / SERVO_MOTOR_FREQ) * (1.0 + (x) / 270.0)))  // x为偏转角度（0-270度）

void set_angle(float x);//            703    722.2  740.74  759  18.5  (1.85)
float Speed_Limit(float now,float last,float limit);

#endif /* CODE_SERVO_H_ */
