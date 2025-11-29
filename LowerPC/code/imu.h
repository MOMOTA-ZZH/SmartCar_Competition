/*
 * imu.h
 *
 *  Created on: 2024年10月19日
 *      Author: 554.
 */
#ifndef _imu_h_
#define _imu_h_

#include "zf_common_headfile.h"

#define M_PI (3.1415927f)
#define sampleFreq	500.0f			// 采样频率(根据定时器修改)
#define twoKpDef	(20)	// 2 * kp  0.1
#define twoKiDef	(0.005)	// 2 * ki  0.0

#define gyro_gain  M_PI/180//将陀螺仪的原始测量值转换为弧度每秒（rad/s）的增益系数
#define ch100gain  16.4f

float my_absf(float x);
void imu_process(void);

// 初始化滤波器状态
typedef struct {
    double prev_output; // 上一个输出值
    double alpha;       // 滤波器系数
} LowPassFilter;

extern float twoKp;
extern float twoKi;

extern LowPassFilter filter_gyro;

float apply_lowpass_filter(LowPassFilter *filter, float input);
void init_lowpass_filter(LowPassFilter *filter, float cutoff_freq, float sample_rate);
#endif
