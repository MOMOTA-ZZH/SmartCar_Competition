/*
 * MPU6050_DMP.h
 *
 *  Created on: 2024年1月18日
 *      Author: 554.
 */
#ifndef _MPU6050_DMP_
#define _MPU6050_DMP_

#include "zf_common_headfile.h"

extern short   gyro[3], accel[3], sensors;
extern float   Pitch, Roll,Yaw;
unsigned short inv_row_2_scale(const signed char *row);

int DMP_Init(void);
int DMP_Read(void);

// 初始化使用
int mpu_set_gyro_fsr(unsigned short fsr);
int mpu_set_accel_fsr(unsigned char fsr);
int mpu_set_lpf(unsigned short lpf);
int mpu_set_sample_rate(unsigned short rate);
int dmp_set_orientation(unsigned short orient);
int mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *da);
int mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *da);
int mpu_reset_fifo_in_dmp_enable_feature(void);
int mpu_reset_fifo_in_mpu_set_dmp_state(void);

// 读值使用
int dmp_read_fifo(short *gyro, short *accel, long *quat, short *sensors, unsigned char *more);

#endif
