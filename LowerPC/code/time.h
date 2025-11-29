/*
 * time.h
 *
 *  Created on: 2024年1月16日
 *      Author: 554.
 */
#ifndef CODE_TIME_H_
#define CODE_TIME_H_
#include "zf_common_headfile.h"
typedef struct {
    unsigned char T_2ms;
    unsigned char T_6ms;
    unsigned char T_10ms;
    unsigned char T_20ms;
    unsigned char T_30ms;
    unsigned char T_100ms;
    unsigned char T_1s;
}Timer_0;
extern Timer_0 Flag;

extern float pidout_1f, pidout_2f, pidout_3f,//动量轮pid输出值
pidout_1a, pidout_2a, pidout_3a;//行进轮pid输出值
extern float increase_xj,increase_dj;

void Fuse_result(void);

#endif
