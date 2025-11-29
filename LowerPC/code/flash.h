/*
 * flash.h
 *
 *  Created on: 2024年6月19日
 *      Author: 554.
 */

#ifndef SRC_USER_FLASH_H_
#define SRC_USER_FLASH_H_

#include "zf_common_headfile.h"

// 联合体，用于将double拆分为两个float
union DoubleToFloat {
    double d;
    float f[2];
};

void flash_test(void);

double twoFloatsToDouble(float input1, float input2);
void doubleToTwoFloats(double input, float* output1, float* output2);

void ins_flash_read(void);
void ins_flash_write(void);

//void rtk_flash_read(void);
//void rtk_flash_write(void);


#endif /* SRC_USER_FLASH_H_ */
