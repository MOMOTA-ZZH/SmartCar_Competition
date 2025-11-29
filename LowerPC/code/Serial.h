/*
 * Serial.c
 *
 *  Created on: 2024Äê9ÔÂ22ÈÕ
 *      Author: 554.
 */
#ifndef __SERIAL_H
#define __SERIAL_H

#include "zf_common_headfile.h"

#define HEAD1   0xA5
#define TAil   0x5A

extern int  Bias_Line;
extern int8  flag_state,R_L_Flag;

void Data_Receive_Analysis(uint8_t RxData);
void Serial_init(void);

#endif
