/*
 * init.h
 *
 *  Created on: 2024Äê1ÔÂ16ÈÕ
 *      Author: 554.
 */
#ifndef CODE_INIT_H_
#define CODE_INIT_H_
#include "zf_common_headfile.h"

extern uint8 beep_flag,ele_flag;
extern int8_t go_flag,cross_flag;
extern uint8 Directory;
extern uint8 TFT_Directory;
extern float ADCbattery;
extern uint8 ele;


void Init(void);
void Init_win(void);
void Directory_Init(void);
void Key_ControlM(void);
void TFT_Task(void);

void insertElement(int size, int index, int element);
void deleteElement(int size, int index);

#endif
