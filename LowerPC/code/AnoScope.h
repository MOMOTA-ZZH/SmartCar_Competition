/*
 * AnoScope.h
 *
 *  Created on: 2024年2月18日
 *      Author: 554.
 */
#ifndef SRC_APPSW_TRICORE_USER_ANO_DT_H_
#define SRC_APPSW_TRICORE_USER_ANO_DT_H_

#include "zf_common_headfile.h"

/* 联合体（方便数据转换） */
typedef union {
float f;
int i;
}type_cast_t;

#define BYTE0(dwTemp)  (*(char *)(&dwTemp))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp) + 3))//发送用

#define COMPOUND_8BIT(data)         (*(data-0) & 0xFF)

#define COMPOUND_16BIT(data)        (((*(data-0) << 8) & 0xFF00) | \
                                     (*(data-1) & 0x00FF))

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))    //接收用

extern uint8_t data_to_Read[50];//数据数组

void ANO_DT_send(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8);
void ANO_DT_Anl(void);
void ANO_DT_Read(void);//匿名上位机
void ANO_DT_float4(float data1, float data2, float data3, float data4);

void ELE_send6(short data1, short data2, short data3, short data4, short data5, short data6);
void ELE_send(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8);
void ELE_Anl(void);
void ELE_Read(uint8_t buf0);//蓝牙协议

#endif
