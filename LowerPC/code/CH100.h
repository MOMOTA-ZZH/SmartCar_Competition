/*
 * CH100.h
 *
 *  Created on: 2024Äê10ÔÂ19ÈÕ
 *      Author: 554.
 */

#ifndef CODE_CH100_H_
#define CODE_CH100_H_
#include "zf_common_headfile.h"

#define CH100_ON           (1)

extern float gyro6[3],gyro1[3],acc[3],Pitch6,Roll6,Yaw6;
void CH100_Read_date(uint8_t bufCH100);

#endif /* CODE_CH100_H_ */
