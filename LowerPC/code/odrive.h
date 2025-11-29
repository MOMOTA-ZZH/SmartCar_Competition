/*
 * odrive.h
 *
 *  Created on: 2024Äê9ÔÂ15ÈÕ
 *      Author: 554
 */

#ifndef CODE_ODRIVE_H_
#define CODE_ODRIVE_H_

#include "zf_common_headfile.h"

#define odrive_UART        (UART_2)
#define odrive_encoder2      (0)


extern uint8_t odrive_encoder;
extern float encoder_bldc,encoder_rear,bdc_speed,bdc_speed_max,bdc_speed_max_set;

#define direct_bdc_speed(x)    \
    do {                       \
        bdc_speed_max_set = x; \
        bdc_speed = x;         \
        bdc_speed_max = x;     \
    } while (0)


void odrive_speed_ctl(unsigned char num, float speed);
void odrive_torque_ctl(unsigned char num, float torque);
void odrive_feedback(unsigned char num);
void odrive_Anl(char *p);
void odrive_analyze_speed(void);

#endif /* CODE_ODRIVE_H_ */
