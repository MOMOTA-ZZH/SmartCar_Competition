/*
 * lane_change.h
 *
 *  Created on: 2024Äê10ÔÂ1ÈÕ
 *      Author: 554.
 */

#ifndef CODE_LANE_CHANGE_H_
#define CODE_LANE_CHANGE_H_

#include "zf_common_headfile.h"

#define R_L_ON       (0)

extern float lane_change_L,lane_change_R;

void normalizeYaw(float *yaw);
void lane_change_Start(int x);
void lane_change_ON(void);

#endif /* CODE_LANE_CHANGE_H_ */
