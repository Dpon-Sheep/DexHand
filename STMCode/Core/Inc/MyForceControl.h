/*
 * MyForceControl.h
 *
 *  Created on: Dec 25, 2023
 *      Author: Dpon毛羊
 */

#ifndef INC_MYFORCECONTROL_H_
#define INC_MYFORCECONTROL_H_

#include "main.h"

extern float Kp;
extern float Ki;
extern float max_int;
extern float k_coff;

float get_goal_velocity(int16_t nowforce,int16_t goalforce);

#endif /* INC_MYFORCECONTROL_H_ */
