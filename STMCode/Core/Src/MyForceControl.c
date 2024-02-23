/*
 * MyForceControl.c
 *
 *  Created on: Dec 25, 2023
 *      Author: Dpon毛羊
 */

#include "MyForceControl.h"
float Kp=1.05;
float Ki=0.0016;
float error_int=0.0;
float max_int=60;
float k_coff=0.08;

float get_goal_velocity(int16_t nowforce,int16_t goalforce)
{
	float sum=0;
	sum+=Kp*(goalforce-nowforce);
	error_int += Ki*(goalforce-nowforce);
	if(error_int > max_int) error_int = max_int;
	if(error_int < -max_int) error_int = -max_int;
	sum+=error_int;
	if(k_coff<0.05)
		k_coff = 0.05;
	return sum*0.09/k_coff;
}
