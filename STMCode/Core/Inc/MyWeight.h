/*
 * MyWeight.h
 *
 *  Created on: Dec 22, 2023
 *      Author: Dpon毛羊
 */

#ifndef INC_MYWEIGHT_H_
#define INC_MYWEIGHT_H_
#include "main.h"
extern float zeropoint;
extern float weighttime;
extern ADC_HandleTypeDef hadc;
float GetNowWeight(ADC_HandleTypeDef* hadc);

#endif /* INC_MYWEIGHT_H_ */
