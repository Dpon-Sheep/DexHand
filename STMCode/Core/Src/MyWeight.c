/*
 * MyWeight.c
 *
 *  Created on: Dec 22, 2023
 *      Author: Dpon毛羊
 */

#include "MyWeight.h"
float a[4]={1,-2.87435689267748,2.75648319522570,-0.881893130592486};
float b[4]={2.91464944656705e-05,8.74394833970116e-05,8.74394833970116e-05,2.91464944656705e-05};
//float a[4]={1,-2.93717072844989,2.87629972347933,-0.939098940325283};
//float b[4]={3.75683801978610e-06,1.12705140593583e-05,1.12705140593583e-05,3.75683801978610e-06};
float lastx[3];
float lasty[3];
float zeropoint=29.0;
float weighttime=0.9099;//0.9099,0.6275
float GetNowWeight(ADC_HandleTypeDef* hadc)
{
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
  int16_t ADC_value = (int16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);

  float y = b[0]*ADC_value;
  for(int i=0;i<3;i++)
  {
	  y+=b[i+1]*lastx[i];
	  y-=a[i+1]*lasty[i];
  }
  lastx[2]=lastx[1];lastx[1]=lastx[0];lastx[0]=ADC_value*1.0;
  lasty[2]=lasty[1];lasty[1]=lasty[0];lasty[0]=y;
//  float y=ADC_value*1.0;

  float nowforce = ((y-zeropoint)*weighttime);
  return nowforce;
}
