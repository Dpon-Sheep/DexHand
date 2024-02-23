/*
 * MyMotor.h
 *
 *  Created on: Dec 22, 2023
 *      Author: Dpon毛羊
 */

#ifndef INC_MYMOTOR_H_
#define INC_MYMOTOR_H_

#include "main.h"


float get_pos_goal_speed(int32_t now_pos,int16_t goal_pos);
uint8_t CAN_Send_Msg(CAN_HandleTypeDef* hcan,uint8_t *msg);
uint8_t get_motor_angle(CAN_HandleTypeDef* hcan);
uint8_t get_motor_info(CAN_HandleTypeDef* hcan);
uint8_t stop_Motor(CAN_HandleTypeDef* hcan);
uint8_t start_Motor(CAN_HandleTypeDef* hcan);
uint8_t clear_error(CAN_HandleTypeDef* hcan);
uint8_t set_Motor_Zero(CAN_HandleTypeDef* hcan);
uint8_t motor_Goto(CAN_HandleTypeDef* hcan,uint32_t pos);
uint8_t set_Motor_Speed(CAN_HandleTypeDef* hcan,float goal);

void CANFilter_init(CAN_HandleTypeDef* hcan);

#endif /* INC_MYMOTOR_H_ */
