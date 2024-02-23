/*
 * MyMotor.c
 *
 *  Created on: Dec 22, 2023
 *      Author: Dpon毛羊
 */

#include "MyMotor.h"
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox=0;
uint8_t can_r[8];
uint8_t can_t[8];
extern uint16_t encoderValue;

int16_t now_current=0;
int16_t now_speed;
int32_t now_pos;
uint16_t now_tick;
uint16_t pos0_r;
int32_t now_r;
extern int motor_response_flag;

const int16_t MAX_POS_SPEED = 1600;
float get_pos_goal_speed(int32_t now_pos,int16_t goal_pos)
{
	int16_t delta_pos = goal_pos - now_pos/100;
	if(delta_pos * 25> MAX_POS_SPEED)
		return MAX_POS_SPEED;
	if(delta_pos * 25< - MAX_POS_SPEED)
		return -MAX_POS_SPEED;
	return delta_pos * 25;
}

uint8_t set_Motor_Speed(CAN_HandleTypeDef* hcan,float goal)
{
	  int32_t goalspeed = (int32_t)(goal*100);
	  can_t[0] = 0xA2;
	  can_t[1] = 0x00;
	  can_t[2] = 0x00;
	  can_t[3] = 0x00;
	  can_t[4] = (goalspeed>>0)&0xff;
	  can_t[5] = (goalspeed>>8)&0xff;
	  can_t[6] = (goalspeed>>16)&0xff;
	  can_t[7] = (goalspeed>>24)&0xff;

	  return CAN_Send_Msg(hcan,can_t);
}
uint8_t motor_Goto(CAN_HandleTypeDef* hcan,uint32_t pos)
{
	  can_t[0] = 0xA4;
	  can_t[1] = 0x00;
	  can_t[2] = 0x00;
	  can_t[3] = 0x04;
	  can_t[4] = (pos>>0)&0xff;
	  can_t[5] = (pos>>8)&0xff;
	  can_t[6] = (pos>>16)&0xff;
	  can_t[7] = (pos>>24)&0xff;

	  return CAN_Send_Msg(hcan,can_t);
}
void clear_can_t()
{
	for(int i=0;i<8;i++)
		can_t[i]=0x00;
}
uint8_t get_motor_info(CAN_HandleTypeDef* hcan)
{
	  clear_can_t();
	  can_t[0] = 0x9C;
	  return CAN_Send_Msg(hcan,can_t);
}
uint8_t get_motor_angle(CAN_HandleTypeDef* hcan)
{
	clear_can_t();
	can_t[0] = 0x92;
	return CAN_Send_Msg(hcan,can_t);
}
uint8_t stop_Motor(CAN_HandleTypeDef* hcan)
{
	  clear_can_t();
	  can_t[0] = 0x81;
	  return CAN_Send_Msg(hcan,can_t);
}
uint8_t start_Motor(CAN_HandleTypeDef* hcan)
{
	  clear_can_t();
	  can_t[0] = 0x88;
	  return CAN_Send_Msg(hcan,can_t);
}
uint8_t clear_error(CAN_HandleTypeDef* hcan)
{
	clear_can_t();
	can_t[0] = 0x9B;
	return CAN_Send_Msg(hcan,can_t);
}
uint8_t set_Motor_Zero(CAN_HandleTypeDef* hcan)// restarting motor is needed to take effect
{
	clear_can_t();
	can_t[0] = 0x19;
	return CAN_Send_Msg(hcan,can_t);
}
uint8_t CAN_Send_Msg(CAN_HandleTypeDef* hcan,uint8_t *msg){
	TxHeader.StdId = 0x140 + 0x01;
	TxHeader.ExtId = 0x00;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
    int8_t ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, msg, &TxMailbox);
    if(ret != HAL_OK)
    	return 1;
    int failtime=0;
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
    {
    	failtime++;
    	if(failtime>100)
    		return 1;
    }
    return 0;
}
void CANFilter_init(CAN_HandleTypeDef* hcan)
{
	// configure the CAN filter
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x140 << 5;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0xFC00;
//	sFilterConfig.FilterMaskIdLow = 0x0006;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 0;
	if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
	    Error_Handler();
	}
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, can_r) != HAL_OK)
  {
	  /* Reception Error */
	  Error_Handler();
  }
  if ((RxHeader.StdId == (0x140 + 0x01)) && (RxHeader.DLC == 8))
  {
	  /* get encoder value */
	  if(can_r[0]==motor_response_flag)
		  motor_response_flag = 0;
	  if(can_r[0] == 0xA2 || can_r[0] == 0xA3 || can_r[0] == 0xA4 || can_r[0]==0x9C)
	  {
		  uint16_t current_data = (can_r[3] << 8) + can_r[2];
		  uint16_t speed_data = (can_r[5] << 8) + can_r[4];
		  uint32_t pos_data = (can_r[7] << 8) + can_r[6];
		  now_speed = (int16_t)speed_data;
		  now_current = ((int16_t)current_data)*16;
		  uint16_t pre_now_pos = (uint16_t)(pos_data*1125 / 2048);
		  if(pre_now_pos> 20000 + now_tick)
			  now_r--;
		  else if(pre_now_pos+ 20000 < now_tick)
			  now_r++;
		  now_tick = pre_now_pos;
		  now_pos = (now_tick-pos0_r) + now_r*36000;
	  }
//	  else if(can_r[0]==0x92)
//	  {
//		  uint64_t pos0_a_data = can_r[7];
//		  for(int i=6;i>=1;i--)
//		  {
//			  pos0_a_data<<=8;
//			  pos0_a_data +=can_r[i];
//		  }
//		  pos0_a = (int64_t)pos0_a_data;
//	  }
  }
}
