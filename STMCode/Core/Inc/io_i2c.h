#ifndef __IO_I2C_H
#define __IO_I2C_H

#include "main.h"

//IO��������
// 
//#define SDA_IN()  {GPIOL->CRH&=0XFF0FFFFF;GPIOA->CRL|=(u32)8<<12;}    //PA5
//#define SDA_OUT() {GPIOL->CRH&=0XFF0FFFFF;GPIOA->CRL|=(u32)3<<12;}
 

//IO��������	 
//#define IIC_SCL    PBout(6) //SCL
//#define IIC_SDA    PBout(7) //SDA	 
//#define READ_SDA   PBin(7)  //����SDA 

#define IIC_SCL(n)    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,n) //SCL
#define IIC_SDA(n)    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,n) //SDA
#define READ_SDA      HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)   //PBin(7)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				        //����IIC��ʼ�ź�
void IIC_Stop(void);	  			      //����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			    //IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				      //IIC�ȴ�ACK�ź�
void IIC_Ack(void);			            //IIC����ACK�ź�
void IIC_NAck(void);				        //IIC������ACK�ź�

void SDA_IN(void);
void SDA_OUT(void);

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  

void delay_us(uint32_t nus);
#endif
