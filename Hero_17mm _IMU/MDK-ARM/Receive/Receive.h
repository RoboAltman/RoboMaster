#ifndef _RECEIVE_H_
#define _RECEIVE_H_
#include "stm32f4xx_hal.h"
#define BSP_USART1_DMA_RX_BUF_LEN 30u
#include "Motor.h"
#include "can.h"
void CAN_Receive_Msg(CAN_HandleTypeDef* hcan);
void Board_I_Receive(void);
void Board_I_Data_Receive_Start(void);
void Mini_PC_Data_Receive(void);
void Mini_PC_Data_Receive_Start(void);
void IMU_Data_Receive_Start(void);
void HI_IMU_Data_Receive(void);
#endif

