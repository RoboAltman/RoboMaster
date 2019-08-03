#ifndef _TRANSMIT_H_
#define _TRANSMIT_H_
#include "stm32f4xx_hal.h"
#include "can.h"
#define BSP_USART6_DMA_TX_BUF_LEN 30u 

extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern uint8_t Armor_Chose;

void SM_CAN_Send_Msg(void); 
void BL_CAN_Send_Msg(void);
void CM_CAN_Send_Msg(void);
void GM_CAN_Send_Msg(void);
void BP_CAN_Send_Msg(void);
void FPV_CAN_Send_Msg(void);
void DataScope_Transmit(void);
void Client_Transmit(void);
void Mini_PC_Transmit(void);
void Data_17mm_Transmit(void);
#endif



