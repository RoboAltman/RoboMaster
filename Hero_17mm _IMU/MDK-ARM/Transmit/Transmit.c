#include "Transmit.h"
#include "Gimbal_Control.h"
#include "Shoot_Control.h"
#include "can.h"

void CAN_Send_Msg(void)
{   	 
	  hcan1.pTxMsg->StdId = 0x1FF;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->DLC = 0x08;
		
		hcan1.pTxMsg->Data[0] = (uint8_t)((int16_t)GMotor_Yaw.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[1] = (uint8_t)GMotor_Yaw.velCtrl.output;
		hcan1.pTxMsg->Data[2] = (uint8_t)((int16_t)GMotor_Pitch.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[3] = (uint8_t)GMotor_Pitch.velCtrl.output;
		hcan1.pTxMsg->Data[4] = (uint8_t)((int16_t)BLMotor.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[5] = (uint8_t)BLMotor.velCtrl.output;	
		HAL_CAN_Transmit(&hcan1, 10);
}

