#include "Board_I_TX.h"
#include "CRC.h"
#include "string.h"
uint8_t Board_I_Data[26];
uint8_t Board_I_Data_Temp[60];
uint8_t Board_CRC_check[24];
uint16_t CRC16_temp;
Data_17mm_t Data_17mm;
Gimbal_42mm_t Gimbal_42mm;
MiniPC_t MiniPC_17mm;
uint8_t Aim_State;
uint8_t Last_Aim_State_17mm;
uint8_t Shoot_Enable;
int i;
float Pitch_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}


void Board_I_Decode(uint8_t *pData)
{
//	if(pData[0] == 0xA5 && pData[1] == 0x5A)
//	{
	for(i=0 ; i<34 ; i++)
	{
	  pData[i] = 0xA5;
	  pData[i+1] = 0x5A;
		memcpy(Board_CRC_check,pData+i,24);
	
//
	  memcpy(&CRC16_temp,pData+24+i,2);
	if(CRC16_temp == Get_CRC16_Check_Sum(Board_CRC_check,24,0xFFFF))
	{
		memcpy(Board_I_Data,pData+i,26);
    //memcpy(&MiniPC_17mm,Board_I_Data+2,12);
		//memcpy(&Aim_State,Board_I_Data+14,1);
		memcpy(&Gimbal_42mm,Board_I_Data+15,8);
		memcpy(&Shoot_Enable,Board_I_Data+23,1);
	}
	}
//	}
}

void Mini_PC_Data_Decode(uint8_t *pData)
{
	float sum;
	int i,j;
	for(i = 0 ; i < 31 ; i++)
	{
		if(pData[i] == 0xA5 && pData[i+1] == 0x5A)
		{
//			pctrans32.u8_temp[0] = pData[i+2];
//			pctrans32.u8_temp[1] = pData[i+3];
//			pctrans32.u8_temp[2] = pData[i+4];
//			pctrans32.u8_temp[3] = pData[i+5];
//			MiniPC_42mm.Pitch = pctrans32.float_temp;
//			pctrans32.u8_temp[0] = pData[i+6];
//			pctrans32.u8_temp[1] = pData[i+7];
//			pctrans32.u8_temp[2] = pData[i+8];
//			pctrans32.u8_temp[3] = pData[i+9];
//			MiniPC_42mm.Yaw   = pctrans32.float_temp;
//			//MiniPC
//			pctrans32.u8_temp[0] = pData[i+10];
//			pctrans32.u8_temp[1] = pData[i+11];
//			pctrans32.u8_temp[2] = pData[i+12];
//			pctrans32.u8_temp[3] = pData[i+13];
//			MiniPC_42mm.Distance = pctrans32.float_temp/1000.0f;
//			Last_Aim_State_42mm = Aim_State_42mm;
//			Aim_State_42mm    =  pData[i+14];
//			pctrans16.u8_temp[0] = pData[i+15];
//			pctrans16.u8_temp[1] = pData[i+16];
//      Bullet_Num_Initnum = pctrans16.u16_temp;
//			Bullet_Num_Init = Init_OK;
			memcpy(&MiniPC_17mm,pData+17+i,12);
			MiniPC_17mm.Distance = MiniPC_17mm.Distance/1000.0f;
      Last_Aim_State_17mm = Aim_State;
			memcpy(&Aim_State,pData+29+i,1);
			Pitch_Filter(MiniPC_17mm.Pitch,0.9);
//			pctrans32.u8_temp[0] = pData[i+17];
//			pctrans32.u8_temp[1] = pData[i+18];
//			pctrans32.u8_temp[2] = pData[i+19];
//			pctrans32.u8_temp[3] = pData[i+20];
//			MiniPC_17mm.Pitch = pctrans32.float_temp;
//			pctrans32.u8_temp[0] = pData[i+21];
//			pctrans32.u8_temp[1] = pData[i+22];
//			pctrans32.u8_temp[2] = pData[i+23];
//			pctrans32.u8_temp[3] = pData[i+24];
//			MiniPC_17mm.Yaw   = pctrans32.float_temp;
//      pctrans32.u8_temp[0] = pData[i+25];
//			pctrans32.u8_temp[1] = pData[i+26];
//			pctrans32.u8_temp[2] = pData[i+27];
//			pctrans32.u8_temp[3] = pData[i+28];
//			MiniPC_17mm.Distance = pctrans32.float_temp/1000.0f;
//			Last_Aim_State_17mm = Aim_State_17mm;
//			Aim_State_17mm = pData[i+29];
			
		}	
	}
}