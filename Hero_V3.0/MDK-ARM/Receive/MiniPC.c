#include "MiniPC.h"
FormatTrans32 pctrans32;
FormatTrans16 pctrans16;
MiniPC_t MiniPC_42mm;
MiniPC_t MiniPC_17mm;
Aim_State_e Aim_State_42mm;
Aim_State_e Aim_State_17mm;
Aim_State_e Last_Aim_State_42mm;
Aim_State_e Last_Aim_State_17mm;
float Filter1[21];
volatile int Bullet_Num_Initnum;
volatile Bullet_Num_Init_e Bullet_Num_Init;
float MiniPC_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}
void Mini_PC_Data_Decode(uint8_t *pData)
{
	float sum;
	int i,j;
	for(i = 0 ; i < 31 ; i++)
	{
		if(pData[i] == 0xA5 && pData[i+1] == 0x5A)
		{
			pctrans32.u8_temp[0] = pData[i+2];
			pctrans32.u8_temp[1] = pData[i+3];
			pctrans32.u8_temp[2] = pData[i+4];
			pctrans32.u8_temp[3] = pData[i+5];
			MiniPC_42mm.Pitch = pctrans32.float_temp;
			pctrans32.u8_temp[0] = pData[i+6];
			pctrans32.u8_temp[1] = pData[i+7];
			pctrans32.u8_temp[2] = pData[i+8];
			pctrans32.u8_temp[3] = pData[i+9];
			MiniPC_42mm.Yaw   = pctrans32.float_temp;
			
			MiniPC_42mm.Yaw = MiniPC_Filter(MiniPC_42mm.Yaw,0.2);
			
			pctrans32.u8_temp[0] = pData[i+10];
			pctrans32.u8_temp[1] = pData[i+11];
			pctrans32.u8_temp[2] = pData[i+12];
			pctrans32.u8_temp[3] = pData[i+13];
			MiniPC_42mm.Distance = pctrans32.float_temp/1000.0f;
			Last_Aim_State_42mm = Aim_State_42mm;
			Aim_State_42mm    =  pData[i+14];
			pctrans16.u8_temp[0] = pData[i+15];
			pctrans16.u8_temp[1] = pData[i+16];
      Bullet_Num_Initnum = pctrans16.u16_temp;
			Bullet_Num_Init = Init_OK;
			pctrans32.u8_temp[0] = pData[i+17];
			pctrans32.u8_temp[1] = pData[i+18];
			pctrans32.u8_temp[2] = pData[i+19];
			pctrans32.u8_temp[3] = pData[i+20];
			MiniPC_17mm.Pitch = pctrans32.float_temp;
			pctrans32.u8_temp[0] = pData[i+21];
			pctrans32.u8_temp[1] = pData[i+22];
			pctrans32.u8_temp[2] = pData[i+23];
			pctrans32.u8_temp[3] = pData[i+24];
			MiniPC_17mm.Yaw   = pctrans32.float_temp;
      pctrans32.u8_temp[0] = pData[i+25];
			pctrans32.u8_temp[1] = pData[i+26];
			pctrans32.u8_temp[2] = pData[i+27];
			pctrans32.u8_temp[3] = pData[i+28];
			MiniPC_17mm.Distance = pctrans32.float_temp/1000.0f;
			Last_Aim_State_17mm = Aim_State_17mm;
			Aim_State_17mm = pData[i+29];
			
		}	
	}
}
