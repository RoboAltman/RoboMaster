#ifndef _GIMBAL_CONTROL_H_
#define _GIMBAL_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "PID.h"
#include "Chassis_Control.h"
#include "Transmit.h"
#include "Receive.h"
#include "Remote_Control.h"
#include "Motor.h"
#define Middle_Completed 3
#define Left_Completed 2
#define Right_Completed 1
#define unCompleted 0
#define Yaw_Motor_OFFSET 4289
#define Pitch_Motor_OFFSET 0
extern Motor_t GMotor_Pitch;
extern Motor_t GMotor_Yaw;
extern Motor_t GMotor_FPV;
extern float Yaw_IMU_OFFSET;
typedef enum 
{
	Chassis_Follow_Gimbal,
	Gimbal_Follow_Chassis
}Follow_Mode_e;
typedef enum
{ 
	NO_Forecast,
	Sentry_Mode,
	Infantry_Mode
}Forecast_Mode_e;


typedef struct
{
/*****?????? (TD)*******/
float v1;//????????
float v2;//???????????
float r;//????
float h;//ADRC??????
float N0;//???????????h0=N*h
 
float h0;
float fh;//??????????
} TD;

typedef struct
{
	uint32_t lastTick;
	
	uint32_t tick;
	
	uint8_t isFirst;
	
	float wYaw[40];
	
	float wPitch[40];
	
	float avgWYaw;
	
	float avgWPitch;
	
	float lastAbsYaw;
	
	float lastAbsPitch;
	
	float yawFore;
	
	float pitchFore;
}GimbalFilter_t;
extern GimbalFilter_t GimbalFilter ;




void Gimbal_Control(void);
void Yaw_Init(void);
void TD_Init(TD *td, float r, float h, float n0);
extern int Yaw_State;
extern GimbalRef_t GimbalRef;
extern Follow_Mode_e Follow_Mode;
extern Follow_Mode_e Last_Follow_Mode;
extern Forecast_Mode_e Forecast_Mode;
extern TD td;
extern TD td1;
#endif



