#ifndef _GIMBAL_CONTROL_H_
#define _GIMBAL_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "Motor.h"
#define Yaw_Motor_OFFSET 5361u
#define Pitch_Motor_OFFSET 6228u
typedef enum 
{
	Fix,
  Follow,
	Auto
}Aim_Mode_e;

typedef struct
{
  float Yaw;
	float Pitch;
}Gimbal_Ref_t;

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
void Bullet_Load_Control(void);
extern Motor_t GMotor_Yaw;
extern Motor_t GMotor_Pitch;
extern Aim_Mode_e Aim_Mode;
extern Gimbal_Ref_t Gimbal_Ref;
#endif

