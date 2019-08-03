#ifndef _Motor_H_
#define _Motor_H_
#include "stm32f4xx_hal.h"
typedef struct
{
	float refpos;
	float refpos_soft;
	float relapos;
	float rawpos;
	float lastrawpos;
	float round;
	float accpos;
	
	float acc;
	float dec;
	
	float sum;
	float output;
	
	float err;
	float lasterr;
	float derr;
	float lastderr;
}PosCtrl_t;

typedef struct
{
	float refvel;
	float refvel_soft;
	float rawvel;
	float lastrawvel;
		
	float acc;
	float dec;
	
	float sum;
	float err;
	float lasterr;
	float derr;
	float lastderr;
	float output;
	
}VelCtrl_t;

typedef struct
{
	float angle;
	int16_t speed;
	int16_t current;
	float lastangle;
}EncoderCtrl_t;

typedef struct
{
	VelCtrl_t velCtrl;
	
	PosCtrl_t posCtrl;
	
	EncoderCtrl_t encoderCtrl;
}Motor_t;
#endif
