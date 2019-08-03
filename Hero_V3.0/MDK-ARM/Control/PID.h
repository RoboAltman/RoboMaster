#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

typedef struct PID_t
{
	float position_ref;
	float ref;
	float position_fdb;
	float speed_fdb;
	float err[2];
	float output;
	float position_sum;
	float speed_sum;
	float position_acc;
	float speed_acc;
}PID_t;



typedef struct PID_Parameter_t
{
		float kp;
		float ki;
		float kd;
    float integralMax;
		float outputMax;
}PID_Parameter_t;


#endif
