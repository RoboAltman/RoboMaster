#ifndef __SUPER_CAP__
#define __SUPER_CAP__

#include "stm32f4xx_hal.h"

#define CAP_I_OFFSET 0.5f//恒压恒流功率传输效率

void ToBattery(void);
void ToCap(void);//调用前需要先声明状态（exstate） 

void SetCAPCurrent(float CAP_RI);
void GetCapVoltage(void);
void SuperCapCtrl(void);
//void StateJudge(void);

extern uint32_t cap_temp;
extern float cap_volt;
extern uint16_t cap_current_value;
extern float current_i[2];
extern uint16_t current_base[2];

enum EX_STATE{_CAP=0,_BATTERY=1};
enum CAP_Ctrl{BATTERY=0,CHARGE=1,DISCHARGE=2,CAP=3,TO_CAP=4,TO_BATTERY=5};
enum CAP_Auto{CAP_MANUAL=0,CAP_AUTO=1};

extern enum EX_STATE exstate;
extern enum CAP_Ctrl EX_CTRL;
extern enum CAP_Ctrl CAP_CTRL;
extern enum CAP_Auto CAP_AUTO_CTRL;



#endif
