#include "SuperCAP.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal.h"
#include "dac.h"
#include "gpio.h"
#include "adc.h"
#include "Referee.h"

#define TIM_MAX 500

extern float total_power[2];				//[0]当前总功率值 | [1]前一
extern float diff_t_p;

enum EX_STATE exstate=_BATTERY;
enum CAP_Ctrl CAP_CTRL,EX_CTRL=BATTERY;
enum CAP_Auto CAP_AUTO_CTRL = CAP_MANUAL;

uint16_t cap_current_value=0;
float current_i[2];
uint32_t cap_temp;
float cap_volt=0;

/***
	*函数名称：Battery
	*函数功能：切换到电池供电状态 
	*入口参数: 无
	*返回值  ：无
***/
void ToBattery(void)
{
	if(exstate == _CAP)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		//HAL_Delay(8);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		exstate = _BATTERY;
	}
	else
	{
		//do nothing
	}
}

/***
	*函数名称：ToCap
	*函数功能：切换到电容供电状态 
	*入口参数: 无
	*返回值  ：无
***/
void ToCap(void) 
{
	if(exstate == _BATTERY)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		//HAL_Delay(8);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		exstate = _CAP;
	}
	else
	{
		//do nothing
	}
}

/***
	*函数名称：SetCAPCurrent
	*函数功能：设定电容充电电流 
	*入口参数: float CAP_RI
	*返回值  ：
***/
void SetCAPCurrent(float CAP_RI)
{
	float current = 1280 * CAP_RI + 60;
	if(CAP_RI<=0.05f)	current=0;
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,current); 
}

/***
	*函数名称：GetCapVoltage
	*函数功能：读取电容电压值 
	*入口参数: 无
	*返回值  ：无 
***/
void GetCapVoltage(void)
{
		//HAL_ADC_Start(&hadc1);
		//HAL_ADC_PollForConversion(&hadc1, 30);
		//cap_temp = HAL_ADC_GetValue(&hadc1);
		cap_temp=value_res[0];
		cap_volt = (float)cap_temp*26/2720-0.65f;
}

/***
	*函数名称：SuperCapCtrl
	*函数功能：超级电容自动控制策略
	*入口参数: 无
	*返回值  ：无 
***/
void SuperCapCtrl(void)
{
	if(CAP_AUTO_CTRL==CAP_AUTO)
	{//自动控制-start
		if(exstate==_BATTERY)
		{//现在是电池供电
			if(PowerHeatData.chassisPower<=80)
			{//裁判系统功率低于80W -》 电池供电
				ToBattery();	//切换电池供电
				if(PowerHeatData.chassisPower<70)
				{//如果底盘功率低于70W（安全范围） -》用富裕功率给电容充电（恒功率）
					float temp_i=0;
					if(cap_volt>5)	//电容电压高于5V时
					{
						temp_i=(75-PowerHeatData.chassisPower+cap_volt*current_i[0])*CAP_I_OFFSET/(cap_volt);//计算充电电流
					}	
					else			//电容电压低于5V时，最大功率运行
					{
						temp_i=1.3f;
					}							
						
					if(temp_i>1.3f)
					{//1.3A限流
						temp_i=1.3f;
					}
					else if(temp_i<=0)	
					{
						temp_i=0;
					}
					SetCAPCurrent(temp_i);//开启充电电流
				}
				else
				{
					SetCAPCurrent(0);//富裕功率低于安全值，不充电
				}
			}
			else if((diff_t_p>=1.75f & total_power[0]>=40.0f)|(total_power[0]>=75.0f))
			{	//裁判系统功率高于80W -》切换超级电容供电 PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
				SetCAPCurrent(0);	//关闭DC-DC协同供电
				if(cap_volt>22)
				{	//判断电容是否高于安全值 ：以一次飞坡所需电压为依据
					ToCap();//切换电容供电
					//SetCAPCurrent(2.5);//开启DC-DC协同供电
				}
				else
				{
					ToBattery();//如果电容没电，则维持电池供电
					//PS:当功率余量低于5J时，底盘功率会被限制在80W一下
				}
			}
		}
		else if(exstate==_CAP)
		{
			static int tim_count;
			if(tim_count<TIM_MAX)	
			{
				tim_count++;//开启电容供电之后，开启定时计数
			}
			else	
			{
				tim_count=TIM_MAX;//TIM_MAX ms后电流提升至最大
			}
			SetCAPCurrent(0.0f*tim_count/TIM_MAX); 
			
			if((cap_volt<20.1f) | (diff_t_p<=1.5f & total_power[0]<=70.0f))
			{//电容电压低于安全值 或 底盘输出电流小于2.5A(具体电流值需要根据参考兵种匀速电流)  | value_res[2]<3250
				SetCAPCurrent(0);//关闭DC-DC供电
				ToBattery();//如果电容没电，则更改为电池供电
				tim_count=0;
			}
		}
	}//自动控制-end
}

