#include "SuperCAP.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal.h"
#include "dac.h"
#include "gpio.h"
#include "adc.h"
#include "Referee.h"

#define TIM_MAX 500

extern float total_power[2];				//[0]��ǰ�ܹ���ֵ | [1]ǰһ
extern float diff_t_p;

enum EX_STATE exstate=_BATTERY;
enum CAP_Ctrl CAP_CTRL,EX_CTRL=BATTERY;
enum CAP_Auto CAP_AUTO_CTRL = CAP_MANUAL;

uint16_t cap_current_value=0;
float current_i[2];
uint32_t cap_temp;
float cap_volt=0;

/***
	*�������ƣ�Battery
	*�������ܣ��л�����ع���״̬ 
	*��ڲ���: ��
	*����ֵ  ����
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
	*�������ƣ�ToCap
	*�������ܣ��л������ݹ���״̬ 
	*��ڲ���: ��
	*����ֵ  ����
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
	*�������ƣ�SetCAPCurrent
	*�������ܣ��趨���ݳ����� 
	*��ڲ���: float CAP_RI
	*����ֵ  ��
***/
void SetCAPCurrent(float CAP_RI)
{
	float current = 1280 * CAP_RI + 60;
	if(CAP_RI<=0.05f)	current=0;
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,current); 
}

/***
	*�������ƣ�GetCapVoltage
	*�������ܣ���ȡ���ݵ�ѹֵ 
	*��ڲ���: ��
	*����ֵ  ���� 
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
	*�������ƣ�SuperCapCtrl
	*�������ܣ����������Զ����Ʋ���
	*��ڲ���: ��
	*����ֵ  ���� 
***/
void SuperCapCtrl(void)
{
	if(CAP_AUTO_CTRL==CAP_AUTO)
	{//�Զ�����-start
		if(exstate==_BATTERY)
		{//�����ǵ�ع���
			if(PowerHeatData.chassisPower<=80)
			{//����ϵͳ���ʵ���80W -�� ��ع���
				ToBattery();	//�л���ع���
				if(PowerHeatData.chassisPower<70)
				{//������̹��ʵ���70W����ȫ��Χ�� -���ø�ԣ���ʸ����ݳ�磨�㹦�ʣ�
					float temp_i=0;
					if(cap_volt>5)	//���ݵ�ѹ����5Vʱ
					{
						temp_i=(75-PowerHeatData.chassisPower+cap_volt*current_i[0])*CAP_I_OFFSET/(cap_volt);//���������
					}	
					else			//���ݵ�ѹ����5Vʱ�����������
					{
						temp_i=1.3f;
					}							
						
					if(temp_i>1.3f)
					{//1.3A����
						temp_i=1.3f;
					}
					else if(temp_i<=0)	
					{
						temp_i=0;
					}
					SetCAPCurrent(temp_i);//����������
				}
				else
				{
					SetCAPCurrent(0);//��ԣ���ʵ��ڰ�ȫֵ�������
				}
			}
			else if((diff_t_p>=1.75f & total_power[0]>=40.0f)|(total_power[0]>=75.0f))
			{	//����ϵͳ���ʸ���80W -���л��������ݹ��� PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
				SetCAPCurrent(0);	//�ر�DC-DCЭͬ����
				if(cap_volt>22)
				{	//�жϵ����Ƿ���ڰ�ȫֵ ����һ�η��������ѹΪ����
					ToCap();//�л����ݹ���
					//SetCAPCurrent(2.5);//����DC-DCЭͬ����
				}
				else
				{
					ToBattery();//�������û�磬��ά�ֵ�ع���
					//PS:��������������5Jʱ�����̹��ʻᱻ������80Wһ��
				}
			}
		}
		else if(exstate==_CAP)
		{
			static int tim_count;
			if(tim_count<TIM_MAX)	
			{
				tim_count++;//�������ݹ���֮�󣬿�����ʱ����
			}
			else	
			{
				tim_count=TIM_MAX;//TIM_MAX ms��������������
			}
			SetCAPCurrent(0.0f*tim_count/TIM_MAX); 
			
			if((cap_volt<20.1f) | (diff_t_p<=1.5f & total_power[0]<=70.0f))
			{//���ݵ�ѹ���ڰ�ȫֵ �� �����������С��2.5A(�������ֵ��Ҫ���ݲο��������ٵ���)  | value_res[2]<3250
				SetCAPCurrent(0);//�ر�DC-DC����
				ToBattery();//�������û�磬�����Ϊ��ع���
				tim_count=0;
			}
		}
	}//�Զ�����-end
}

