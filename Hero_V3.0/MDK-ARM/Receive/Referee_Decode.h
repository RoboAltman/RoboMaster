#ifndef _REFEREE__DECODE_H
#define _REFEREE__DECODE_H
#include "stm32f4xx_hal.h"

#define BSP_USART6_DMA_RX_BUF_LEN 128u
#define BSP_USART6_DMA_TX_BUF_LEN 30u

#define FRAME_HEADER_LEN	sizeof(FrameHeader_t)
#define CMD_ID_LEN					2u	//命令码长度
#define CRC16_LEN					2u	//CRC16

/* 裁判系统数据段长度 */
#define GAME_STATE_LEN					3u	
#define GAME_RESULT_LEN					1u
#define GAME_ROBOTSURVIVORS_LEN			2u	
#define EVENT_DATA_LEN					4u
#define SUPPLY_PROJECTILE_ACTION_LEN	3u
#define SUPPLY_PROJECTILE_BOOKING_LEN	2u
#define GAME_ROBOT_STATE_LEN			15u
#define POWER_HEAT_DATA_LEN				14u
#define GAME_ROBOT_POS_LEN				16u
#define BUFF_MUSK_LEN					1u
#define AERIAL_ROBOT_ENERGY_LEN			3u
#define ROBOT_HURT_LEN					1u
#define SHOOT_DATA_LEN					6u

#define FRAME_HEADER_SOF		0xA5

#define GAME_STATE_CMD_ID					0x0001
#define GAME_RESULT_CMD_ID					0x0002
#define GAME_ROBOTSURVIVORS_CMD_ID			0x0003
#define EVENT_DATA_CMD_ID					0x0101
#define SUPPLY_PROJECTILE_ACTION_CMD_ID		0x0102
#define SUPPLY_PROJECTILE_BOOKING_CMD_ID	0x0103
#define GAME_ROBOT_STATE_CMD_ID				0x0201
#define POWER_HEAT_DATA_CMD_ID				0x0202
#define GAME_ROBOT_POS_CMD_ID				0x0203
#define BUFF_MUSK_CMD_ID					0x0204
#define AERIAL_ROBOT_ENERGY_CMD_ID			0x0205
#define ROBOT_HURT_CMD_ID					0x0206
#define SHOOT_DATA_CMD_ID					0x0207
#define INTERACTIVE_DATA_CMD_ID				0x0301

typedef __packed struct
{
	uint8_t		sof;			//帧起始字节
	uint16_t	dataLength;		//数据段Data长度
	uint8_t		seq;			//包序号
	uint8_t		crc8;			//帧头CRC8校验
}FrameHeader_t;					//FrameHeader

typedef __packed struct
{
	uint8_t		game_type 		: 4;	//比赛类型
	uint8_t 	game_progress 	: 4;	//当前比赛阶段
	uint16_t 	stage_remain_time;		//当前阶段剩余时间
}ext_game_state_t;					//比赛状态

typedef __packed struct
{
	uint8_t winner;			//比赛结果
}ext_game_result_t;			//比赛结果数据

typedef __packed struct
{
	uint16_t robot_legion;
}ext_game_robot_survivors_t;	//机器人存活数据

typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;				//场地事件数据

typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
}ext_supply_projectile_action_t;	//补给站动作

typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_num;
}ext_supply_projectile_booking_t;	//补给站预约子弹

typedef __packed struct
{
	uint8_t 	robot_id;
	uint8_t		robotLevel;
	uint16_t	remain_HP;
	uint16_t 	max_HP;
	uint16_t 	shooter_heat0_cooling_rate;
	uint16_t 	shooter_heat0_cooling_limit;
	uint16_t 	shooter_heat1_cooling_rate;
	uint16_t 	shooter_heat1_cooling_limit;
	uint8_t 	mains_power_gimbal_output :		1;
	uint8_t 	mains_power_chassis_output : 	1;
	uint8_t 	mains_power_shooter_output :	1;
}ext_game_robot_state_t;			//比赛机器人状态

typedef __packed struct
{
	uint16_t	chassisVolt;			//底盘输出电压
	uint16_t	chassis_current;		//底盘输出电流
	float		chassisPower;			//底盘输出功率
	uint16_t	chassisPowerBuffer;	//底盘功率缓冲
	uint16_t	shooterHeat0;			//17mm枪口热量
	uint16_t	shooter_heat1;			//42mm枪口热量
}ext_power_heat_data_t;					//实时功率热量数据

typedef __packed struct
{
	float x;				//位置X坐标值
	float y;				//位置Y坐标值
	float z;				//位置Z坐标值
	float yaw;				//枪口朝向角度值
}ext_game_robot_pos_t;		//机器人位置

typedef __packed struct
{
	uint8_t power_rune_buff;	//Buff类型，1表示有效
}ext_buff_musk_t;					//Buff获取数据

typedef __packed struct
{
	uint8_t energy_point;		//积累的能量点
	uint8_t attack_time;		//可攻击时间
}aerial_robot_energy_t;			//空中机器人能量状态

typedef __packed struct
{
    uint8_t	armor_id  	: 4;	//若变化类型为装甲伤害时，标识装甲ID
    uint8_t	hurt_type   : 4;	//血量变化类型
}ext_robot_hurt_t;				//伤害状态

typedef __packed struct
{
    uint8_t	bullet_type;      	//子弹类型
    uint8_t	bullet_freq;      	//子弹射频
    float	bullet_speed;		//子弹射速
}ext_shoot_data_t;              //实时射击信息

typedef __packed struct
{
	uint16_t data_cmd_id;		//数据段内容ID
	uint16_t send_ID;			//发送者ID
	uint16_t receiver_ID;		//接收者ID
}ext_student_interactive_header_data_t;		//交互数据段头

typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
}client_custom_data_t;			//客户端自定义数据



typedef __packed struct
{
	ext_game_state_t					GameState_t;
	
	ext_game_result_t					GameResult_t;
	
	ext_game_robot_survivors_t			GameRobotSurvivors_t;
	
	ext_event_data_t					EventData_t;
	
	ext_supply_projectile_action_t		SupplyProjectileAction_t;
	
	ext_supply_projectile_booking_t		SupplyProjectileBooking_t;
	
	ext_game_robot_state_t				GameRobotState_t;
	
	ext_power_heat_data_t				PowerHeatData_t;
	
	ext_game_robot_pos_t				GameRobotPos_t;
	
	ext_buff_musk_t						BuffMusk_t;
	
	aerial_robot_energy_t				AerialRobotEnergy_t;
	
	ext_robot_hurt_t					RobotHurt_t;
	
	ext_shoot_data_t					ShootData_t;
	
}ext_referee_data_t;
//typedef __packed struct
//{
//    uint16_t  stageRemianTime; //当前阶段剩余时间
//    uint8_t   gameProgress;    //当前比赛阶段
//    uint8_t   robotLevel;      //机器人当前等级
//    uint16_t  remainHP;        //机器人当前血量
//    uint16_t  maxHP;           //机器人满血量
//}extGameRobotState_t;          //比赛机器人状态

//typedef __packed struct
//{
//    uint8_t   armorType  : 4;  //若变化类型为装甲伤害时，标识装甲ID
//    uint8_t   hurtType   : 4;  //血量变化类型
//}extRobotHurt_t;               //伤害数据

//typedef __packed struct
//{
//    uint8_t 	bulletType;      //弹丸类型
//    uint8_t 	bulletFreq;      //弹丸射频
//    float  	    bulletSpeed;     //弹丸射速
//}extShootData_t;               //实时射击信息

//typedef __packed struct
//{
//    float     chassisVolt;        //底盘输出电压
//    float     chassisCurrent;     //底盘输出电流
//    float     chassisPower;       //底盘输出功率 
//    float     chassisPowerBuffer; //底盘功率缓冲
//    uint16_t  shooterHeat0;       //17mm枪口热量
//    uint16_t  shooterHeat1;       //42mm枪口热量
//	float     Buffer_to_Current;
//}extPowerHeatData_t;//实时功率热量数据

//typedef __packed struct
//{
//    uint8_t cardType;           //卡类型
//    uint8_t cardIdx;						//卡索引号，用于区分不同区域
//}extRfidDetect_t;//场地交互数据

//typedef __packed struct         
//{
//  uint8_t winner;               //比赛结果
//}extGameResult_t;//比赛胜负数据

//typedef __packed struct
//{
//uint16_t buffMusk;              //Buff类型，1表示有效
//}extBuffMusk_t;//Buff获取数据

//typedef __packed struct
//{
//    float x;                    //位置X坐标值
//    float y;										//位置Y坐标值
//		float z;										//位置Z坐标值
//    float yaw;                  //枪口朝向角度值
//}extGameRobotPos_t;//机器人位置朝向信息

//typedef __packed struct
//{
//    float   data1;								//自定义数据1
//    float   data2;								//自定义数据2
//    float   data3;								//自定义数据3
//    uint8_t mask;								//自定义数据4	
//}extShowData_t;//参赛队自定义数据

//typedef union
//{
//	float float_data;
//	uint8_t u8_data[4];
//}Data;//自定义数据存储和转换

typedef union
{
        uint8_t     u8_temp[4];
        uint32_t    u32_temp;
	      float       float_temp;
}FormatTrans32;//32位数据的转换

typedef union
{
        uint8_t     u8_temp[2];
        uint16_t    u16_temp;
}FormatTrans16;//16位数据的转换

typedef __packed struct
{ 
	FrameHeader_t                          frame_header;
	uint16_t                               cmd_id;
	ext_student_interactive_header_data_t  client_header;
	client_custom_data_t                      client_data;
	uint16_t                               CRC16;
}send_data_t;		           //发送的数据
extern send_data_t send_data;

#define Frameheaderlength 5
#define CmdIDlength       2





extern ext_shoot_data_t      ShootData;             //发射状态,包括射速射频
extern ext_power_heat_data_t  PowerHeatData;         //功率和热量用这个变量就可以
extern unsigned char lightcontrol;
extern client_custom_data_t Data1,Data2,Data3;



extern uint32_t system_time;
extern float Shoot_Speed_Store[500];
extern int Number;
extern ext_game_robot_state_t RobotState;
extern ext_buff_musk_t BuffMusk;
void RefereeDecode(uint8_t *pData);
void SendDatabuild(void);
void FloatToUint8(uint8_t * char_array,float data);
void Client_Data_Build(void);
extern uint8_t Client_Data[28];
void data_head_init(void);
void SendDatabuild(void);
//extern extPowerHeatData_t  PowerHeatData;         //功率和热量用这个变量就可以
//extern unsigned char lightcontrol;
//extern Data Data1,Data2,Data3;
//extern float Shoot_Speed_Store[500];
//extern int Number;
//extern extPowerHeatData_t  DataGameinfo;
//extern extGameRobotState_t RobotState;
//void RefereeDecode(uint8_t *pData);
//void SendDatabuild(void);

#endif  
