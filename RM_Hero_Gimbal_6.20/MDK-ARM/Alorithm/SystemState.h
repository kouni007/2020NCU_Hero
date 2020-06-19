#include "main.h"


#define OutLine_Time 100 //断线检测时间
//#define ShangDan_TIME  10000  //上弹时间


#define MyFlagSet(x,y) x=x|(0x000000000001<<y) //设置标志位  y第几位
#define MyFlagClear(x,y) x=x&~(0x000000000001<<y)
#define MyFlagGet(x,y) (x&(0x000000000001<<y))

typedef struct{
	short Mode;//运行模式
	short Enable;//状态
	short State;//状态
	short Task;//任务
//	BEEPMode Beep;//蜂鸣器
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//时间计数器句柄
	uint16_t OutLine_Flag;//断线标志
	uint16_t task_OutLine_Flag;//断线标志	
//	RobotDistDef RobotDist;//机器人测量
}SystemStateDef;

typedef enum
{
		Remote_NO,
	  JY61_NO,
	 Caipan_NO,
		McCa1_NO,
	  McCa2_NO,
		Pit_NO,
	  Yaw_NO,
	  Bodan_NO,
    Bopan_NO,
	
		DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{
	//testTask_ON,
	RemoteDataTask_ON,
	GimbalContrlTask_ON,
	GunTask_ON,
	//LedTask_ON,
	//vOutLineCheckTask_ON,
	
	TASKTotal_No	
}TASK_NoDEF;


extern SystemStateDef SystemState;
void RefreshSysTime(void);
float GetSystemTimer();
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);
void RefreshTaskOutLineTime(TASK_NoDEF Task_No);
void OutLine_Check();
void TASK_Check();