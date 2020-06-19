#include "main.h"


#define OutLine_Time 100 //���߼��ʱ��
//#define ShangDan_TIME  10000  //�ϵ�ʱ��


#define MyFlagSet(x,y) x=x|(0x000000000001<<y) //���ñ�־λ  y�ڼ�λ
#define MyFlagClear(x,y) x=x&~(0x000000000001<<y)
#define MyFlagGet(x,y) (x&(0x000000000001<<y))

typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
//	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	uint16_t OutLine_Flag;//���߱�־
	uint16_t task_OutLine_Flag;//���߱�־	
//	RobotDistDef RobotDist;//�����˲���
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