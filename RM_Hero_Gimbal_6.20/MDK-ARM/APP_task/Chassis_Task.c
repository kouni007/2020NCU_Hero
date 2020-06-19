#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "mecanum_calc.h"
#include "CAN_USE_Moto.h"
#include "can.h"

/**************************�ⲿ����*******************************/

moto_3508_type moto_3508_set;    //�ֽ�Ϊx��y��w��ֵ
uint8_t round_flag=0;    //����ģʽ����
uint8_t back_flag=0;    //����ģʽ
int8_t chassis_disable_flg;   //����ֹͣ����
pid_t pid_chassis_follow = {0};//���̸���λ�û�
pid_t pid_chassis_follow_spd = {0};//���̸����ٶȻ�


/***************************�ڲ�����/����******************************/
pid_t pid_3508_pos;     		 //���̵��λ�û�
pid_t pid_3508_spd[4];			 //���̵���ٶȻ�
pid_t pid_3508_current[4];	 //���̵����������	


#define CHASSIS_PERIOD 5
#define Middle_angle 3700   //��̨�м����ֵ(*ע��*��ֵȥȥ��ֵ�����������²⣡��
/***************************************************************************************
**
	*	@brief	Chassis_pid_init(void const * argument)
	*	@param
	*	@supplement	����pid��ʼ��
	*	@retval	
****************************************************************************************/
void Chassis_pid_init(void)
{
	PID_struct_init(&pid_chassis_follow,POSITION_PID,0,0,0,0,0);
	//���̸���λ�û�
	PID_struct_init(&pid_chassis_follow_spd,POSITION_PID,0,0,0,0,0);
	//���̸����ٶȻ�
	
   for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 15000, 5000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.
		}     //�ٶȻ���ʼ��


}


/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void)
	*	@param
	*	@supplement	���̿�������
	*	@retval	
****************************************************************************************/

void Chassis_Contrl_Task(void)
{
	/*���ݳ�ʼ��*/
	static float  wheel[4]={0,0,0,0};
	static float Angle_gap;
	osDelay(200);//��ʱ200ms
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();     //����ϵͳ
	
	chassis_disable_flg=0;
	Chassis_pid_init();  //����pid��ʼ��
	
	while(1)
	{
		Angle_gap=yaw_get.angle-Middle_angle; //�����ʱ��̨���м�ֵ����ĽǶ�
	 if(chassis_gimble_Mode_flg==0||round_flag)
	 {      //����ģʽ
		 if(back_flag) 
		 {
			 goto back;	 
			}
		  motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
	   //����ģʽ�µ����ֽ���
	  }
			
		else
		{      //����ģʽ
			back:
			/*����λ�û�*/
			pid_calc(&pid_chassis_follow,yaw_get.angle,Middle_angle);
      /*�����ٶȻ�*/ 
			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
      moto_3508_set.dstVmmps_W = pid_chassis_follow_spd.pos_out;
      if(ABS(Angle_gap)<50) 
			{
				moto_3508_set.dstVmmps_W=0;
			}
        /*���ֽ���ó�wheel[4]*/
			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W); 	

		}
		
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
		}
			
		/*��������*/
			
			
			
			
			
			
			
		/**********/
			
			//�������
			if(chassis_disable_flg==1)//ʧ��
			{
				  Disable_Chassis_Motor(&hcan2);
			}
			else
			{
				Chassis_Motor(&hcan2,
												pid_3508_spd[0].pos_out,
												pid_3508_spd[1].pos_out, 
												pid_3508_spd[2].pos_out, 
												pid_3508_spd[3].pos_out);
      } 
			osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
	}
}








