#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "mecanum_calc.h"
#include "CAN_USE_Moto.h"
#include "can.h"

/**************************外部变量*******************************/

moto_3508_type moto_3508_set;    //分解为x，y，w的值
uint8_t round_flag=0;    //分离模式变量
uint8_t back_flag=0;    //跟随模式
int8_t chassis_disable_flg;   //紧急停止变量
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环


/***************************内部变量/常量******************************/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	


#define CHASSIS_PERIOD 5
#define Middle_angle 3700   //云台中间的数值(*注意*该值去去年值，今年需重新测！）
/***************************************************************************************
**
	*	@brief	Chassis_pid_init(void const * argument)
	*	@param
	*	@supplement	底盘pid初始化
	*	@retval	
****************************************************************************************/
void Chassis_pid_init(void)
{
	PID_struct_init(&pid_chassis_follow,POSITION_PID,0,0,0,0,0);
	//底盘跟随位置环
	PID_struct_init(&pid_chassis_follow_spd,POSITION_PID,0,0,0,0,0);
	//底盘跟随速度环
	
   for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 15000, 5000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.
		}     //速度环初始化


}


/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/

void Chassis_Contrl_Task(void)
{
	/*数据初始化*/
	static float  wheel[4]={0,0,0,0};
	static float Angle_gap;
	osDelay(200);//延时200ms
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();     //操作系统
	
	chassis_disable_flg=0;
	Chassis_pid_init();  //底盘pid初始化
	
	while(1)
	{
		Angle_gap=yaw_get.angle-Middle_angle; //计算此时云台和中间值所差的角度
	 if(chassis_gimble_Mode_flg==0||round_flag)
	 {      //分离模式
		 if(back_flag) 
		 {
			 goto back;	 
			}
		  motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
	   //分离模式下的麦轮解算
	  }
			
		else
		{      //跟随模式
			back:
			/*跟随位置环*/
			pid_calc(&pid_chassis_follow,yaw_get.angle,Middle_angle);
      /*跟随速度环*/ 
			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
      moto_3508_set.dstVmmps_W = pid_chassis_follow_spd.pos_out;
      if(ABS(Angle_gap)<50) 
			{
				moto_3508_set.dstVmmps_W=0;
			}
        /*麦轮解算得出wheel[4]*/
			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W); 	

		}
		
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
		}
			
		/*功率限制*/
			
			
			
			
			
			
			
		/**********/
			
			//驱动电机
			if(chassis_disable_flg==1)//失能
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








