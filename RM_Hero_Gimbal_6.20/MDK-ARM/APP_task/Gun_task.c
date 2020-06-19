#include "gun_task.h"

#include "pid.h"
#include "cmsis_os.h"
#include "CAN_USE_Moto.h"
#include "can.h"
#include "SystemState.h"
#include "gimbal_task.h"
#include "data_pro_task.h"
#include "minipc.h"
/**************************外部变量*******************************/
uint16_t remain_heat = 0;      //枪口热量剩余值
uint8_t MoCa_Flag = 0;    //摩擦轮电机开启标志位
uint8_t GunReady = 0;    //？？？？？
Heat_Gun_t  ptr_heat_gun_t;
uint8_t Bullets_count=0;



/***************************内部变量/常量******************************/
#define GUN_PERIOD  10

#define fixed_set_yaw1 1235
#define fixed_set_yaw2 7895
#define fixed_set_yaw3 8855
#define fixed_set_yaw4 9965

uint16_t set_yaw1;
uint16_t set_yaw2;
uint16_t set_yaw3;
uint16_t set_yaw4;

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
pid_t pid_shot_spd[2]  = {0};	//摩擦轮速度环
pid_t pid_stir_spd;  //转盘电机速度环
pid_t pid_stir_pos;   //转盘电机位置环
/**                                                           
	**************************************************************
	** Descriptions: 摩擦轮和拨弹pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/
void Gun_Pid_Init()
{
	/*供弹（转盘）电机*/
	 PID_struct_init(&pid_stir_pos, POSITION_PID, 6000, 5000,
	                0.0f, 0.0f, 0.0f);
	 PID_struct_init(&pid_stir_spd, POSITION_PID, 6000, 5000,
	                0.0f, 0.0f, 0.0f);
	
	
  /*拨弹电机*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 20000, 5000,
									0.18f,	0.0f,	0.15f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.15f	);    //此数据在学校时测过，完美落点
   /*摩擦轮*/
  for(uint8_t i = 0;i<2;i++)
  {
    PID_struct_init(&pid_shot_spd[i], POSITION_PID, 6000, 5000,
									0.0f,	0.0f,	0.0f	); 
  }
}

/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪管发射任务
	*	@retval	
****************************************************************************************/


void Gun_Task(void const * argument)
{ 
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	uint8_t motor_stop_flag=0;   //电机关闭标志位
  int32_t set_angle = 0;   //以前的拨齿角度，目前换摩擦轮拨弹
	int32_t set_speed = 0;  
  int32_t set_M_speed = 0;   //发弹摩擦轮速度
  uint8_t set_cnt = 0;    //转动圈数
	uint8_t contiue_flag = 0;   //用于拨弹模式切换时的转变
	int32_t set_stir_speed = 0;
	int32_t set_stir_angle=0;
	float k=1;   //底盘转速比例值（需要给其赋值！！！）
	Gun_Pid_Init();  //pid初始化
	
	while(1)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
		
		if(stir_motor_flag == 1)   //转盘电机，如果光电检测为空则送弹
		{
			switch(Bullets_count){
				case 0:{  //补给3发
					set_stir_angle=set_stir_angle-1365*3*19;  //当前算6个弹孔（8191/6≈1365），转3/6圈，减速比19:1
				  break;
				}
				case 1:{   //补给2发
					set_stir_angle=set_stir_angle-1365*2*19;  //转2/6圈
					break;
				}
				case 2:{   //补给1发
					set_stir_angle=set_stir_angle-1365*1*19;   //转1/6圈
					break;
				}
				case 3:{   //这种情况为：弹丸在下落的途中，底部关电开关还未检测到，但弹丸已经补给
					set_stir_angle=set_stir_angle;   //不动
					break;
				}
			}
			if(Check_stir_locked() || stir_locktime)   //堵转判断
         {
           stir_locktime--;
           set_stir_speed = pid_calc(&pid_stir_pos,moto_stir_get.total_angle,set_stir_angle);
         }
       else set_stir_speed = -800;  //堵住电机反转
         
			pid_calc(&pid_stir_spd, moto_stir_get.speed_rpm, set_stir_speed); 
			Chassis_Motor(&hcan2,0,0,0,pid_stir_spd.pos_out);      //转盘电机数据的发送
		}
		else 
		{
			set_stir_speed=0;
			Chassis_Motor(&hcan2,0,0,0,set_stir_speed);      //转盘电机数据的发送
		}

		switch(MoCa_Flag)
		{
			case 0:   //关闭摩擦轮
			{
				set_M_speed=0;
				ptr_heat_gun_t.sht_flg=GunHold;
				break;
			}
			case 1:
			{
				set_M_speed=6000;   //摩擦轮速度为6000，弹丸理论为18m/s线速度
			  break;
			}
			default :break;
			
			if(minipc_flag==1){
				if(chassis_gimble_Mode_flg==3)  //大陀螺模式状态
			{
				set_yaw1 = fixed_set_yaw1-dstVmmps_W * k;
				set_yaw2 = fixed_set_yaw2+dstVmmps_W * k;
				set_yaw3 = fixed_set_yaw3-dstVmmps_W * k;
				set_yaw4 = fixed_set_yaw4+dstVmmps_W * k;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)||
					minipc_rx.state_flag==0){   
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			else{
				set_yaw1 = fixed_set_yaw1;
				set_yaw2 = fixed_set_yaw2;
				set_yaw3 = fixed_set_yaw3;
				set_yaw4 = fixed_set_yaw4;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)||
					minipc_rx.state_flag==0){     //自瞄模式下yaw轴角度范围，视觉无目标（即摄像头被挡)就停止发射
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			}
			else{
				//防止打到支撑杆
			if(chassis_gimble_Mode_flg==3)  //大陀螺模式状态
			{
				set_yaw1 = fixed_set_yaw1-dstVmmps_W * k;
				set_yaw2 = fixed_set_yaw2+dstVmmps_W * k;
				set_yaw3 = fixed_set_yaw3-dstVmmps_W * k;
				set_yaw4 = fixed_set_yaw4+dstVmmps_W * k;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)){
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			else{
				set_yaw1 = fixed_set_yaw1;
				set_yaw2 = fixed_set_yaw2;
				set_yaw3 = fixed_set_yaw3;
				set_yaw4 = fixed_set_yaw4;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle)&&(yaw_get.angle<set_yaw4)){
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			}
			
			
			
			if(remain_heat<100)//热量限制部分（可以考虑优化将热量利用率达到最大)
			{
				ptr_heat_gun_t.sht_flg=GunStop;
			}
			
      
			
			
      switch(ptr_heat_gun_t.sht_flg)//判断发射模式
			{
				case GunStop:
				{
					
					switch(contiue_flag)
					{
						case 0:
						{   /*设定角度(保存上一次的角度值)*/
							set_angle=moto_dial_get.total_angle;
							contiue_flag=1;
							break;
						}
						case 1:
						{
							goto position;
							break;
						}
					}
					break;
				}
				
				case GunOne:    //使用摩擦轮驱动  (单发模式）
				{
					set_angle=set_angle-5400*36 ;   //减速比为36:1(转2/3圈)相当于发射一颗弹丸
					Bullets_count--;
					ptr_heat_gun_t.sht_flg = GunHold;  //发射完后（鼠标点击一次后）变回等待模式
          contiue_flag = 0;
					goto position;    //这一次就进入位置环，是否有问题，待测试！！！
					break;
				}
				
				case GunFire:   //(连发模式)
				{
					set_angle=set_angle-5400*36*3;   //3连发
					Bullets_count-=3;
					ptr_heat_gun_t.sht_flg = GunHold;  //发射完后（鼠标点击一次后）变回等待模式
					contiue_flag = 0;
					goto position; 
					break;
				}
				
				case GunHold:
				{
					position:
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	

				  set_speed=pid_dial_pos.pos_out;
					break;
				}
				default :break;
			}
			
			//拨弹电机和发射摩擦轮速度环
			pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
      pid_calc(&pid_shot_spd[0],moto_M_get[0].speed_rpm ,-set_M_speed);
      pid_calc(&pid_shot_spd[1],moto_M_get[1].speed_rpm ,set_M_speed);
			
			//驱动拨弹电机，摩擦轮
			Shot_Motor(&hcan2,pid_dial_spd.pos_out,pid_shot_spd[0].pos_out,pid_shot_spd[1].pos_out);		
				
			//set_speed=0; //拨弹电机重置		
				

			

			}
			osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}

}


















