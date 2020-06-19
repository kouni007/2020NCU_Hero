#include "gimbal_task.h"

#include "pid.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "CAN_USE_Moto.h"
#include "JY61.h"
#include "MPU6500.h"
#include "SystemState.h"
#include "data_pro_task.h"

#include "Remote.h"
/**************************外部变量*******************************/
Pos_Set  yaw_set;
Pos_Set  yaw_set_jy61;
Pos_Set  pit_set;

uint8_t round_flag=0;    //分离模式变量
uint8_t back_flag=0;    //跟随模式
	uint8_t a=0;
	int32_t set_angle1=0;
	int32_t set_speed1=0;


/***************************内部变量/常量******************************/
#define GIMBAL_PERIOD 5
pid_t pid_yaw_jy61 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_yaw_jy61_spd = {0};    //陀螺仪模式下的yaw轴pid

pid_t pid_pit_start = {0};
pid_t pid_pit_start_spd = {0};
pid_t pid_yaw_start = {0};
pid_t pid_yaw_start_spd = {0};    //启动时pit和yaw轴所用的pid

pid_t pid_yaw       = {0};  //编码器模式下yaw轴位置环
pid_t pid_pit       = {0};	//共用pit轴位置环
pid_t pid_yaw_spd   = {0};	//编码器模式下yaw轴速度环
pid_t pid_pit_spd   = {0};	//共用pit轴速度环

/*******测试的pid********/
pid_t pid_test_dial_pos;
pid_t pid_test_dial_spd;
/************************/


int8_t gimbal_disable_flg;   //云台紧急停止标志位

static  int16_t Yaw_Current_Value = 0;    //yaw最终输出值
static  int16_t Pitch_Current_Value = 0;    //pit最终输出值

/**                                                           
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/

  /*imu pid parameter*/

	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									0.0f, 0.0f,0.0f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 0, 0,
                  0.0f, 0.0f,0.0f);
  
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  0.0f, 0.0f,0.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  0.0f, 0.0f,0.0f);



  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  0.0f, 0.0f, 0.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  0.0f, 0.0f, 0.0f );
  
   PID_struct_init(&pid_pit_start, POSITION_PID, 400, 230, 0.75f, 0.015f, 0.0f);              //启动pid,因偏差会比较大，给的值较小
   PID_struct_init(&pid_pit_start_spd, POSITION_PID, 18000, 10000, 41.0f, 0.75f, 0.1f ); 
	
	 PID_struct_init(&pid_yaw_start, POSITION_PID, 400, 230, 0.85f, 0.018f, 0.0f);              //启动pid
   PID_struct_init(&pid_yaw_start_spd, POSITION_PID, 18000, 10000, 45.0f, 0.75f, 0.1f ); 
	
	/************测试的pid初始化*************/
PID_struct_init(&pid_test_dial_pos,POSITION_PID, 20000, 5000,
									0.18f,	0.0f,	0.15f); 
PID_struct_init(&pid_test_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.15f	);  									
	
}



/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	

//如果是陀螺仪零漂，可以采集开机静止一段时间的零漂值做去零漂处理，同时增加一个模式遥控器在一段时间无输入之后云台不跟随陀螺仪控制。
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	//初始化
	static uint16_t gimbal_start_count = 0;
	static uint8_t gimbal_start_mode = 1;//云台初始化模式
	yaw_set.expect = 0; 
	pit_set.expect = 0;

	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	minipc_flag=0;
	
	gimbal_pid_init();  //pid初始化
	
	
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
	while(1)
	{
		RefreshTaskOutLineTime(GimbalContrlTask_ON);
		IMU_Get_Data();
		if(gimbal_start_mode == 1 )         //pit轴启动 
			{
         chassis_gimble_Mode_flg = 4;  //pit轴启动模式
				
			}else if(gimbal_start_mode == 2 ) //pit轴启动到yaw轴启动间的过渡
			{
				 chassis_gimble_Mode_flg = 4;
				 gimbal_start_count++;
				 if(gimbal_start_count > 100 )   //过渡时间等待
				 {
						gimbal_start_mode = 3;  //yaw轴启动
				 }
			}else if(gimbal_start_mode == 3 )    //yaw轴启动
			{
         chassis_gimble_Mode_flg = 5;     //yaw轴启动模式    
			}                                              

			
		  if(gimbal_start_mode == 1)      //pit轴启动模式下
			{
		     if( ABS(pit_get.total_angle) <= 100)  //pit轴到达目标位置附近
				 {
					 gimbal_start_mode = 2;   //过渡阶段
				 }
			}else if(gimbal_start_mode == 3)
			{
				if( ABS(yaw_get.total_angle) <= 100)  //yaw轴到达目标位置附近
				 {
					 gimbal_start_mode = 0;   //启动阶段结束
					 chassis_gimble_Mode_flg=1;//陀螺仪模式

				 }
			}
			/*******pit轴限位******************/
			
			
			
			
			/*********************************/
			if(minipc_flag==1)    //自瞄模式
			{
				pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect_pc);
				pid_calc(&pid_yaw_spd,(imu_data.gx), pid_yaw.pos_out);
				Yaw_Current_Value = pid_yaw_spd.pos_out;
				
				pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect_pc);
				pid_calc(&pid_pit_spd,(imu_data.gz), pid_pit.pos_out);   
				Pitch_Current_Value = pid_pit_spd.pos_out; 
			}
			else
			{
				switch(chassis_gimble_Mode_flg)
			{	
				case 1: //陀螺仪模式(跟随)
				{     
							pid_calc(&pid_yaw_jy61,ptr_jy61_t_yaw.final_angle,yaw_set_jy61.expect );  
							pid_calc(&pid_yaw_jy61_spd,(imu_data.gx),pid_yaw_jy61.pos_out);
					    Yaw_Current_Value = pid_yaw_jy61_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz), pid_pit.pos_out);          //(imu_data.gz)/16.4  为°/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
	
				}break;
				
        case 2: //编码器模式 (分离，应该继续用陀螺仪模式，后期看情况更改)
			  { 
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx), pid_yaw.pos_out);
							Yaw_Current_Value = pid_yaw_spd.pos_out;
				
							pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_spd,(imu_data.gz), pid_pit.pos_out);   
							Pitch_Current_Value = pid_pit_spd.pos_out; 
					

			  }break;
				
			  case 3: //小陀螺模式（与陀螺仪模式相同，仅在底盘控制上有区别）  
			  {  
				      pid_calc(&pid_yaw_jy61,ptr_jy61_t_yaw.final_angle,yaw_set_jy61.expect );  
							pid_calc(&pid_yaw_jy61_spd,(imu_data.gx),pid_yaw_jy61.pos_out);
					    Yaw_Current_Value = pid_yaw_jy61_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz), pid_pit.pos_out);          //(imu_data.gz)/16.4  为°/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      
			  }break; 

				
				
				case 4:  //pit轴启动（必须先启动pit轴，在启动yaw轴）
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
				}break;
				
				case 5:  //yaw轴启动
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
					
					    pid_calc(&pid_yaw_start, yaw_get.total_angle, 0);
						  pid_calc(&pid_yaw_start_spd,(imu_data.gx)/16.4, pid_yaw_start.pos_out);
							Yaw_Current_Value = pid_yaw_start_spd.pos_out;
				}break;
				
			  default: break;
				
			 }   
			}
			
			
			if(gimbal_disable_flg==0)
			{
					Disable_Gimbal_Motor(&hcan1);
			}
			else 
			{
				Cloud_Gimbal_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
			}
				
		  /*发送数据给底盘部分*/
//				CAN_Send_YT(&hcan2,yaw_get.angle,yaw_get.total_angle,chassis_gimble_Mode_flg);
				
				
				
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
	}

}




/***************************************************************************************
**
	*	@brief	testOfCan(void const * argument)
	*	@param   测试can1,can2，将1个电机电调分别接can1，can2.
	*	@supplement	
	*	@retval	
****************************************************************************************/


 

void testOfCan(){

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	gimbal_pid_init();
	while(1){
		
  
		
//HAL_GPIO_TogglePin(GPIOG,LED2_Pin); 
	//	HAL_Delay(100);
	//	IMU_Get_Data();
//		if(rc.s1==3){
//			
//	}			//led1闪烁}

		IMU_Get_Data();
		HAL_GPIO_TogglePin(GPIOG,LED2_Pin); 
		HAL_Delay(100);
    if(rc.s1==3)
		{
			if(rc.s2 ==3){
					a=1;
				}
				if(a==1&&rc.s2 ==2){
				  set_angle1=set_angle1-5400*36;   //减速比为36:1
			    a=0;
				pid_calc(&pid_test_dial_pos, moto_test_dial_get.total_angle,set_angle1);	
				set_speed1=-pid_test_dial_pos.pos_out;
		    pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
				Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
					
				}    //拨齿开关判断，开一次转一次
				
			  pid_calc(&pid_test_dial_pos, moto_test_dial_get.total_angle,set_angle1);	
				set_speed1=-pid_test_dial_pos.pos_out;
		    pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
				Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
//			set_speed1=1000;
//			pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
//      Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
				
			}
		else {
			Chassis_Motor(&hcan2,0,0,0,0);
		}
		
			}
		
			
//				
				osDelayUntil(&xLastWakeTime,2);
	}

	
	
	