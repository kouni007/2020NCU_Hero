#include "test.h"


#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "shiboqi.h"
#include "usart.h"
#include "Remote.h"
#include "pid.h"
#include "can.h"
#include "data_pro_task.h"
#include "CAN_USE_Moto.h"
#include "protocol.h"

/***************************************************************************************
**
	*	@brief	testOfMochaTask()
	*	@param
	*	@supplement	弹道测试（2006,3508摩擦轮）
	*	@retval	
****************************************************************************************/

pid_t pid_testofMocha_dial_pos;
pid_t pid_testofMocha_dial_spd;
pid_t pid_test_shot_spd[2]  = {0};	//摩擦轮速度环

int32_t set_test_angle = 0;   //拨弹位置目标值
int32_t set_test_M_speed = 0;    //摩擦轮速度值 
int32_t set_test_speed = 0;   //拨弹速度目标值
	uint8_t a_test=0;    //拨齿开关标志位

void testOfMocha_pid_init(void)
{
	PID_struct_init(&pid_testofMocha_dial_pos,POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.7f); 
  PID_struct_init(&pid_testofMocha_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.7f	); 


/* 拨弹电机
	位置环0.4  0.0 0.8     速度环0.3  0.0  0.8    （20000，5000）  速度响应要求较快几乎达到预计值，有震感，可能会有点影响云台稳定
	 d最大0.8，再大意义不大，d越小，震感越小，离预计值越偏离越远    若加i值，会造成来回抖动


*/	

  for(uint8_t i = 0;i<2;i++)
  {
    PID_struct_init(&pid_test_shot_spd[i], POSITION_PID, 6000, 5000,
									2.0f,	0.01f,	0.5f	); 
  }
}

void testOfMochaTask(void const * argument)
{
	
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

	testOfMocha_pid_init();
	
	for(;;)
	{ 		
		Remote_Ctrl();  //遥控器数据处理
		HAL_Delay(5);    //遥控器数据接收

		if(rc.s1==3){
			set_test_M_speed=5500;
      pid_calc(&pid_test_shot_spd[0],moto_M_get[0].speed_rpm ,-set_test_M_speed);
      pid_calc(&pid_test_shot_spd[1],moto_M_get[1].speed_rpm ,set_test_M_speed);	
						if(rc.s2 ==3){
					a_test=1;
				}
				if(a_test==1&&rc.s2 ==2){
				  set_test_angle=set_test_angle-5400*36 ;   //减速比为36:1
			    a_test=0;
					pid_calc(&pid_testofMocha_dial_pos, moto_test_dial_get.total_angle,set_test_angle);	
				set_test_speed=pid_testofMocha_dial_pos.pos_out;
		    pid_calc(&pid_testofMocha_dial_spd,moto_test_dial_get.speed_rpm ,set_test_speed);
				Chassis_Motor(&hcan2,pid_testofMocha_dial_spd.pos_out,0,0,0);
					
				}    //拨齿开关判断，开一次转一次
			  pid_calc(&pid_testofMocha_dial_pos, moto_test_dial_get.total_angle,set_test_angle);	
				set_test_speed=pid_testofMocha_dial_pos.pos_out;
		    pid_calc(&pid_testofMocha_dial_spd,moto_test_dial_get.speed_rpm ,set_test_speed);
				Chassis_Motor(&hcan2,pid_testofMocha_dial_spd.pos_out,pid_test_shot_spd[0].pos_out,pid_test_shot_spd[1].pos_out,0);
		}
		else{
			Chassis_Motor(&hcan2,0,0,0,0);
		}
		
	
		int16_t angle1[4];
    int16_t  *ptr = angle1; //初始化指针
		angle1[0]	= (int16_t)(2000);
		angle1[1]	= (int16_t)(-moto_M_get[0].speed_rpm);
		angle1[2]	= ((int16_t)moto_M_get[1].speed_rpm);
		angle1[3]	= (int16_t)(Robot.Chassis_Power.Chassis_Current);
		//用虚拟示波器，发送数据
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[0]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[1]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[2]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[3]));	
		
		osDelayUntil(&xLastWakeTime,3);
	}
}
	