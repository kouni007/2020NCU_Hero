
 /*******************************************************************************
  文 件 名   : data_pro_task.c
  作    者   : 
  生成日期   : 2020年1月
  最近修改   :
  功能描述   : 云台控制数据处理
  函数列表   :
*******************************************************************************/

#include "data_pro_task.h"

#include "cmsis_os.h"
#include "Remote.h"
#include "CAN_USE_Moto.h"
#include "gimbal_task.h"
#include "jy61.h"
#include "can.h"
#include "gun_task.h"
#include "SystemState.h"
#include "minipc.h"
#include "gpio.h"

#include "MPU6500.h"
#include "shiboqi.h"
#include "usart.h"
/************************外部变量*********************************/

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\




/*************************内部变量/常量***************************/
#define REMOTE_PERIOD 5
#define MINIPC_PERIOD 10
#define press_times  20


int16_t XY_speed_max = 12000;
int16_t XY_speed_min = -12000; 
int16_t W_speed_max = 2000;
int16_t W_speed_min = -2000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;

uint8_t minipc_flag=0;

uint8_t flag;
/***************************************************************************************
**
	*	@brief	ChassisModeProcess()
	*	@param
	*	@supplement	用操作杆运行
	*	@retval	
****************************************************************************************/
void ChassisModeProcess()
{

	 

	switch(RC_Ctl.rc.s2)  //右按钮
    {
      case 1://上,大陀螺
      {
        chassis_gimble_Mode_flg = 3;
        
      }break;
      case 3://中，底盘跟随
      {
        chassis_gimble_Mode_flg = 1;
        
       
      }break;
      case 2://下,底盘分离
      {
        chassis_gimble_Mode_flg = 2;  
    
      }break;
      default:break;
    
    }
   	CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc. s2,chassis_gimble_Mode_flg);
	
		if(chassis_gimble_Mode_flg==1 || chassis_gimble_Mode_flg==3) //XY运动,陀螺仪模式
   {
      pit_set.expect = pit_set.expect - (0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set_jy61.expect = yaw_set_jy61.expect + (0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set.expect = yaw_get.total_angle;//更新分离编码器期望
   }
   else//WY运动，编码器模式
   {
      pit_set.expect = pit_set.expect - (0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set.expect = yaw_set.expect - (0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set_jy61.expect = -ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望(就是total_angle，必须加负，因为陀螺仪和电机相反)
   }
	  MoCa_Flag = 0; 
  }

				 
/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/			    
void MouseKeyControlProcess()
{
	
	static uint16_t delay = 0;
	chassis_gimble_Mode_flg=1;
		
	CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc. s2,chassis_gimble_Mode_flg);
	if(BULLTE_EMPTY){
		stir_motor_flag = 1;
	}
	else stir_motor_flag = 0;
	
	
	if(E_Press || Q_Press)
	{
		chassis_gimble_Mode_flg=2;
		
	}
	if(F_Press)
	{
		chassis_gimble_Mode_flg=3;
	}
	
	if(R_Press)   //开启自瞄
	{
		minipc_flag=1;
	}
	if(R_Press&&CTRL_Press)    //关闭自瞄
	{
		minipc_flag=0;
	}
	
	if(minipc_flag==1)   //自瞄状态
	{
		if(my_abs(minipc_rx.angle_yaw) > 640 || my_abs(minipc_rx.angle_pit) > 360)  //pc数据异常
				{
					 minipc_flag=0;
					goto Notzimiao;
				 }
		else
			{
				if(minipc_rx.state_flag == 1)    //视觉检测到瞄准至目标附近后，不再调整位置
					{
									yaw_set.expect_pc = minipc_rx.angle_yaw;    //yaw右边为负
									pit_set.expect_pc = minipc_rx.angle_pit;    //               
									
								}
				else if(minipc_rx.state_flag == 2)
					{
									yaw_set.expect_pc = minipc_rx.angle_yaw;    //yaw右边为负
									pit_set.expect_pc = minipc_rx.angle_pit;    // 
								}      
						}
						minipc_rx.angle_yaw = 0;
						minipc_rx.angle_pit = 0; 
						yaw_set.expect = yaw_get.total_angle;		
						yaw_set_jy61.expect = -ptr_jy61_t_yaw.final_angle;
						pit_set.expect=pit_get.total_angle;
			}
	else
	{
		Notzimiao:
		if(chassis_gimble_Mode_flg==1 || chassis_gimble_Mode_flg==3) //XY运动,陀螺仪模式
   {
      pit_set.expect = pit_set.expect - RC_Ctl.mouse.y/2;	
      yaw_set_jy61.expect = yaw_set_jy61.expect -  RC_Ctl.mouse.x*0.4;	
     
     yaw_set.expect = yaw_get.total_angle;//更新分离编码器期望
   }
   else//WY运动，编码器模式
   {
      pit_set.expect = pit_set.expect - RC_Ctl.mouse.y/2;	
      yaw_set.expect = yaw_set.expect  -  RC_Ctl.mouse.x*0.4;	
     
     yaw_set_jy61.expect = -ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望(就是total_angle，必须加负，因为陀螺仪和电机相反)
   }
		
	}
						

	 /*CTRL+鼠标右键关闭摩擦轮*/
   if(CTRL_Press&&Right_Press)
   {
     MoCa_Flag = 0;
   }
   /*鼠标右键开启摩擦轮*/
   else if(Right_Press)
   {
     MoCa_Flag = 1;
   }
	 if(MoCa_Flag == 1)
  {
    /*发弹控制*/
		
    if(Left_Press)        //鼠标左键单发
    {
			if(CTRL_Press)
			{
				if(delay > PRESS_DELAY && Left_Press)
				{
				ptr_heat_gun_t.sht_flg = GunFire;
				delay = 0;
				}
			}
			
      else
      {
				if(delay > PRESS_DELAY && Left_Press)
				{
        ptr_heat_gun_t.sht_flg = GunOne;
        delay = 0;
				}
      }
      delay++; 
    }
	}

				 
				 
} 


				 
				 
				 
				 
				 
				 
				 
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务（主体任务）
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void)
{
	uint32_t NotifyValue;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();    //操作系统
	
	
	while(1)
	{
//		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
//		
//    if(NotifyValue == 1)
//		{

		Remote_Ctrl();  //遥控器数据处理
		RefreshTaskOutLineTime(RemoteDataTask_ON);
		switch(rc.s1)
		{
			case 1:{ ChassisModeProcess();       ; break;}   //操作杆模式

			case 2:{ MouseKeyControlProcess ()    ; break;}      //键盘模式
			
			case 3:{       ;  break;}     //此模式不需要
//	  }
		
		
	
	
		
		
		
		
		/****************/
		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	
	}
}
	
}




/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	视觉数据处理任务
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	uint32_t NotifyValue;
	osDelay(4000);//延时4000ms，等待云台任务进入正常运转后才开始视觉数据接收
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1){
//		NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
//    if(NotifyValue==1)
//		{
			Get_MiniPC_Data();  //获取视觉传来的信息
			
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
//	}
	
}
	
}




/********遥控器数据测试部分************/
/***************************************************************************************
**
	*	@brief	testOfUsart(void const * argument)
	*	@param
	*	@supplement	测试遥控器接收数据（采用操作系统收发功能）--测试串口1；获取MPU6500陀螺仪数据---spi通信；
                打印串口3（用虚拟示波器），串口6（打印遥控器数据，陀螺仪数据）;采用操作系统控制；
	*	@retval	
****************************************************************************************/


void testOfUsart(){      
	uint32_t NotifyValue;
	uint8_t name1=12;
	uint8_t name2=13;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1){
		//Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,SizeofRemote);
		HAL_GPIO_TogglePin(GPIOG,LED1_Pin); 
		HAL_Delay(100);
//	NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
//    if(NotifyValue==1)
//		{
		    Remote_Ctrl();  //遥控器数据处理
      flag=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_9);
//   }
		

//		
//		  HAL_UART_Transmit(&huart2,&name1 , 1, 10);
//		 HAL_UART_Transmit(&huart6,&name2 , 1, 10);
//		
		osDelayUntil(&xLastWakeTime,2);
		
	}
}




