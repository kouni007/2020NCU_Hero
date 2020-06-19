
 /*******************************************************************************
  �� �� ��   : data_pro_task.c
  ��    ��   : 
  ��������   : 2020��1��
  ����޸�   :
  ��������   : ��̨�������ݴ���
  �����б�   :
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
/************************�ⲿ����*********************************/

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\




/*************************�ڲ�����/����***************************/
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
	*	@supplement	�ò���������
	*	@retval	
****************************************************************************************/
void ChassisModeProcess()
{

	 

	switch(RC_Ctl.rc.s2)  //�Ұ�ť
    {
      case 1://��,������
      {
        chassis_gimble_Mode_flg = 3;
        
      }break;
      case 3://�У����̸���
      {
        chassis_gimble_Mode_flg = 1;
        
       
      }break;
      case 2://��,���̷���
      {
        chassis_gimble_Mode_flg = 2;  
    
      }break;
      default:break;
    
    }
   	CAN_Send_YK(&hcan1,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc. s2,chassis_gimble_Mode_flg);
	
		if(chassis_gimble_Mode_flg==1 || chassis_gimble_Mode_flg==3) //XY�˶�,������ģʽ
   {
      pit_set.expect = pit_set.expect - (0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set_jy61.expect = yaw_set_jy61.expect + (0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set.expect = yaw_get.total_angle;//���·������������
   }
   else//WY�˶���������ģʽ
   {
      pit_set.expect = pit_set.expect - (0x400-RC_Ctl.rc.ch3)/20;	
      yaw_set.expect = yaw_set.expect - (0x400-RC_Ctl.rc.ch2)/20;	
     
     yaw_set_jy61.expect = -ptr_jy61_t_yaw.final_angle;//���¸�������������(����total_angle������Ӹ�����Ϊ�����Ǻ͵���෴)
   }
	  MoCa_Flag = 0; 
  }

				 
/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
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
	
	if(R_Press)   //��������
	{
		minipc_flag=1;
	}
	if(R_Press&&CTRL_Press)    //�ر�����
	{
		minipc_flag=0;
	}
	
	if(minipc_flag==1)   //����״̬
	{
		if(my_abs(minipc_rx.angle_yaw) > 640 || my_abs(minipc_rx.angle_pit) > 360)  //pc�����쳣
				{
					 minipc_flag=0;
					goto Notzimiao;
				 }
		else
			{
				if(minipc_rx.state_flag == 1)    //�Ӿ���⵽��׼��Ŀ�긽���󣬲��ٵ���λ��
					{
									yaw_set.expect_pc = minipc_rx.angle_yaw;    //yaw�ұ�Ϊ��
									pit_set.expect_pc = minipc_rx.angle_pit;    //               
									
								}
				else if(minipc_rx.state_flag == 2)
					{
									yaw_set.expect_pc = minipc_rx.angle_yaw;    //yaw�ұ�Ϊ��
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
		if(chassis_gimble_Mode_flg==1 || chassis_gimble_Mode_flg==3) //XY�˶�,������ģʽ
   {
      pit_set.expect = pit_set.expect - RC_Ctl.mouse.y/2;	
      yaw_set_jy61.expect = yaw_set_jy61.expect -  RC_Ctl.mouse.x*0.4;	
     
     yaw_set.expect = yaw_get.total_angle;//���·������������
   }
   else//WY�˶���������ģʽ
   {
      pit_set.expect = pit_set.expect - RC_Ctl.mouse.y/2;	
      yaw_set.expect = yaw_set.expect  -  RC_Ctl.mouse.x*0.4;	
     
     yaw_set_jy61.expect = -ptr_jy61_t_yaw.final_angle;//���¸�������������(����total_angle������Ӹ�����Ϊ�����Ǻ͵���෴)
   }
		
	}
						

	 /*CTRL+����Ҽ��ر�Ħ����*/
   if(CTRL_Press&&Right_Press)
   {
     MoCa_Flag = 0;
   }
   /*����Ҽ�����Ħ����*/
   else if(Right_Press)
   {
     MoCa_Flag = 1;
   }
	 if(MoCa_Flag == 1)
  {
    /*��������*/
		
    if(Left_Press)        //����������
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
	*	@supplement	ң�����ݽ��ռ�����������������
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void)
{
	uint32_t NotifyValue;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();    //����ϵͳ
	
	
	while(1)
	{
//		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
//		
//    if(NotifyValue == 1)
//		{

		Remote_Ctrl();  //ң�������ݴ���
		RefreshTaskOutLineTime(RemoteDataTask_ON);
		switch(rc.s1)
		{
			case 1:{ ChassisModeProcess();       ; break;}   //������ģʽ

			case 2:{ MouseKeyControlProcess ()    ; break;}      //����ģʽ
			
			case 3:{       ;  break;}     //��ģʽ����Ҫ
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
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	uint32_t NotifyValue;
	osDelay(4000);//��ʱ4000ms���ȴ���̨�������������ת��ſ�ʼ�Ӿ����ݽ���
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1){
//		NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
//    if(NotifyValue==1)
//		{
			Get_MiniPC_Data();  //��ȡ�Ӿ���������Ϣ
			
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
//	}
	
}
	
}




/********ң�������ݲ��Բ���************/
/***************************************************************************************
**
	*	@brief	testOfUsart(void const * argument)
	*	@param
	*	@supplement	����ң�����������ݣ����ò���ϵͳ�շ����ܣ�--���Դ���1����ȡMPU6500����������---spiͨ�ţ�
                ��ӡ����3��������ʾ������������6����ӡң�������ݣ����������ݣ�;���ò���ϵͳ���ƣ�
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
//	NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //δ������֪ͨ��������״̬ȥ�ȴ�����֪ͨ
//    if(NotifyValue==1)
//		{
		    Remote_Ctrl();  //ң�������ݴ���
      flag=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_9);
//   }
		

//		
//		  HAL_UART_Transmit(&huart2,&name1 , 1, 10);
//		 HAL_UART_Transmit(&huart6,&name2 , 1, 10);
//		
		osDelayUntil(&xLastWakeTime,2);
		
	}
}




