/********************************************************************************
* 文 件 名   : Motor_USE_CAN.c
* 作    者   : NCURM
* 生成日期   : 2020年1月
* 最近修改   :
* 功能描述   : 电机库模块中使用CAN进行控制的电机
* 函数列表   :
*使用CAN通讯的电机：云台电机   		 底盘电机	 	 	  拨弹电机
*				 	对应型号： c620						3508					 C2006
*接口函数：
*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
*					Chassis_Motor( CAN_HandleTypeDef * hcan,
*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
*******************************************************************************/

#include "CAN_USE_Moto.h"

#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"

#include "usart.h"
#include "gpio.h"
#include "pid.h"
#include "stdlib.h"
#include "stdio.h"
#include "MPU6500.h"
#include "Remote.h"



#define STIR_BLOCK_TIME 1000

uint32_t  Tx_Mailbox;
 CAN_RxHeaderTypeDef  RxMessage;
 static CAN_TxHeaderTypeDef  Cloud_Platform_Data; 


uint8_t Receive_DATA[8];  //can中断接收数据数组
uint32_t stir_locktime ;     //堵转判断变量
moto_measure_t   moto_stir_get;  

 uint8_t arr[8];
 
 /****************************外部变量***********************************/
moto_measure_t   moto_chassis_get[4] = {0};//底盘电机数据返回值
moto_measure_t   pit_get;   //pit返回值
moto_measure_t   yaw_get;     //yaw返回值
moto_measure_t   moto_dial_get;  //c2006
moto_measure_t   moto_M_get[2];
moto_measure_t   moto_stir_get = {0};  //转盘电机3508


uint8_t chassis_gimble_Mode_flg=1;   //云台底盘分离模式标志
uint8_t stir_motor_flag = 0;     //转盘电机开启标志位
int16_t yaw_speed;       //yaw速度
uint8_t dstVmmps_W;

/***********测试电机获取值************/
moto_measure_t moto_test_dial_get;

/***************************************CAN数据的发送*********************************************/

/**
	**************************************************************
	** Descriptions: 云台电机校准函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
	void Cloud_Gimbal_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)   //发送数据给6623电机
{
	  uint8_t CAN_TX_DATA[8];
	
		Cloud_Platform_Data.StdId = 0x1FF;
//		Cloud_Platform_Data.StdId = 0x3F0;  
		Cloud_Platform_Data.IDE = CAN_ID_STD; 
		Cloud_Platform_Data.DLC = 0X08;
		
		CAN_TX_DATA[0] = yaw>>8;
		CAN_TX_DATA[1] = yaw;
		CAN_TX_DATA[2] = pitch>>8;
		CAN_TX_DATA[3] = pitch;
		CAN_TX_DATA[4] = 0x00;
		CAN_TX_DATA[5] = 0x00;
		CAN_TX_DATA[6] = 0x00;
		CAN_TX_DATA[7] = 0x00;

  	HAL_CAN_AddTxMessage(hcan, &Cloud_Platform_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}

/**
	**************************************************************
	** Descriptions: 云台电机失能函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
	void Disable_Gimbal_Motor(CAN_HandleTypeDef * hcan)   //关闭6623电机
{
	  uint8_t CAN_TX_DATA[8];
	
		Cloud_Platform_Data.StdId = 0x1FF;
//		Cloud_Platform_Data.StdId = 0x3F0;  
		Cloud_Platform_Data.IDE = CAN_ID_STD; 
		Cloud_Platform_Data.DLC = 0X08;
		
		CAN_TX_DATA[0] = 0x00;
		CAN_TX_DATA[1] = 0x00;
		CAN_TX_DATA[2] = 0x00;
		CAN_TX_DATA[3] = 0x00;
		CAN_TX_DATA[4] = 0x00;
		CAN_TX_DATA[5] = 0x00;
		CAN_TX_DATA[6] = 0x00;
		CAN_TX_DATA[7] = 0x00;

  	HAL_CAN_AddTxMessage(hcan, &Cloud_Platform_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}


void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2,int16_t iq3, int16_t iq4){  //发送数据给3508电机
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.StdId=0x200;
	
	arr[0]=iq1>>8;
	    arr[1]=iq1;
	arr[2]=iq2>>8;
	arr[3]=iq2;
		arr[4]=iq3>>8;
		arr[5]=iq3;
		arr[6]=iq4>>8;
		arr[7]=iq4;
	HAL_CAN_AddTxMessage(hcan,&TxMessage,arr,&Tx_Mailbox);
}


void Disable_Chassis_Motor( CAN_HandleTypeDef * hcan){   //失能3508电机
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.StdId=0x200;
	
	arr[0]=0;
	   arr[1]=0;
	arr[2]=0;
	arr[3]=0;
	arr[4]=0;
	   arr[5]=0;
	arr[6]=0;
	arr[7]=0;
	HAL_CAN_AddTxMessage(hcan,&TxMessage,arr,&Tx_Mailbox);
}

/**
	**************************************************************
	** Descriptions: 拨弹电机驱动函数
	** Input: 	
	**			   hcan:要使用的CAN1
	**				value:拨弹电机的电流值
	** Output: NULL
	**************************************************************
**/
void Shot_Motor(CAN_HandleTypeDef * hcan,int16_t bo_value,int16_t M1_value,int16_t M2_value)
{
	
  CAN_TxHeaderTypeDef TxMessage;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.StdId=0x201;
	
	arr[0]=bo_value>>8;
	    arr[1]=bo_value;
	arr[2]=M1_value>>8;
	arr[3]=M1_value;
		arr[4]=M2_value>>8;
		arr[5]=M2_value;
		arr[6]=0;
		arr[7]=0;
	HAL_CAN_AddTxMessage(hcan,&TxMessage,arr,&Tx_Mailbox);
}
/**********************************CAN数据的接收********************************/
	
/**
	**************************************************************
	** Descriptions:获取电机返回值的偏差值
	** Input: 	
	**			  ptr:目标数据的内存地址
	**				hcan->pRxMsg->Data:保存的来自CAN的数据的数组
	** Output: NULL
	**************************************************************
**/
void get_moto_offset(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->offset_angle = ptr->angle;
}




	/**************************************************************
	** Descriptions: 获取CAN通讯的6623电机的返回值
	** Input: 	
	**			  ptr:目标数据的内存地址
	**				hcan->pRxMsg->Data:保存的来自CAN的数据的数组
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6623(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{                                                                                                                               

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->real_current  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->given_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->speed_rpm = ptr->real_current;
//	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}


void get_moto_measure_3508(moto_measure_t *ptr,uint8_t CAN_RX_date[])   //3508，2006电机获取值
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->speed_rpm  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->real_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->hall = CAN_RX_date[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/************************************传送电机堵转检测*******************************/
	uint32_t HAL_GetSystemTimer()
{
	return HAL_GetTick();
}

int Check_stir_locked(void)    //转盘电机堵转检测
{
  moto_stir_get.cmd_time = HAL_GetSystemTimer();//刷新当前时间
  if(!stir_locktime)
  {
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_stir_get.total_angle) >= 70)
						{
							
										moto_stir_get.round_cnt=0;
										moto_stir_get.offset_angle=moto_stir_get.angle;
										moto_stir_get.total_angle=0;
										moto_stir_get.run_time=HAL_GetSystemTimer();//未堵转时间
                    return 0;//未堵转
						}
						else if( my_abs(moto_stir_get.run_time-moto_stir_get.cmd_time)>STIR_BLOCK_TIME )//堵转判定
						{
                    stir_locktime = 75;
									  return 1;	//堵转
            }
            else return 0;//堵转时间继续加力
   }
    else return 0;
}

/*************************************云台->底盘(发送)***************************************/
//void CAN_Send_YT( CAN_HandleTypeDef * hcan,uint16_t yaw_angle,uint16_t yaw_total_angle , uint8_t flag ){  //云台yaw轴及一些标志位给底盘
//	CAN_TxHeaderTypeDef TxMessage;
//	TxMessage.DLC=0x08;
//	TxMessage.RTR=CAN_RTR_DATA;
//	TxMessage.IDE=CAN_ID_STD;
//	TxMessage.StdId=CAN_Yaw;
//	uint8_t CAN_DATA[8];
//	
//	CAN_DATA[0]=yaw_angle>>8;
//	CAN_DATA[1]=yaw_angle;
//	CAN_DATA[2]=yaw_total_angle>>8;
//	CAN_DATA[3]=yaw_total_angle;
//  CAN_DATA[4]=flag;
//	CAN_DATA[5]=0;
//	CAN_DATA[6]=0;
//	CAN_DATA[7]=0;
//	
//	HAL_CAN_AddTxMessage(hcan,&TxMessage,CAN_DATA,(uint32_t *)CAN_TX_MAILBOX1);
//}

void CAN_Send_YK( CAN_HandleTypeDef * hcan,int16_t key_v, int16_t rc_ch0, int16_t rc_ch1, uint8_t rc_s1, uint8_t rc_s2,uint8_t flag){  //遥控器数据
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.StdId=CAN_Remote;
	uint8_t CAN_DATA[8];
	
	
	CAN_DATA[0]=key_v>>8;
	   CAN_DATA[1]=key_v;
	CAN_DATA[2]=rc_ch0>>8;
	CAN_DATA[3]=rc_ch0;
		CAN_DATA[4]=rc_ch1>>8;
		CAN_DATA[5]=rc_ch1;
		CAN_DATA[6]=(rc_s1<<4)|rc_s2;
		CAN_DATA[7]=flag;
	HAL_CAN_AddTxMessage(hcan,&TxMessage,CAN_DATA,(uint32_t *)CAN_TX_MAILBOX1);
}




/*************************************底盘->云台(接收)****************************************/
void CAN_RX_Dipan(uint8_t CAN_RX_date_Dipan[])        //底盘数据接收
{
	dstVmmps_W=CAN_RX_date_Dipan[0];
	
	
	
	
}

