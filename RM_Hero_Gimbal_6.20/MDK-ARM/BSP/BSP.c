#include "BSP.h"

#include "CAN_USE_Moto.h"
#include "Remote.h"
#include "stm32f4xx_it.h"
#include "JY61.h"
#include "minipc.h"
#include "MPU6500.h"


void Power_Init(void){
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);   //power1
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);   //power2
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);   //power3
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //power4
}//A板其他电源需要打开其引脚



/**
	**************************************************************
	** Descriptions:初始化引脚，串口，can等
	** Input: 	
  ** Output: NULL				
	**					
	** 注意：当cubemx再次生成时在main函数中拥有这些初始化函数，需要将其删除				
	** 
	**************************************************************
**/
	void BSP_Init(void)
{
	/*引脚*/
	MX_GPIO_Init();
  MX_DMA_Init();
	Power_Init();
  /*CAN*/
	MX_CAN1_Init();
  MX_CAN2_Init();
	
	/*SPI通信*/
	MX_SPI5_Init();
	
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();   //板子里没有串口2
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_UART8_Init();
	delay_init();
	
	/*ADC*/
//	MX_ADC1_Init();
	
	
	HAlCANFilter1();
	HAlCANFilter2();
	HAL_CAN_Start(&hcan1);   
	HAL_CAN_Start(&hcan2);//can1,2的开启
	
	 MPU6500_Init();    //板载陀螺仪初始化
	 
	 	Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,18);
	//遥控器接收数据
	 
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_FULL);   //can1,2接收中断开启
	
	



	
	
	
	/*串口*/
	

	
  
	//Bsp_UART_Receive_IT(&huart8,USART8_RX_DATA,SizeofJY61);
 
 
	
  
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofReferee);  //接裁判系统数据
	
	
	
	
	
	//JY陀螺仪接收数据
	

  

 

	/*串口中断*/
	
//	JY61_Frame();
//陀螺仪初始化  
	
}
