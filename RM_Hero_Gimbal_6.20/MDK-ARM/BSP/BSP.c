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
}//A��������Դ��Ҫ��������



/**
	**************************************************************
	** Descriptions:��ʼ�����ţ����ڣ�can��
	** Input: 	
  ** Output: NULL				
	**					
	** ע�⣺��cubemx�ٴ�����ʱ��main������ӵ����Щ��ʼ����������Ҫ����ɾ��				
	** 
	**************************************************************
**/
	void BSP_Init(void)
{
	/*����*/
	MX_GPIO_Init();
  MX_DMA_Init();
	Power_Init();
  /*CAN*/
	MX_CAN1_Init();
  MX_CAN2_Init();
	
	/*SPIͨ��*/
	MX_SPI5_Init();
	
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();   //������û�д���2
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_UART8_Init();
	delay_init();
	
	/*ADC*/
//	MX_ADC1_Init();
	
	
	HAlCANFilter1();
	HAlCANFilter2();
	HAL_CAN_Start(&hcan1);   
	HAL_CAN_Start(&hcan2);//can1,2�Ŀ���
	
	 MPU6500_Init();    //���������ǳ�ʼ��
	 
	 	Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,18);
	//ң������������
	 
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_FULL);   //can1,2�����жϿ���
	
	



	
	
	
	/*����*/
	

	
  
	//Bsp_UART_Receive_IT(&huart8,USART8_RX_DATA,SizeofJY61);
 
 
	
  
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofReferee);  //�Ӳ���ϵͳ����
	
	
	
	
	
	//JY�����ǽ�������
	

  

 

	/*�����ж�*/
	
//	JY61_Frame();
//�����ǳ�ʼ��  
	
}
