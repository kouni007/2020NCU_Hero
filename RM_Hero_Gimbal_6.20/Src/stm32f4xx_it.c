/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_USE_Moto.h"
#include "JY61.h"
#include "SystemState.h"
#include "cmsis_os.h"
#include "minipc.h"
#include "gun_task.h"
#include "Remote.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern  osThreadId RemoteDataTaskHandle;
extern  osThreadId RefereeDataTaskHandle;
extern  osThreadId	MiniPCDataTaskHandle; 
extern osThreadId TestOfUsartHandle;
extern osThreadId TestOfCanHandle;
extern osThreadId TestOfLedHandle;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/



/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	 static  BaseType_t  pxHigherPriorityTaskWoken=pdFALSE;
 uint8_t temp1,temp2;
	uint32_t flag;
	temp1=__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	temp2=__HAL_UART_GET_IT_SOURCE(&huart1,UART_FLAG_IDLE);
	
	if(temp1!=RESET){

		 __HAL_DMA_DISABLE(&hdma_usart1_rx);
		   
		flag=__HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx);	
	
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,flag);
		
		__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,18);
		
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
   
//		vTaskNotifyGiveFromISR(TestOfUsartHandle,&pxHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
		
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);	
  /* USER CODE END USART1_IRQn 0 */
 HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	}
  /* USER CODE END USART1_IRQn 1 */
}


/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
// static  BaseType_t  pxHigherPriorityTaskWoken;
//	uint8_t tmp1,tmp2;
//	uint32_t flag;
//	tmp1 = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
//  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE);
//  /* USER CODE END USART6_IRQn 0 */
//	if((tmp1 != RESET) && (tmp2 != RESET))
//  { 
//    
//	__HAL_DMA_DISABLE(&hdma_usart2_rx);
//		
//	flag = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart2_rx);	
//	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,flag);
//  //  Get_MiniPC_Data();
//	__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,SizeofMinipc);
//	__HAL_DMA_ENABLE(&hdma_usart2_rx);

//	vTaskNotifyGiveFromISR(MiniPCDataTaskHandle,&pxHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);		

//				/*清除IDLE标志位*/
//   __HAL_UART_CLEAR_IDLEFLAG(&huart2);		
//	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE);
	
   if((tmp1 != RESET))
	{
		

		
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		
		//RefreshDeviceOutLineTime(JY61_NO);
		
		USART6_RX_NUM=(SizeofReferee)-(hdma_usart6_rx.Instance->NDTR);
		 Referee_Data_Handler();
		
		__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofJY61);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
	}

  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}


void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart8, UART_IT_IDLE);
	
   if((tmp1 != RESET))
	{
		

		
		__HAL_DMA_DISABLE(&hdma_uart8_rx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);
		
		//RefreshDeviceOutLineTime(JY61_NO);
		
		USART6_RX_NUM=(SizeofJY61)-(hdma_uart8_rx.Instance->NDTR);
		
		JY61_Data_Pro();
		__HAL_DMA_SET_COUNTER(&hdma_uart8_rx,SizeofJY61);
    __HAL_DMA_ENABLE(&hdma_uart8_rx);
	}

  /* USER CODE END USART8_IRQn 0 */
//  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART8_IRQn 1 */
}


/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	
	if (htim->Instance == TIM6) {
		RefreshSysTime();      //定时为1ms，每1ms进入中断，刷新时间
		
	}
	

}



void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){    //can1回调函数，返回数值
if(hcan->Instance == CAN1)	 
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,Receive_DATA);

		switch(RxMessage.StdId)
		{
//			case CAN_6623_YAW:   //底盘程序要和此一致（需检查）！！！！！
//			{
//       if(yaw_get.msg_cnt++ <= 10)
//				{
//					yaw_get.angle = 7462 ;            //基准位置选定(需要到学校实地测量)
//          yaw_get.offset_angle = 7462;;
//				}
//				else{
//					yaw_get.msg_cnt = 11;
//					get_moto_measure_6623(&yaw_get,Receive_DATA);
//					RefreshDeviceOutLineTime(Yaw_NO);
//				}
//			}	
//			break;
//			case CAN_6623_PIT:
//			{
//       if(pit_get.msg_cnt++ <= 10)    //msg_cnt有什么用处  （回答：复位作用)
//				{
//					pit_get.angle = 7000 ;            //基准位置选定(需要到学校实地测量)
//          pit_get.offset_angle = 7000;
//				}else
//				{
//					pit_get.msg_cnt = 11;
//					get_moto_measure_6623(&pit_get,Receive_DATA);
//					RefreshDeviceOutLineTime(Pit_NO);
//				}
//			break;
//			}
      /*************测试的can1************/
			case 0x201:{
				if(moto_stir_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_test_dial_get,Receive_DATA);
				}
				else
				{		
					moto_stir_get.msg_cnt=51;	
					get_moto_measure_3508(&moto_test_dial_get, Receive_DATA);
			}
				break;}
				default:break;
		}
		HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
	}
	if(hcan->Instance == CAN2)	 
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxMessage,Receive_DATA);

		switch(RxMessage.StdId)
		{
//			case CAN_2006_B:
//			{
//				if(moto_stir_get.msg_cnt++ <= 50)	
//				{
//					get_moto_offset(&moto_dial_get,Receive_DATA);
//				}
//				else
//				{		
//					moto_stir_get.msg_cnt=51;	
//					get_moto_measure_3508(&moto_dial_get, Receive_DATA);
//					RefreshDeviceOutLineTime(Bodan_NO);
//				}
//			break;
//			}
//			case CAN_3508_M1:
//			{
//				if(moto_stir_get.msg_cnt++ <= 50)	
//				{
//					get_moto_offset(&moto_M_get[0],Receive_DATA);
//				}
//				else
//				{		
//					moto_stir_get.msg_cnt=51;	
//					get_moto_measure_3508(&moto_M_get[0], Receive_DATA);
//					RefreshDeviceOutLineTime(McCa1_NO);
//				}
//			break;
//			}
//			case CAN_3508_M2:
//			{
//				if(moto_stir_get.msg_cnt++ <= 50)	
//				{
//					get_moto_offset(&moto_M_get[1],Receive_DATA);
//				}
//				else
//				{		
//					moto_stir_get.msg_cnt=51;	
//					get_moto_measure_3508(&moto_M_get[1], Receive_DATA);
//					RefreshDeviceOutLineTime(McCa2_NO);
//				}
//			break;
//			}
//			case CAN_3508_Stir:
//			{
//				if(moto_stir_get.msg_cnt++ <= 50)	
//				{
//					get_moto_offset(&moto_stir_get,Receive_DATA);
//				}
//				else
//				{		
//					moto_stir_get.msg_cnt=51;	
//					get_moto_measure_3508(&moto_stir_get, Receive_DATA);
//					RefreshDeviceOutLineTime(Bopan_NO);
//			  }
//			break;
//		}
      /*************测试的can2************/

       case 0x201:{
				if(moto_stir_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_test_dial_get,Receive_DATA);
				}
				else
				{		
					moto_stir_get.msg_cnt=51;	
					get_moto_measure_3508(&moto_test_dial_get, Receive_DATA);
			}
				break;}
			 			case CAN_3508_M1:
			{
				if(moto_stir_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_M_get[0],Receive_DATA);
				}
				else
				{		
					moto_stir_get.msg_cnt=51;	
					get_moto_measure_3508(&moto_M_get[0], Receive_DATA);
					RefreshDeviceOutLineTime(McCa1_NO);
				}
			break;
			}
			case CAN_3508_M2:
			{
				if(moto_stir_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_M_get[1],Receive_DATA);
				}
				else
				{		
					moto_stir_get.msg_cnt=51;	
					get_moto_measure_3508(&moto_M_get[1], Receive_DATA);
					RefreshDeviceOutLineTime(McCa2_NO);
				}
			break;
			}
			default:break;
		}
		HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_FULL);
	}
	

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){   //GPIO中断函数，上方光电开关检测一次进入中断
	if(GPIO_Pin == GPIO_PIN_4){
		Bullets_count++;
	
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
