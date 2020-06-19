/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "data_pro_task.h"
#include "gimbal_task.h"
#include "gun_task.h"
#include "offline_check.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId RemoteDataTaskHandle;  
osThreadId GimbalTaskHandle;		
osThreadId GunTaskHandle; 
osThreadId MiniPCDataTaskHandle;


osThreadId TestOfUsartHandle;
osThreadId TestOfCanHandle;
osThreadId TestOfLedHandle;
osThreadId TestOfMochaHandle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//  /* USER CODE BEGIN RTOS_THREADS */
//	osThreadDef(Task_Remote, Remote_Data_Task, osPriorityHigh, 0, 256);
//  RemoteDataTaskHandle = osThreadCreate(osThread(Task_Remote), NULL);  //遥控器数据处理最高优先级
//
//	osThreadDef(Task_MiniPC, MiniPC_Data_task, osPriorityAboveNormal, 0, 128);
//  RemoteDataTaskHandle = osThreadCreate(osThread(Task_MiniPC), NULL);  //视觉数据处理
//	
//	
//	
//	osThreadDef(Gimbal_Task, Gimbal_Contrl_Task, osPriorityNormal, 0, 256);
//  GimbalTaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL); 
//	
//  osThreadDef(Gun_Task, Gun_Task, osPriorityNormal, 0, 128);
//  GunTaskHandle = osThreadCreate(osThread(Gun_Task), NULL); 
	
	  osThreadDef(testOfMocha_Task, testOfMochaTask, osPriorityHigh, 0, 256);
	  TestOfMochaHandle=osThreadCreate(osThread(testOfMocha_Task), NULL);
		
//		osThreadDef(testOfCan_Task, testOfCan, osPriorityHigh, 0, 256);
//		TestOfCanHandle=osThreadCreate(osThread(testOfCan_Task),NULL);
//		
//		osThreadDef(testOfLed_Task, testOfLed, osPriorityHigh, 0, 128);
//		TestOfLedHandle=osThreadCreate(osThread(testOfLed_Task),NULL);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
