#include "offline_check.h"

#include "SystemState.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "CAN_USE_Moto.h"
#include "can.h"

void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
				if((SystemState.OutLine_Flag & 0x001))//遥控器数据接收掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED1_Pin);  //led1闪烁
					  

						osDelayUntil(&xLastWakeTime,200);		  
				}else  
				{
						
						HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_SET);  //否则常亮
				}
				
				if((SystemState.OutLine_Flag & 0x002))  //JY61数据接收掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED2_Pin);
					  

						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_SET);

				}

				if((SystemState.OutLine_Flag & 0x004))  //裁判系统数据掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED3_Pin);

						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_SET);
				}
				if((SystemState.OutLine_Flag & 0x008)||(SystemState.OutLine_Flag & 0x010))  //发弹摩擦轮其中之一数据接收掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED4_Pin);

						osDelayUntil(&xLastWakeTime,200);
				} else 
        {	
					HAL_GPIO_WritePin(GPIOG,LED4_Pin,GPIO_PIN_SET);
				}
				if((SystemState.OutLine_Flag & 0x020)||(SystemState.OutLine_Flag & 0x040))  //PIT,YAW其中之一数据接收掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED5_Pin);
						osDelayUntil(&xLastWakeTime,200);
					  
				} else  
				{
					HAL_GPIO_WritePin(GPIOG,LED5_Pin,GPIO_PIN_SET);
				}
				if((SystemState.OutLine_Flag & 0x080))  //拨弹电机数据接收掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED6_Pin);
						osDelayUntil(&xLastWakeTime,200);
					  
				} else  
				{
					HAL_GPIO_WritePin(GPIOG,LED6_Pin,GPIO_PIN_SET);
				}
				if((SystemState.task_OutLine_Flag & 0x100))  //拨盘电机数据接收掉线
				{
					HAL_GPIO_TogglePin(GPIOG,LED7_Pin);
					osDelayUntil(&xLastWakeTime,200);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOG,LED7_Pin,GPIO_PIN_SET);
				}
				if((SystemState.task_OutLine_Flag & 0x001))  //遥控器程序掉线
				{
					HAL_GPIO_TogglePin(GPIOG,LED8_Pin);

					osDelayUntil(&xLastWakeTime,200);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOG,LED8_Pin,GPIO_PIN_SET);

				}
				if((SystemState.task_OutLine_Flag & 0x002))  //云台程序掉线
				{
					HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);
					osDelayUntil(&xLastWakeTime,100);
					Disable_Gimbal_Motor(&hcan1);
				}
				else
				{
					HAL_GPIO_WritePin(LED_Red_GPIO_Port,LED_Red_Pin,GPIO_PIN_RESET);
				}
				if((SystemState.task_OutLine_Flag & 0x004))  //枪管程序掉线
				{
					HAL_GPIO_TogglePin(LED_Green_GPIO_Port,LED_Green_Pin);
					osDelayUntil(&xLastWakeTime,100);
					Shot_Motor(&hcan2,0,0,0);
				}
				else
				{
					HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Green_Pin,GPIO_PIN_RESET);
				}
	  	osDelayUntil(&xLastWakeTime,LED_PERIOD); 
		
	 }
 
}




//断线检测
void vOutLineCheck_Task(void const *argument)
{

	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		//RefreshTaskOutLineTime(vOutLineCheckTask_ON);
		
		TASK_Check();       //任务断线检测
		OutLine_Check();    //模块断线检测
		osDelayUntil(&xLastWakeTime,20);
		
	}
}

/***************************************************************************************
**
	*	@brief	testOfLed(void const * argument)
	*	@param
	*	@supplement	测试每一个led灯的运行情况，闪烁
	*	@retval	
****************************************************************************************/

void testOfLed(){
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	while(1){
		
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin 
                          |LED4_Pin|LED3_Pin, GPIO_PIN_SET);

     HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin 
                          |LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

     HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
//		HAL_GPIO_TogglePin(GPIOG,LED1_Pin);  //led1闪烁
//		HAL_Delay(200);
//		HAL_GPIO_TogglePin(GPIOG,LED2_Pin);  //led1闪烁
//		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOG,LED3_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOG,LED4_Pin);  //led1闪烁
    HAL_Delay(200);	
		HAL_GPIO_TogglePin(GPIOG,LED5_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOG,LED6_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOG,LED7_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOG,LED8_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(LED_Green_GPIO_Port,LED_Green_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);  //led1闪烁
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin 
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

     HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
		osDelayUntil(&xLastWakeTime,2);
	}

	
}





