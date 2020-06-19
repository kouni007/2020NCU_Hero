#include "main.h"
#include "stm32f4xx_it.h"



typedef struct{

unsigned char 		frame_header; 		  //帧头0xFF
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//当前状态：【0 未瞄准目标 】【 1 近距离锁定目标】【2 远距离锁定目标】【3 打符模式】
	int16_t 					distance;     			//目标距离
	int16_t           angle;
unsigned char 		frame_tail; 	  	  //帧尾0xFE
}Minipc_Rx;




extern uint8_t USART8_RX_DATA[SizeofMinipc]; 
extern Minipc_Rx minipc_rx;


void Get_MiniPC_Data(void);










