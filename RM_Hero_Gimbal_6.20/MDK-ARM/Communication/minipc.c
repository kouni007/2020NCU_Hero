#include "minipc.h"




Minipc_Rx minipc_rx;
uint8_t USART8_RX_DATA[SizeofMinipc];   //接收minipc数据


/**
	**************************************************************
	** Descriptions: 获取来自minipc的数据
	** Input: NULL
	**			  
	** Output: NULL
	**************************************************************
**/
void Get_MiniPC_Data(void)
{
		uint8_t *buff = USART8_RX_DATA;
			
		minipc_rx.frame_header = buff[0];
		minipc_rx.frame_tail 	 = buff[8];
		if((minipc_rx.frame_header == 0xFF) && (minipc_rx.frame_tail == 0xFE))
		{
			minipc_rx.angle_yaw  = (int16_t)(buff[1] << 8 | buff[2]);
			minipc_rx.angle_pit  = (int16_t)(buff[3] << 8 | buff[4]);
			minipc_rx.state_flag = buff[5];  //0无目标，1有目标，2低速，3中速，4高速
			minipc_rx.distance   = buff[6]<<8|buff[7]; 
		}
}
