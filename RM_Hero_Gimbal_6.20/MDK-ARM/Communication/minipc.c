#include "minipc.h"




Minipc_Rx minipc_rx;
uint8_t USART8_RX_DATA[SizeofMinipc];   //����minipc����


/**
	**************************************************************
	** Descriptions: ��ȡ����minipc������
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
			minipc_rx.state_flag = buff[5];  //0��Ŀ�꣬1��Ŀ�꣬2���٣�3���٣�4����
			minipc_rx.distance   = buff[6]<<8|buff[7]; 
		}
}
