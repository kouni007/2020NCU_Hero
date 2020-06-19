#include "main.h"
#include "stm32f4xx_it.h"



typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFF
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//��ǰ״̬����0 δ��׼Ŀ�� ���� 1 ����������Ŀ�꡿��2 Զ��������Ŀ�꡿��3 ���ģʽ��
	int16_t 					distance;     			//Ŀ�����
	int16_t           angle;
unsigned char 		frame_tail; 	  	  //֡β0xFE
}Minipc_Rx;




extern uint8_t USART8_RX_DATA[SizeofMinipc]; 
extern Minipc_Rx minipc_rx;


void Get_MiniPC_Data(void);










