#include "usart.h"

#include "stdio.h"



uint8_t ch;
uint8_t ch_r;


/*
********************************
*Descriptions��
*				�ض���c���printf����
*			�����ڣ��ķ��ͺ���
********************************
*/

// int fputc(int c, FILE * f)
//{
//     ch=c;
//     HAL_UART_Transmit(&huart3,&ch,1,1000);
//     return c;
// }
// ��ʾ�������Ѿ����壨�����Ƿ����Σ������ԣ�


 int fgetc(FILE * F)    
{
     HAL_UART_Receive (&huart3,&ch_r,1,0xffff);
     return ch_r;
}