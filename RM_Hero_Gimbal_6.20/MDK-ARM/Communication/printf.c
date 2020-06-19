#include "usart.h"

#include "stdio.h"



uint8_t ch;
uint8_t ch_r;


/*
********************************
*Descriptions：
*				重定向c库的printf函数
*			到串口３的发送函数
********************************
*/

// int fputc(int c, FILE * f)
//{
//     ch=c;
//     HAL_UART_Transmit(&huart3,&ch,1,1000);
//     return c;
// }
// 在示波器中已经定义（这里是否屏蔽，待测试）


 int fgetc(FILE * F)    
{
     HAL_UART_Receive (&huart3,&ch_r,1,0xffff);
     return ch_r;
}