#include "shiboqi.h"



/****************�����������������´���****************
int16_t angle1[4];
int16_t  *ptr = angle1; //��ʼ��ָ��
			angle1[0]	= (int16_t)(100);
			angle1[1]	= (200);
			angle1[2]	= ((int16_t)300);
			angle1[3]	= (int16_t)(400);
			������ʾ��������������
			vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[0]));
		  vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[1]));
		  vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[2]));
		  vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[3]));
*****************************************************/


#ifdef __GNUC__ /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */ 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 
#else 
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 
#endif /* __GNUC__ */ /** * @brief Retargets the C library printf function to the USART. * @param None * @retval None */ 
PUTCHAR_PROTOTYPE { 
//while(HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX){}
HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0); 
return ch; }//���ڵ��Ա���

#define uart_putbuff Usart_SendArray


/******** ����һ���ַ� *********/
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch )
{
  HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 10); 
}
	

/******** ����8λ������ *********/
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num)
{
	uint8_t i;
	for(i=0;i<num;i++)
	{
		Usart_SendByte(huart,array[i]);
	}
		
}

void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
		#define CMD_WARE     0x03
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    uart_putbuff(&huart3, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    uart_putbuff(&huart3, wareaddr, waresize);    //��������
    uart_putbuff(&huart3, cmdr, sizeof(cmdr));    //���ͺ�����
}



