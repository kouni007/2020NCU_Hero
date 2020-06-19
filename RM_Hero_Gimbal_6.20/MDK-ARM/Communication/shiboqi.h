#include "usart.h"

#include "stdio.h"


void vcan_sendware(uint8_t *wareaddr, uint32_t waresize);
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );
void Usart_SendArray( UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);