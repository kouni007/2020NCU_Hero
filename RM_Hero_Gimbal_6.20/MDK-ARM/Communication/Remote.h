#include "main.h"


typedef struct //遥控器及键鼠通道
		{ 
			int16_t x; //!< Byte 6-7 
			int16_t y; //!< Byte 8-9 
			int16_t z; //!< Byte 10-11 
			uint8_t press_l; //!< Byte 12 
			uint8_t press_r; //!< Byte 13 
    }Mouse; 
		
	typedef 	struct 
		{ 
	 uint16_t ch0; 
	 uint16_t ch1; 
	 uint16_t ch2; 
	 uint16_t ch3; 
		uint16_t s1;
			uint16_t s2;
		}Rc; 
		typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	
}moto3508_type;
	typedef struct 
		{ 
		  uint16_t v; //!< Byte 14-15 
		}Key; 
		typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 




extern uint8_t USART1_RX_DATA[18];
extern Rc rc;

extern float  wheel[4];
extern RC_Ctl_t RC_Ctl;      //遥控器数据(4个通道)

void Bsp_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Remote_Ctrl(void);
void RemoteControlProcess(void);

