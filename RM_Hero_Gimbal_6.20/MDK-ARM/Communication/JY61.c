#include "JY61.h"


#include "usart.h"
#include "cmsis_os.h"
#include "pid.h"

 uint8_t USART6_RX_DATA[(SizeofReferee)];//JY61����
 uint16_t USART6_RX_NUM;
 
  struct STime			stcTime;
 struct SAcc 			stcAcc;
 struct SGyro 		stcGyro;
 struct SAngle 		stcAngle;
 struct SMag 			stcMag;
 struct SDStatus  stcDStatus;
 struct SPress 		stcPress;
 struct SLonLat 	stcLonLat;
 struct SGPSV 		stcGPSV;
 struct SQ        stcQ;
 
 
 //���������
JY61_t   ptr_jy61_t_yaw =  {0};
JY61_t   ptr_jy61_t_pit =  {0};
JY61_t  	ptr_jy61_t_angular_velocity = {0};

/*
** Descriptions: �޷��˲�
** Input:   ���ڵ���������
** Output: �˲����
** ע�⣺�ú�������Ӧ���ڹ������Ƶ��㷨�У���ʱ���õ��ˣ�����
*/
float Limit_filter(float oldData,float newData,float val)
{
	if(ABS(newData-oldData)>val)
		{
			return oldData;
		}
	else
		{
			return newData;
		}
}

/* �������岿�� --------------------------------------------------------------*/
/**
	**************************************************************
	** Descriptions:	JY61����/������
	** Input:	huart  ����ָ��Ĵ��ڣ�������Ҫ��Ϊ115200
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart)
{
	uint8_t buff[3] = {0xff,0xaa,0x60};
	//����,������
	HAL_UART_Transmit(huart,buff,3,10);
}

/**
	**************************************************************
	** Descriptions: JY61֡���뺯��
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/

void JY61_Frame(void)
{
	static uint8_t JY61_Frame_flag = 0;
	static	uint8_t JY61_Frame_Num = 0;
	
while( USART6_RX_DATA[0] != 0x55 ||  JY61_Frame_flag == 1)
{
	
	if(USART6_RX_DATA[0] != 0x55 && JY61_Frame_flag == 0)
	{
				
				HAL_UART_DMAPause(&huart6);
				*USART6_RX_DATA = 0;
				JY61_Frame_flag = 1;
				
	}
	if(JY61_Frame_flag == 1)//����һ�Σ����������
	{
			JY61_Frame_Num++;
			
					if(JY61_Frame_Num == 25)
					 {
						 
//								JY61_SLEEPorUNSLEEP(&huart4);
//								JY61_Frame_flag = 0;
//								JY61_Frame_Num = 0;
							
								HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofJY61);	//�����ǽ���

				   } else if(JY61_Frame_Num == 50)
							 {
								   HAL_UART_DMAResume(&huart6);
							 } else if(JY61_Frame_Num > 100  )
									 {
										 JY61_Frame_flag = 0;
							       JY61_Frame_Num = 0;
									 }

	 }
}
	
}



/**
	**************************************************************
	** Descriptions:JY61��ʼ������
	** Input: 	
  ** ע�⣺��ʱ��֪��������ʲô�����������в�û�е��ô˺�����					
	**					
	**					
	** Output: NULL
	**************************************************************
**/
/*void JY61_Init(void)
{
	uint8_t JY61[6][5] = {
													{0xff,0xaa,0x24,0x01,0x00},//�����㷨
													{0xff,0xaa,0x02,0x00,0x00},//�����Զ�У׼
													{0xff,0xaa,0x02,0x0c,0x00},//�ش�����:0x0c������ٶȺͽǶ�//0x08��ֻ����Ƕ�
													{0xff,0xaa,0x03,0x0b,0x00},//�ش�����:200hz
													{0xff,0xaa,0x00,0x00,0x00},//���浱ǰ����
													{0xff,0xaa,0x04,0x06,0x00}//���ô��ڲ�����:115200
												};

		
	HAL_UART_Transmit_DMA(&huart4,JY61[2],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart4,JY61[3],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart4,JY61[4],5);	
	HAL_Delay(100);
	if(HAL_UART_Transmit_DMA(&huart4,JY61[2],5) == HAL_OK )	
	{
		printf("JY61 Init \n\r");
	}
		if(HAL_UART_Transmit_DMA(&huart4,JY61[4],5) == HAL_OK)	
	{
		printf("JY61 Init save\n\r");
	}
}
*/






/***************************************************************************************
**
	*	@brief	JY61_Data_Pro()
	*	@param
	*	@supplement	���ж��б����ã����ڴ��ڽ��������ǵ����ݣ��������ݽ��д���
	*	@retval	
****************************************************************************************/
static unsigned char ucRxCnt = 0;
static unsigned char buff_transition[11] = {0}; //���ɶλ���
static unsigned char buff_last[SizeofJY61] = {0};

/**********************jy601***********************/

void select(uint8_t * buff,uint8_t i)
{
	switch(buff[i+1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{
//		case 0x50:	memcpy(&stcTime,&buff[i+2],8);		break;
		case 0x51:	memcpy(&stcAcc,&buff[i+2],8);			break;
		case 0x52:	memcpy(&stcGyro,&buff[i+2],8);		break;
		case 0x53:	memcpy(&stcAngle,&buff[i+2],8);		break;
//		case 0x54:	memcpy(&stcMag,&buff[i+2],8);			break;
//		case 0x55:	memcpy(&stcDStatus,&buff[i+2],8);	break;
//		case 0x56:	memcpy(&stcPress,&buff[i+2],8);		break;
//		case 0x57:	memcpy(&stcLonLat,&buff[i+2],8);	break;
//		case 0x58:	memcpy(&stcGPSV,&buff[i+2],8);		break;
//		case 0x59:	memcpy(&stcQ,&buff[i+2],8);				break;
	}
}

void JY61_Data_Pro()
{	
	
		portTickType xLastWakeTime;
	  portTickType xWakeTime;
		xWakeTime = xTaskGetTickCount();
	
  int16_t data_sum=0;
	static int16_t i=0;
	uint8_t  * buff = USART6_RX_DATA;//����6
	
	uint8_t JY_NUM = 0;
	static uint8_t buff_last[11] = {0};  	 //��һ֡�Ĳ�������
	static uint8_t index = 0;		
	
	//jy601
	switch(buff[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[2],8);		  break;
		case 0x53:	memcpy(&stcAngle,&buff[2],8);		  break;

	}
	
	
		switch(buff[12])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[13],8);		  break;
		case 0x53:	memcpy(&stcAngle,&buff[13],8);		  break;

	}
	
		
		switch(buff[23])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[24],8);		  break;
		case 0x53:	memcpy(&stcAngle,&buff[24],8);		  break;

	}
	
	//jy61
	/*Ѱ�ҵ�һ��֡ͷλ��*/
//	for(uint8_t i = 0;i < SizeofJY61/3;i++)
//	{
//		if(buff[i] == 0x55)
//		{
//			select(buff,i);
//			ucRxCnt = i;
//			break;
//		}
//	}
//	/*������ɶλ���*/
//			memcpy(buff_transition,&buff_last[ucRxCnt + 11],11 - ucRxCnt - 1);
//			memcpy(&buff_transition[11 - ucRxCnt - 1],buff_last,ucRxCnt + 1);
//			//�����ϴλ���ֵ
//			memcpy(buff_last,buff,11);
//			
//			select(buff_transition,0);

//	printf("���ٶ�:%3f,�Ƕ�:%3f\n\r",(float)stcGyro.w[0]/32768*2000,(float)stcAngle.Angle[0]/32768*180);	
	

	
			ptr_jy61_t_pit.JY61_angle = (float)stcAngle.Angle[1]*0.005493f;
			ptr_jy61_t_yaw.JY61_angle = (float)stcAngle.Angle[2]*0.005493f;	
	   
//		if(pritnf_JY61){    //������
//			printf("gz=%f\n",ptr_jy61_t_yaw.JY61_angle);
//    	float *ptr = NULL; //��ʼ��ָ��
//			ptr = &(ptr_jy61_t_yaw.final_angle);	
//			/*������ʾ��������������*/
//			vcan_sendware((uint8_t *)ptr,sizeof(ptr_jy61_t_yaw.final_angle));
//		 }
		if(ptr_jy61_t_yaw.times>5)
			{
//			ptr_jy61_t_pit.JY61_angle = Limit_filter(ptr_jy61_t_pit.JY61_angle_last,ptr_jy61_t_pit.JY61_angle,30);
//	    ptr_jy61_t_yaw.JY61_angle = Limit_filter(ptr_jy61_t_yaw.JY61_angle_last,ptr_jy61_t_yaw.JY61_angle,30);
				ptr_jy61_t_yaw.times=6;
				ptr_jy61_t_yaw.err=ptr_jy61_t_yaw.JY61_angle-ptr_jy61_t_yaw.JY61_angle_last;
			if(ptr_jy61_t_yaw.err<-180)  
				ptr_jy61_t_yaw.angle_round++;
			else if(ptr_jy61_t_yaw.err>180)  
				ptr_jy61_t_yaw.angle_round--;
			ptr_jy61_t_yaw.final_angle=(ptr_jy61_t_yaw.angle_round*360+ptr_jy61_t_yaw.JY61_angle-ptr_jy61_t_yaw.first_angle)*22.75f;
			
			ptr_jy61_t_pit.err=ptr_jy61_t_pit.JY61_angle-ptr_jy61_t_pit.JY61_angle_last;
			if(ptr_jy61_t_pit.err<-180) 
				ptr_jy61_t_pit.angle_round++;
			else if(ptr_jy61_t_pit.err>180)
				ptr_jy61_t_pit.angle_round--;
			//�������ս��
		  ptr_jy61_t_pit.final_angle=(ptr_jy61_t_pit.angle_round*360+ptr_jy61_t_pit.JY61_angle-ptr_jy61_t_pit.first_angle);
//		  ptr_jy61_t_pit.final_angle=(ptr_jy61_t_pit.JY61_angle-ptr_jy61_t_pit.first_angle);
			i=(xWakeTime-xLastWakeTime);
		  xLastWakeTime=	xWakeTime;
			}
			else 
			{
				ptr_jy61_t_yaw.first_angle = ptr_jy61_t_yaw.JY61_angle;
				ptr_jy61_t_pit.first_angle = ptr_jy61_t_pit.JY61_angle;
			}
			

			ptr_jy61_t_yaw.JY61_angle_last = ptr_jy61_t_yaw.JY61_angle;
			ptr_jy61_t_pit.JY61_angle_last = ptr_jy61_t_pit.JY61_angle;
			ptr_jy61_t_yaw.times++;

		
			

			ptr_jy61_t_angular_velocity.vx = stcGyro.w[0] * 0.06103516f;
			ptr_jy61_t_angular_velocity.vy = stcGyro.w[1] * 0.06103516f;
			ptr_jy61_t_angular_velocity.vz = stcGyro.w[2] * 0.06103516f;			
			
			ptr_jy61_t_angular_velocity.vz = Limit_filter(ptr_jy61_t_angular_velocity.vz_last,ptr_jy61_t_angular_velocity.vz,700);
			ptr_jy61_t_angular_velocity.vy = Limit_filter(ptr_jy61_t_angular_velocity.vy_last,ptr_jy61_t_angular_velocity.vy,700);
			
			ptr_jy61_t_angular_velocity.vy_last = ptr_jy61_t_angular_velocity.vy;
			ptr_jy61_t_angular_velocity.vz_last = ptr_jy61_t_angular_velocity.vz;
			



}

/**********************jy601***********************/