#include "main.h"


/* ��ģ�����ⲿ�ṩ���������Ͷ���--------------------------------------------*/
typedef struct{
		//int16_t expect;
		float expect;
    float expect_last;
    float ture_value;
		uint8_t	step;
		uint8_t mode;
		int16_t expect_pc;
} Pos_Set;


/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿڳ�������--------------------------------------------*/
extern uint8_t round_flag;    //����ģʽ����
extern uint8_t back_flag;    //����ģʽ

extern Pos_Set  yaw_set;
extern Pos_Set  yaw_set_jy61;
extern Pos_Set  pit_set;



/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������----------------------------------------*/
void Gimbal_Contrl_Task(void const * argument);


void testOfCan();