#include "main.h"


/* ��ģ�����ⲿ�ṩ���������Ͷ���--------------------------------------------*/
typedef struct Heat_Gun_t
{
	int16_t  shted_bullet;
	int16_t limt_bullet;
	int16_t last_limt_bullet;
	uint16_t limt_heat;
	uint16_t rel_heat;
	uint16_t last_rel_heat;
	float    remain_power;
	uint8_t  limt_spd;
	uint8_t  roboLevel;
	uint8_t  sht_flg;
	uint8_t  stop_flg;
	uint8_t  heat_down_flg;
}Heat_Gun_t;

volatile typedef struct 
{
	volatile uint16_t rel_heat;
	volatile float remain_power;
}Power_Heat;

typedef enum
{
  GunStop = 0,
  GunOne,
  GunFire,
  GunHold
}GunMode;



/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/



/* ��ģ�����ⲿ�ṩ�Ľӿڱ�������--------------------------------------------*/
extern uint16_t remain_heat ;      //ǹ������ʣ��ֵ
extern uint8_t MoCa_Flag ;    //Ħ���ֵ��������־λ
extern uint8_t GunReady ;    //����������
extern Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t Bullets_count;
/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������----------------------------------------*/
void Gun_Task(void const * argument);
	