#include "pid.h"

/*************************�ṹ�嶨��*****************************/

typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
	}moto_3508_type;

/*************************�ⲿ�ӿڱ���*****************************/
extern uint8_t round_flag;    //����ģʽ����
extern uint8_t back_flag;    //����ģʽ
extern int8_t chassis_disable_flg;   //����ֹͣ����
extern moto_3508_type moto_3508_set;
extern pid_t pid_chassis_follow;//���̸���λ�û�
extern pid_t pid_chassis_follow_spd;//���̸����ٶȻ�
