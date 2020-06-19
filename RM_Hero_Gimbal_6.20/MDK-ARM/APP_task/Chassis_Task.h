#include "pid.h"

/*************************结构体定义*****************************/

typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
	}moto_3508_type;

/*************************外部接口变量*****************************/
extern uint8_t round_flag;    //分离模式变量
extern uint8_t back_flag;    //跟随模式
extern int8_t chassis_disable_flg;   //紧急停止变量
extern moto_3508_type moto_3508_set;
extern pid_t pid_chassis_follow;//底盘跟随位置环
extern pid_t pid_chassis_follow_spd;//底盘跟随速度环
