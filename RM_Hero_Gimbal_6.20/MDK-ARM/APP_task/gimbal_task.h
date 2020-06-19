#include "main.h"


/* 本模块向外部提供的数据类型定义--------------------------------------------*/
typedef struct{
		//int16_t expect;
		float expect;
    float expect_last;
    float ture_value;
		uint8_t	step;
		uint8_t mode;
		int16_t expect_pc;
} Pos_Set;


/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/
extern uint8_t round_flag;    //分离模式变量
extern uint8_t back_flag;    //跟随模式

extern Pos_Set  yaw_set;
extern Pos_Set  yaw_set_jy61;
extern Pos_Set  pit_set;



/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gimbal_Contrl_Task(void const * argument);


void testOfCan();