/*******************************************************************************
  文 件 名   : communication.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2019年1月
  最近修改   :
  功能描述   : 麦轮解算
  函数列表   :void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
																			float dstVmmps_Y,float dstVmmps_W)

*******************************************************************************/
#include "mecanum_calc.h"

void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
{
			wheel[0] = (-dstVmmps_X + dstVmmps_Y + dstVmmps_W);
			wheel[1] = (-(dstVmmps_X + dstVmmps_Y - dstVmmps_W));
			wheel[2] = (-(-dstVmmps_X + dstVmmps_Y - dstVmmps_W));
			wheel[3] = (dstVmmps_X + dstVmmps_Y + dstVmmps_W);	
}

