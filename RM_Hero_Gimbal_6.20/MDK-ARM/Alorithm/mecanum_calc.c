/*******************************************************************************
  �� �� ��   : communication.c
  �� �� ��   : ����
  ��    ��   : NCUERM
  ��������   : 2019��1��
  ����޸�   :
  ��������   : ���ֽ���
  �����б�   :void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
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

