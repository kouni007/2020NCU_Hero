#include "test.h"


#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "shiboqi.h"
#include "usart.h"
#include "Remote.h"
#include "pid.h"
#include "can.h"
#include "data_pro_task.h"
#include "CAN_USE_Moto.h"
#include "protocol.h"

/***************************************************************************************
**
	*	@brief	testOfMochaTask()
	*	@param
	*	@supplement	�������ԣ�2006,3508Ħ���֣�
	*	@retval	
****************************************************************************************/

pid_t pid_testofMocha_dial_pos;
pid_t pid_testofMocha_dial_spd;
pid_t pid_test_shot_spd[2]  = {0};	//Ħ�����ٶȻ�

int32_t set_test_angle = 0;   //����λ��Ŀ��ֵ
int32_t set_test_M_speed = 0;    //Ħ�����ٶ�ֵ 
int32_t set_test_speed = 0;   //�����ٶ�Ŀ��ֵ
	uint8_t a_test=0;    //���ݿ��ر�־λ

void testOfMocha_pid_init(void)
{
	PID_struct_init(&pid_testofMocha_dial_pos,POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.7f); 
  PID_struct_init(&pid_testofMocha_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.7f	); 


/* �������
	λ�û�0.4  0.0 0.8     �ٶȻ�0.3  0.0  0.8    ��20000��5000��  �ٶ���ӦҪ��Ͽ켸���ﵽԤ��ֵ������У����ܻ��е�Ӱ����̨�ȶ�
	 d���0.8���ٴ����岻��dԽС�����ԽС����Ԥ��ֵԽƫ��ԽԶ    ����iֵ����������ض���


*/	

  for(uint8_t i = 0;i<2;i++)
  {
    PID_struct_init(&pid_test_shot_spd[i], POSITION_PID, 6000, 5000,
									2.0f,	0.01f,	0.5f	); 
  }
}

void testOfMochaTask(void const * argument)
{
	
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

	testOfMocha_pid_init();
	
	for(;;)
	{ 		
		Remote_Ctrl();  //ң�������ݴ���
		HAL_Delay(5);    //ң�������ݽ���

		if(rc.s1==3){
			set_test_M_speed=5500;
      pid_calc(&pid_test_shot_spd[0],moto_M_get[0].speed_rpm ,-set_test_M_speed);
      pid_calc(&pid_test_shot_spd[1],moto_M_get[1].speed_rpm ,set_test_M_speed);	
						if(rc.s2 ==3){
					a_test=1;
				}
				if(a_test==1&&rc.s2 ==2){
				  set_test_angle=set_test_angle-5400*36 ;   //���ٱ�Ϊ36:1
			    a_test=0;
					pid_calc(&pid_testofMocha_dial_pos, moto_test_dial_get.total_angle,set_test_angle);	
				set_test_speed=pid_testofMocha_dial_pos.pos_out;
		    pid_calc(&pid_testofMocha_dial_spd,moto_test_dial_get.speed_rpm ,set_test_speed);
				Chassis_Motor(&hcan2,pid_testofMocha_dial_spd.pos_out,0,0,0);
					
				}    //���ݿ����жϣ���һ��תһ��
			  pid_calc(&pid_testofMocha_dial_pos, moto_test_dial_get.total_angle,set_test_angle);	
				set_test_speed=pid_testofMocha_dial_pos.pos_out;
		    pid_calc(&pid_testofMocha_dial_spd,moto_test_dial_get.speed_rpm ,set_test_speed);
				Chassis_Motor(&hcan2,pid_testofMocha_dial_spd.pos_out,pid_test_shot_spd[0].pos_out,pid_test_shot_spd[1].pos_out,0);
		}
		else{
			Chassis_Motor(&hcan2,0,0,0,0);
		}
		
	
		int16_t angle1[4];
    int16_t  *ptr = angle1; //��ʼ��ָ��
		angle1[0]	= (int16_t)(2000);
		angle1[1]	= (int16_t)(-moto_M_get[0].speed_rpm);
		angle1[2]	= ((int16_t)moto_M_get[1].speed_rpm);
		angle1[3]	= (int16_t)(Robot.Chassis_Power.Chassis_Current);
		//������ʾ��������������
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[0]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[1]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[2]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[3]));	
		
		osDelayUntil(&xLastWakeTime,3);
	}
}
	