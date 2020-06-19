#include "gun_task.h"

#include "pid.h"
#include "cmsis_os.h"
#include "CAN_USE_Moto.h"
#include "can.h"
#include "SystemState.h"
#include "gimbal_task.h"
#include "data_pro_task.h"
#include "minipc.h"
/**************************�ⲿ����*******************************/
uint16_t remain_heat = 0;      //ǹ������ʣ��ֵ
uint8_t MoCa_Flag = 0;    //Ħ���ֵ��������־λ
uint8_t GunReady = 0;    //����������
Heat_Gun_t  ptr_heat_gun_t;
uint8_t Bullets_count=0;



/***************************�ڲ�����/����******************************/
#define GUN_PERIOD  10

#define fixed_set_yaw1 1235
#define fixed_set_yaw2 7895
#define fixed_set_yaw3 8855
#define fixed_set_yaw4 9965

uint16_t set_yaw1;
uint16_t set_yaw2;
uint16_t set_yaw3;
uint16_t set_yaw4;

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�
pid_t pid_shot_spd[2]  = {0};	//Ħ�����ٶȻ�
pid_t pid_stir_spd;  //ת�̵���ٶȻ�
pid_t pid_stir_pos;   //ת�̵��λ�û�
/**                                                           
	**************************************************************
	** Descriptions: Ħ���ֺͲ���pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/
void Gun_Pid_Init()
{
	/*������ת�̣����*/
	 PID_struct_init(&pid_stir_pos, POSITION_PID, 6000, 5000,
	                0.0f, 0.0f, 0.0f);
	 PID_struct_init(&pid_stir_spd, POSITION_PID, 6000, 5000,
	                0.0f, 0.0f, 0.0f);
	
	
  /*�������*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 20000, 5000,
									0.18f,	0.0f,	0.15f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.15f	);    //��������ѧУʱ������������
   /*Ħ����*/
  for(uint8_t i = 0;i<2;i++)
  {
    PID_struct_init(&pid_shot_spd[i], POSITION_PID, 6000, 5000,
									0.0f,	0.0f,	0.0f	); 
  }
}

/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	ǹ�ܷ�������
	*	@retval	
****************************************************************************************/


void Gun_Task(void const * argument)
{ 
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	uint8_t motor_stop_flag=0;   //����رձ�־λ
  int32_t set_angle = 0;   //��ǰ�Ĳ��ݽǶȣ�Ŀǰ��Ħ���ֲ���
	int32_t set_speed = 0;  
  int32_t set_M_speed = 0;   //����Ħ�����ٶ�
  uint8_t set_cnt = 0;    //ת��Ȧ��
	uint8_t contiue_flag = 0;   //���ڲ���ģʽ�л�ʱ��ת��
	int32_t set_stir_speed = 0;
	int32_t set_stir_angle=0;
	float k=1;   //����ת�ٱ���ֵ����Ҫ���丳ֵ��������
	Gun_Pid_Init();  //pid��ʼ��
	
	while(1)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
		
		if(stir_motor_flag == 1)   //ת�̵������������Ϊ�����͵�
		{
			switch(Bullets_count){
				case 0:{  //����3��
					set_stir_angle=set_stir_angle-1365*3*19;  //��ǰ��6�����ף�8191/6��1365����ת3/6Ȧ�����ٱ�19:1
				  break;
				}
				case 1:{   //����2��
					set_stir_angle=set_stir_angle-1365*2*19;  //ת2/6Ȧ
					break;
				}
				case 2:{   //����1��
					set_stir_angle=set_stir_angle-1365*1*19;   //ת1/6Ȧ
					break;
				}
				case 3:{   //�������Ϊ�������������;�У��ײ��ص翪�ػ�δ��⵽���������Ѿ�����
					set_stir_angle=set_stir_angle;   //����
					break;
				}
			}
			if(Check_stir_locked() || stir_locktime)   //��ת�ж�
         {
           stir_locktime--;
           set_stir_speed = pid_calc(&pid_stir_pos,moto_stir_get.total_angle,set_stir_angle);
         }
       else set_stir_speed = -800;  //��ס�����ת
         
			pid_calc(&pid_stir_spd, moto_stir_get.speed_rpm, set_stir_speed); 
			Chassis_Motor(&hcan2,0,0,0,pid_stir_spd.pos_out);      //ת�̵�����ݵķ���
		}
		else 
		{
			set_stir_speed=0;
			Chassis_Motor(&hcan2,0,0,0,set_stir_speed);      //ת�̵�����ݵķ���
		}

		switch(MoCa_Flag)
		{
			case 0:   //�ر�Ħ����
			{
				set_M_speed=0;
				ptr_heat_gun_t.sht_flg=GunHold;
				break;
			}
			case 1:
			{
				set_M_speed=6000;   //Ħ�����ٶ�Ϊ6000����������Ϊ18m/s���ٶ�
			  break;
			}
			default :break;
			
			if(minipc_flag==1){
				if(chassis_gimble_Mode_flg==3)  //������ģʽ״̬
			{
				set_yaw1 = fixed_set_yaw1-dstVmmps_W * k;
				set_yaw2 = fixed_set_yaw2+dstVmmps_W * k;
				set_yaw3 = fixed_set_yaw3-dstVmmps_W * k;
				set_yaw4 = fixed_set_yaw4+dstVmmps_W * k;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)||
					minipc_rx.state_flag==0){   
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			else{
				set_yaw1 = fixed_set_yaw1;
				set_yaw2 = fixed_set_yaw2;
				set_yaw3 = fixed_set_yaw3;
				set_yaw4 = fixed_set_yaw4;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)||
					minipc_rx.state_flag==0){     //����ģʽ��yaw��Ƕȷ�Χ���Ӿ���Ŀ�꣨������ͷ����)��ֹͣ����
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			}
			else{
				//��ֹ��֧�Ÿ�
			if(chassis_gimble_Mode_flg==3)  //������ģʽ״̬
			{
				set_yaw1 = fixed_set_yaw1-dstVmmps_W * k;
				set_yaw2 = fixed_set_yaw2+dstVmmps_W * k;
				set_yaw3 = fixed_set_yaw3-dstVmmps_W * k;
				set_yaw4 = fixed_set_yaw4+dstVmmps_W * k;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle&&yaw_get.angle<set_yaw4)){
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			else{
				set_yaw1 = fixed_set_yaw1;
				set_yaw2 = fixed_set_yaw2;
				set_yaw3 = fixed_set_yaw3;
				set_yaw4 = fixed_set_yaw4;
				if((set_yaw1<yaw_get.angle&&yaw_get.angle<set_yaw2)||(set_yaw3<yaw_get.angle)&&(yaw_get.angle<set_yaw4)){
					ptr_heat_gun_t.sht_flg=GunHold;
				}
			}
			}
			
			
			
			if(remain_heat<100)//�������Ʋ��֣����Կ����Ż������������ʴﵽ���)
			{
				ptr_heat_gun_t.sht_flg=GunStop;
			}
			
      
			
			
      switch(ptr_heat_gun_t.sht_flg)//�жϷ���ģʽ
			{
				case GunStop:
				{
					
					switch(contiue_flag)
					{
						case 0:
						{   /*�趨�Ƕ�(������һ�εĽǶ�ֵ)*/
							set_angle=moto_dial_get.total_angle;
							contiue_flag=1;
							break;
						}
						case 1:
						{
							goto position;
							break;
						}
					}
					break;
				}
				
				case GunOne:    //ʹ��Ħ��������  (����ģʽ��
				{
					set_angle=set_angle-5400*36 ;   //���ٱ�Ϊ36:1(ת2/3Ȧ)�൱�ڷ���һ�ŵ���
					Bullets_count--;
					ptr_heat_gun_t.sht_flg = GunHold;  //������������һ�κ󣩱�صȴ�ģʽ
          contiue_flag = 0;
					goto position;    //��һ�ξͽ���λ�û����Ƿ������⣬�����ԣ�����
					break;
				}
				
				case GunFire:   //(����ģʽ)
				{
					set_angle=set_angle-5400*36*3;   //3����
					Bullets_count-=3;
					ptr_heat_gun_t.sht_flg = GunHold;  //������������һ�κ󣩱�صȴ�ģʽ
					contiue_flag = 0;
					goto position; 
					break;
				}
				
				case GunHold:
				{
					position:
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	

				  set_speed=pid_dial_pos.pos_out;
					break;
				}
				default :break;
			}
			
			//��������ͷ���Ħ�����ٶȻ�
			pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
      pid_calc(&pid_shot_spd[0],moto_M_get[0].speed_rpm ,-set_M_speed);
      pid_calc(&pid_shot_spd[1],moto_M_get[1].speed_rpm ,set_M_speed);
			
			//�������������Ħ����
			Shot_Motor(&hcan2,pid_dial_spd.pos_out,pid_shot_spd[0].pos_out,pid_shot_spd[1].pos_out);		
				
			//set_speed=0; //�����������		
				

			

			}
			osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}

}


















