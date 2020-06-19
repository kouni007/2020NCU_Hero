#include "gimbal_task.h"

#include "pid.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "CAN_USE_Moto.h"
#include "JY61.h"
#include "MPU6500.h"
#include "SystemState.h"
#include "data_pro_task.h"

#include "Remote.h"
/**************************�ⲿ����*******************************/
Pos_Set  yaw_set;
Pos_Set  yaw_set_jy61;
Pos_Set  pit_set;

uint8_t round_flag=0;    //����ģʽ����
uint8_t back_flag=0;    //����ģʽ
	uint8_t a=0;
	int32_t set_angle1=0;
	int32_t set_speed1=0;


/***************************�ڲ�����/����******************************/
#define GIMBAL_PERIOD 5
pid_t pid_yaw_jy61 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_yaw_jy61_spd = {0};    //������ģʽ�µ�yaw��pid

pid_t pid_pit_start = {0};
pid_t pid_pit_start_spd = {0};
pid_t pid_yaw_start = {0};
pid_t pid_yaw_start_spd = {0};    //����ʱpit��yaw�����õ�pid

pid_t pid_yaw       = {0};  //������ģʽ��yaw��λ�û�
pid_t pid_pit       = {0};	//����pit��λ�û�
pid_t pid_yaw_spd   = {0};	//������ģʽ��yaw���ٶȻ�
pid_t pid_pit_spd   = {0};	//����pit���ٶȻ�

/*******���Ե�pid********/
pid_t pid_test_dial_pos;
pid_t pid_test_dial_spd;
/************************/


int8_t gimbal_disable_flg;   //��̨����ֹͣ��־λ

static  int16_t Yaw_Current_Value = 0;    //yaw�������ֵ
static  int16_t Pitch_Current_Value = 0;    //pit�������ֵ

/**                                                           
	**************************************************************
	** Descriptions: ��̨pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/

  /*imu pid parameter*/

	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									0.0f, 0.0f,0.0f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 0, 0,
                  0.0f, 0.0f,0.0f);
  
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  0.0f, 0.0f,0.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  0.0f, 0.0f,0.0f);



  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  0.0f, 0.0f, 0.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  0.0f, 0.0f, 0.0f );
  
   PID_struct_init(&pid_pit_start, POSITION_PID, 400, 230, 0.75f, 0.015f, 0.0f);              //����pid,��ƫ���Ƚϴ󣬸���ֵ��С
   PID_struct_init(&pid_pit_start_spd, POSITION_PID, 18000, 10000, 41.0f, 0.75f, 0.1f ); 
	
	 PID_struct_init(&pid_yaw_start, POSITION_PID, 400, 230, 0.85f, 0.018f, 0.0f);              //����pid
   PID_struct_init(&pid_yaw_start_spd, POSITION_PID, 18000, 10000, 45.0f, 0.75f, 0.1f ); 
	
	/************���Ե�pid��ʼ��*************/
PID_struct_init(&pid_test_dial_pos,POSITION_PID, 20000, 5000,
									0.18f,	0.0f,	0.15f); 
PID_struct_init(&pid_test_dial_spd, POSITION_PID, 20000, 5000,
									0.3f,	0.0f,	0.15f	);  									
	
}



/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	��̨�������
	*	@retval	

//�������������Ư�����Բɼ�������ֹһ��ʱ�����Ưֵ��ȥ��Ư����ͬʱ����һ��ģʽң������һ��ʱ��������֮����̨�����������ǿ��ơ�
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	//��ʼ��
	static uint16_t gimbal_start_count = 0;
	static uint8_t gimbal_start_mode = 1;//��̨��ʼ��ģʽ
	yaw_set.expect = 0; 
	pit_set.expect = 0;

	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	minipc_flag=0;
	
	gimbal_pid_init();  //pid��ʼ��
	
	
	
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
	while(1)
	{
		RefreshTaskOutLineTime(GimbalContrlTask_ON);
		IMU_Get_Data();
		if(gimbal_start_mode == 1 )         //pit������ 
			{
         chassis_gimble_Mode_flg = 4;  //pit������ģʽ
				
			}else if(gimbal_start_mode == 2 ) //pit��������yaw��������Ĺ���
			{
				 chassis_gimble_Mode_flg = 4;
				 gimbal_start_count++;
				 if(gimbal_start_count > 100 )   //����ʱ��ȴ�
				 {
						gimbal_start_mode = 3;  //yaw������
				 }
			}else if(gimbal_start_mode == 3 )    //yaw������
			{
         chassis_gimble_Mode_flg = 5;     //yaw������ģʽ    
			}                                              

			
		  if(gimbal_start_mode == 1)      //pit������ģʽ��
			{
		     if( ABS(pit_get.total_angle) <= 100)  //pit�ᵽ��Ŀ��λ�ø���
				 {
					 gimbal_start_mode = 2;   //���ɽ׶�
				 }
			}else if(gimbal_start_mode == 3)
			{
				if( ABS(yaw_get.total_angle) <= 100)  //yaw�ᵽ��Ŀ��λ�ø���
				 {
					 gimbal_start_mode = 0;   //�����׶ν���
					 chassis_gimble_Mode_flg=1;//������ģʽ

				 }
			}
			/*******pit����λ******************/
			
			
			
			
			/*********************************/
			if(minipc_flag==1)    //����ģʽ
			{
				pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect_pc);
				pid_calc(&pid_yaw_spd,(imu_data.gx), pid_yaw.pos_out);
				Yaw_Current_Value = pid_yaw_spd.pos_out;
				
				pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect_pc);
				pid_calc(&pid_pit_spd,(imu_data.gz), pid_pit.pos_out);   
				Pitch_Current_Value = pid_pit_spd.pos_out; 
			}
			else
			{
				switch(chassis_gimble_Mode_flg)
			{	
				case 1: //������ģʽ(����)
				{     
							pid_calc(&pid_yaw_jy61,ptr_jy61_t_yaw.final_angle,yaw_set_jy61.expect );  
							pid_calc(&pid_yaw_jy61_spd,(imu_data.gx),pid_yaw_jy61.pos_out);
					    Yaw_Current_Value = pid_yaw_jy61_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz), pid_pit.pos_out);          //(imu_data.gz)/16.4  Ϊ��/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
	
				}break;
				
        case 2: //������ģʽ (���룬Ӧ�ü�����������ģʽ�����ڿ��������)
			  { 
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx), pid_yaw.pos_out);
							Yaw_Current_Value = pid_yaw_spd.pos_out;
				
							pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_spd,(imu_data.gz), pid_pit.pos_out);   
							Pitch_Current_Value = pid_pit_spd.pos_out; 
					

			  }break;
				
			  case 3: //С����ģʽ����������ģʽ��ͬ�����ڵ��̿�����������  
			  {  
				      pid_calc(&pid_yaw_jy61,ptr_jy61_t_yaw.final_angle,yaw_set_jy61.expect );  
							pid_calc(&pid_yaw_jy61_spd,(imu_data.gx),pid_yaw_jy61.pos_out);
					    Yaw_Current_Value = pid_yaw_jy61_spd.pos_out;
					
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_spd, (imu_data.gz), pid_pit.pos_out);          //(imu_data.gz)/16.4  Ϊ��/s
              Pitch_Current_Value = pid_pit_spd.pos_out; 
					
				      
			  }break; 

				
				
				case 4:  //pit������������������pit�ᣬ������yaw�ᣩ
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
				}break;
				
				case 5:  //yaw������
				{
					    pid_calc(&pid_pit_start, pit_get.total_angle, 0);
						  pid_calc(&pid_pit_start_spd,(imu_data.gz)/16.4, pid_pit_start.pos_out);   
							Pitch_Current_Value = pid_pit_start_spd.pos_out;
					
					    pid_calc(&pid_yaw_start, yaw_get.total_angle, 0);
						  pid_calc(&pid_yaw_start_spd,(imu_data.gx)/16.4, pid_yaw_start.pos_out);
							Yaw_Current_Value = pid_yaw_start_spd.pos_out;
				}break;
				
			  default: break;
				
			 }   
			}
			
			
			if(gimbal_disable_flg==0)
			{
					Disable_Gimbal_Motor(&hcan1);
			}
			else 
			{
				Cloud_Gimbal_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
			}
				
		  /*�������ݸ����̲���*/
//				CAN_Send_YT(&hcan2,yaw_get.angle,yaw_get.total_angle,chassis_gimble_Mode_flg);
				
				
				
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
	}

}




/***************************************************************************************
**
	*	@brief	testOfCan(void const * argument)
	*	@param   ����can1,can2����1���������ֱ��can1��can2.
	*	@supplement	
	*	@retval	
****************************************************************************************/


 

void testOfCan(){

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	gimbal_pid_init();
	while(1){
		
  
		
//HAL_GPIO_TogglePin(GPIOG,LED2_Pin); 
	//	HAL_Delay(100);
	//	IMU_Get_Data();
//		if(rc.s1==3){
//			
//	}			//led1��˸}

		IMU_Get_Data();
		HAL_GPIO_TogglePin(GPIOG,LED2_Pin); 
		HAL_Delay(100);
    if(rc.s1==3)
		{
			if(rc.s2 ==3){
					a=1;
				}
				if(a==1&&rc.s2 ==2){
				  set_angle1=set_angle1-5400*36;   //���ٱ�Ϊ36:1
			    a=0;
				pid_calc(&pid_test_dial_pos, moto_test_dial_get.total_angle,set_angle1);	
				set_speed1=-pid_test_dial_pos.pos_out;
		    pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
				Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
					
				}    //���ݿ����жϣ���һ��תһ��
				
			  pid_calc(&pid_test_dial_pos, moto_test_dial_get.total_angle,set_angle1);	
				set_speed1=-pid_test_dial_pos.pos_out;
		    pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
				Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
//			set_speed1=1000;
//			pid_calc(&pid_test_dial_spd,moto_test_dial_get.speed_rpm ,set_speed1);
//      Chassis_Motor(&hcan2,pid_test_dial_spd.pos_out,0,0,0);
				
			}
		else {
			Chassis_Motor(&hcan2,0,0,0,0);
		}
		
			}
		
			
//				
				osDelayUntil(&xLastWakeTime,2);
	}

	
	
	