#include "main.h"

#define FILTER_BUF_LEN		5

typedef struct{
	int16_t	 			speed_rpm;
	int16_t  			real_current;
	int16_t  			given_current;
	uint8_t  			hall;
	uint16_t 			angle;				//abs angle range:[0,8191]
	uint16_t 			last_angle;	//abs angle range:[0,8191]
	uint16_t			offset_angle;
	int32_t				round_cnt;
	int32_t				total_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
	int32_t      run_time;
	int32_t      cmd_time;
	int32_t      reverse_time;
	int32_t      REVE_time;
}moto_measure_t;



typedef enum
{
  /*CAN1*/
  CAN_6623_YAW = 0X205,
  CAN_6623_PIT = 0X206,
  /*云台->底盘*/
  CAN_Remote = 0x110,
  CAN_Yaw = 0x120,
  /*底盘->云台*/
  CAN_RX_B = 0X012,
  /*CAN2*/
  CAN_2006_B = 0X201,
	CAN_3508_M1 = 0x202,
	CAN_3508_M2 = 0x203,
	CAN_3508_Stir=0x204
}CAN_Message_ID;


#define my_abs(x) ((x)>0?(x):-(x)) //ABS宏定义



extern	uint8_t Receive_DATA[8];
extern moto_measure_t   moto_dial_get;  //c2006
extern moto_measure_t   moto_M_get[2]; 
extern moto_measure_t   moto_stir_get;
extern  moto_measure_t   yaw_get;	
extern moto_measure_t   pit_get;
extern uint32_t  Tx_Mailbox;
extern CAN_RxHeaderTypeDef  RxMessage;
extern uint32_t stir_locktime ;      //堵转判断  
extern moto_measure_t   moto_chassis_get[4];  //底盘电机数据返回值
extern uint8_t chassis_gimble_Mode_flg;   //云台底盘分离模式标志
extern uint8_t stir_motor_flag ;     //传送电机开启标志位（如果将传送电机放在云台上则不需要）
extern int16_t yaw_speed;       //yaw速度
extern uint8_t dstVmmps_W;

extern moto_measure_t moto_test_dial_get;

void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2,int16_t iq3, int16_t iq4);
void Disable_Chassis_Motor( CAN_HandleTypeDef * hcan);
void Cloud_Gimbal_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Disable_Gimbal_Motor(CAN_HandleTypeDef * hcan);
void get_moto_measure_6623(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void get_moto_offset(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void get_moto_measure_3508(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void Shot_Motor(CAN_HandleTypeDef * hcan,int16_t bo_value,int16_t M1_value,int16_t M2_value);
int Check_stir_locked(void)  ;
//void CAN_Send_YT( CAN_HandleTypeDef * hcan,uint16_t yaw_angle,uint16_t yaw_total_angle , uint8_t flag );
void CAN_Send_YK( CAN_HandleTypeDef * hcan,int16_t key_v, int16_t rc_ch0, int16_t rc_ch1, uint8_t rc_s1, uint8_t rc_s2,uint8_t flag);
