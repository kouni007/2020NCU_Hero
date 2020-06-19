#include "stm32f4xx_hal.h"

/********************pid结构体定义***************************/
enum {
	
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3];				//
    float get[3];				//
    float err[3];				//


    float pout;							
    float iout;							
    float dout;							
	  float pout_max;
	  float iout_max;
	  float dout_max;
	  float dout_last;
	  float dout_new;

    float pos_out;						//
    float last_pos_out;				//
    float delta_u;						//
    float delta_out;					// = last_delta_out + delta_u
    float last_delta_out;

    float max_err;
    float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//
    uint32_t IntegralLimit;		//

    void (*f_param_init)(struct __pid_t *pid,  //PID?????
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid??????

} pid_t;

#define ABS(x)		((x>0)? (x): (-x)) 


/*************************************提供外部接口函数*******************************/
float LPF_1st(float oldData, float newData, float lpf_factor);
void ABS_limit(float *a, float ABS_MAX);
void Chassis_pid_init(void);      //pid初始化
float pid_calc(pid_t* pid, float get, float set);     //pid解算
void PID_struct_init(
											pid_t* pid,
											uint32_t mode,
											uint32_t maxout,
											uint32_t intergral_limit,   
											float 	kp, 
											float 	ki, 
											float 	kd);










