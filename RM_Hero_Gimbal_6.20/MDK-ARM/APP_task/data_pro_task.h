#include "main.h"



#define  NORMAL_SPEED_MAX 	1000
#define  NORMAL_SPEED_MIN  -1000

#define  ACC_SPEED    30 
#define  DEC_SPEED    300 
/*°´¼ü*/
#define PRESS_DELAY  10
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

#define Left_Press          (RC_Ctl.mouse.press_l==1)
#define Left__NoPress       (RC_Ctl.mouse.press_l==0)
#define Right_Press         (RC_Ctl.mouse.press_r==1)
#define Right_NoPress       (RC_Ctl.mouse.press_r==0)

#define W_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
#define S_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
#define A_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
#define D_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)

#define SHIFT_Press         (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)
#define CTRL_Press          (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL)

#define Q_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
#define E_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)
#define R_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)
#define F_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)
#define G_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)
#define Z_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z)
#define X_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)
#define C_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)
#define V_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)
#define B_Press             (RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)




void Remote_Data_Task(void);
void MiniPC_Data_task(void const * argument);
extern uint8_t minipc_flag;
void testOfUsart();