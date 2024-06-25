/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines -----------------------------------------------------------*/
#define SBUS_RX_BUF_NUM     36u
#define RC_FRAME_LENGTH     18u
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

#define abs(x) ((x)>0? (x):(-(x)))       //绝对值宏定义

//鼠标位移速度
#define    MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
#define    MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )


/*鼠标速度最大值限制*/
#define Xmax    660
#define Ymax    660

/*鼠标滑动滤波长度*/
#define SF_LENGTH 30  

/* 检测键盘按键状态 */
#define KEY_UP                    0x00
#define KEY_DOWN                  0x01

/*按键时间 长短按的判断*/
#define SHORT_CHANGE_TIM            5   //ms
#define LONG_CHANGE_TIM_W      		300		//ms
#define LONG_CHANGE_TIM_S      		300		//ms
#define LONG_CHANGE_TIM_A      		300		//ms
#define LONG_CHANGE_TIM_D      		300		//ms
#define LONG_CHANGE_TIM_Q      		300 	//ms
#define LONG_CHANGE_TIM_E      		300 	//ms
#define LONG_CHANGE_TIM_R      		300 	//ms
#define LONG_CHANGE_TIM_F      		500 	//ms
#define LONG_CHANGE_TIM_G      		300 	//ms
#define LONG_CHANGE_TIM_Z      		150 	//ms
#define LONG_CHANGE_TIM_X      		150 	//ms
#define LONG_CHANGE_TIM_C      		300 	//ms
#define LONG_CHANGE_TIM_V      		300 	//ms
#define LONG_CHANGE_TIM_B      		300 	//ms
#define LONG_CHANGE_TIM_CTRL   		300		//ms
#define LONG_CHANGE_TIM_SHIFT  		300 	//ms
#define LONG_CHANGE_TIM_MOUSE_L		300 	//ms
#define LONG_CHANGE_TIM_MOUSE_R		100 	//ms

//鼠标按下为1
#define    MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
#define    MOUSE_PRESSED_RIGH    (remote_ctrl.mouse.press_r)

//按键按下为1
#define    KEY_PRESSED_W       remote_ctrl.key.set.W
#define    KEY_PRESSED_S       remote_ctrl.key.set.S
#define    KEY_PRESSED_A       remote_ctrl.key.set.A
#define    KEY_PRESSED_D       remote_ctrl.key.set.D
#define    KEY_PRESSED_Q       remote_ctrl.key.set.Q
#define    KEY_PRESSED_E       remote_ctrl.key.set.E
#define    KEY_PRESSED_G       remote_ctrl.key.set.G
#define    KEY_PRESSED_X       remote_ctrl.key.set.X
#define    KEY_PRESSED_Z       remote_ctrl.key.set.Z
#define    KEY_PRESSED_C       remote_ctrl.key.set.C
#define    KEY_PRESSED_B       remote_ctrl.key.set.B
#define    KEY_PRESSED_V       remote_ctrl.key.set.V
#define    KEY_PRESSED_F       remote_ctrl.key.set.F
#define    KEY_PRESSED_R       remote_ctrl.key.set.R
#define    KEY_PRESSED_CTRL    remote_ctrl.key.set.CTRL
#define    KEY_PRESSED_SHIFT   remote_ctrl.key.set.SHIFT


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint32_t online_cnt;   /*!< online count */

} Remote_Info_Typedef;

/*按键状态枚举*/
typedef enum
{
	UP,			//抬起
	SHORT_DOWN,	//短按
	DOWN,		//长按
	PRESS,		//0->1
	RELAX,		//1->0
	KEY_STATE_CNT,
}KEY_SET_STATE;

/*单独按键信息*/
typedef struct 
{  
  uint16_t state_cnt;
  uint8_t State;
  KEY_SET_STATE prev_State;
  bool prev_KEY_PRESS;
  bool KEY_PRESS;
}KEY_SET_INFO_T;

/*总体键盘按键信息*/
typedef struct
{
	KEY_SET_INFO_T W;
	KEY_SET_INFO_T S;
	KEY_SET_INFO_T A;
	KEY_SET_INFO_T D;
	KEY_SET_INFO_T SHIFT;
	KEY_SET_INFO_T CTRL;
	KEY_SET_INFO_T Q;
	KEY_SET_INFO_T E;
	KEY_SET_INFO_T R;
	KEY_SET_INFO_T F;
	KEY_SET_INFO_T G;
	KEY_SET_INFO_T Z;
	KEY_SET_INFO_T X;
	KEY_SET_INFO_T C;
	KEY_SET_INFO_T V;
	KEY_SET_INFO_T B;
	KEY_SET_INFO_T MOUSE_L;
	KEY_SET_INFO_T MOUSE_R;

}KEYBOARD_INFO_T;

extern KEYBOARD_INFO_T Key_Info;


bool Key_SHIFT(void);
bool Key_MOUSE_L(void);
bool Key_MOUSE_R(void);
bool Key_R(void);
void Key_Q(void);
void Key_E(void);
bool Key_B(void);
bool Key_Z(void);
bool Key_X(void);
bool Key_C(void);
bool Key_V(void);
bool Key_F(void);
bool Key_G(void);

float Mouse_X_Speed(void);
float Mouse_Y_Speed(void);
float Mouse_Z_Speed(void);

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl);

#endif //REMOTE_CONTROL_H

