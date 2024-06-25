            /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Shoot_Task.c
  * @brief          : Shoot_Task
  * @author         : Wang Jinjing
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Shoot_Task.h"
#include "Vision_Task.h"
#include "System_Task.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "bsp_can.h"
#include "motor.h"
#include "remote_control.h"
#include "api_trajectory.h"

/**
  * @brief 
  */
Shoot_Info_Typedef Shoot_Info; 

int SHOOT_SPEED_16M_S = 5800;//5850；   6600
int SHOOT_SPEED_10M_S = 4200;
float Gimbal_Speed_Update;
bool Gimbal_Speed_Update_FLAG = false;


int Shoot_speed_mode = 0;
uint8_t Shoot_TxFrameData[8];

/* Private variables ---------------------------------------------------------*/
/**
  * @brief 
  */
PID_Info_TypeDef Shoot_Pid[2][2];
//P I D DeadBand IntegeralMAX OutputMAX
float Shoot_Pid_Para[3][PID_PARAMETER_NUM] = {
    [0] = {15, 0.f, 0, 0, 10000, 16000},       // SHOOTL/R_SPEED
    [1] = {24, 0.1f, 2, 0, 10000, 16000},      // TRIGGER_SPEED
    [2] = {180, 0.f, 0, 0, 0, 10000},          // TRIGGER_ANGLE
}; 

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief 
 */
static void Shoot_Task_Init(void);
static void Shoot_Ctrl(Shoot_Info_Typedef *Shoot_Info);

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the shoot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
	Shoot_Task_Init();
  /* Infinite loop */
  for(;;)
  {
		if(chassis.ctrl == 1)
		{
			if(remote_ctrl.rc.s[1] == 2 && remote_ctrl.rc.s[0] == 3) Shoot_speed_mode = 0;
			else Shoot_speed_mode = 1;
			if(remote_ctrl.rc.s[1] == 1 && remote_ctrl.rc.s[0] == 2) Vision_Info_mode = 1;
		  else Vision_Info_mode = 0;
		}
		
		if(chassis.ctrl == 2)
		{
			if(Key_B() == true)
			{
				Shoot_Info.mode = SHOOT_ON;
			}
			else if(Key_B() == false)
			{
				Shoot_Info.mode = SHOOT_OFF;
			}
			
			if(Key_G() == true) 
				Unlimited_flag=1;
			else if(Key_G() == false)
				 Unlimited_flag=0;	
			
						
			if(Key_Z() == true && Gimbal_Speed_Update_FLAG == 0)
			{
				Gimbal_Speed_Update_FLAG = 1;
				Gimbal_Speed_Update += 100;
			}
			else if(Key_X() == true && Gimbal_Speed_Update_FLAG == 0)
			{
				Gimbal_Speed_Update_FLAG = 1;
				Gimbal_Speed_Update -= 100;
			}
			else if(Key_V() == false || Key_C() == false)
			{
				Gimbal_Speed_Update_FLAG = 0;
			}
			
			
			if(Key_MOUSE_R() == true)
			{
				Vision_Info_mode = 1;
				vision_flag = 1;
			}
			else
			{
				Vision_Info_mode = 0;
				vision_flag = 0;
			}

		}
		
		Shoot_Ctrl(&Shoot_Info);
		
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */

//------------------------------------------------------------------------------
/**
 * @brief 
 */
static void Shoot_Task_Init(void)
{
	PID_Init(&Shoot_Pid[0][0], PID_POSITION, Shoot_Pid_Para[0]);  // SHOOTL_SPEED
	PID_Init(&Shoot_Pid[0][1], PID_POSITION, Shoot_Pid_Para[0]);  // SHOOTR_SPEED
	PID_Init(&Shoot_Pid[1][0], PID_POSITION, Shoot_Pid_Para[1]);  // TRIGGER_SPEED
	PID_Init(&Shoot_Pid[1][1], PID_POSITION, Shoot_Pid_Para[2]);  // TRIGGER_ANGLE
	
	Shoot_Info.target.trigger_Angle = DJI_Motor[TRIGGER].Data.angle;
}

/**
 * @brief 
 */
static float SpeedAdapt_16M(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res = 0;
	
  if(real_S < 14.3f) res += 1.8f * up_num;
	else if(real_S < min_S && real_S > 14.3f) res += up_num;
  else if(real_S >= min_S && real_S <= max_S )res = 0;
	else if(real_S > max_S && real_S <15.5f) res -= down_num;
	else if(real_S >15.5f)res -= 1.5f * down_num;

  return res;
}

/**
 * @brief 
 */
float Last_Speed;
float Speed_Add;
bool FIRE_SINGLE_Flag = false;
bool Key_Shoot_Flag = false;
int Unlimited_flag = false;
uint16_t Fire_Vision_Cnt;
static bool IF_SHOOT = false;

static void Shoot_Ctrl(Shoot_Info_Typedef *Shoot_Info)
{
	if(Vision_Info_mode == 1)
	{
		if(Vision_Info.IF_Aiming_Enable && Shoot_Info->mode == SHOOT_ON)
		{
			Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_16M_S + Gimbal_Speed_Update;
			
			if(FIRE_SINGLE_Flag == false && Vision_Info.IF_Fire_Accept)
			{
					Shoot_Info->target.trigger_Angle -= (remote_ctrl.rc.s[0] == 2 || Key_MOUSE_R() == 1) * 158.f;
					FIRE_SINGLE_Flag = true;
			}
		}
		
		if(ABS(remote_ctrl.rc.ch[1]) > 0  && FIRE_SINGLE_Flag == false)
		{
				Shoot_Info->target.trigger_Angle -= remote_ctrl.rc.ch[1] / ABS(remote_ctrl.rc.ch[1])*158.f;
				FIRE_SINGLE_Flag = true;
		}
		
		if(FIRE_SINGLE_Flag == true)
		{
			Fire_Vision_Cnt++;
			if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100)
			{
				if(Fire_Vision_Cnt++ > 3000)
				{
					if(ABS(Shoot_Pid[1][1].Err[0]) < 10.f && (ABS(remote_ctrl.rc.ch[1]) == 0))
					{
						FIRE_SINGLE_Flag = false;
					}
					Fire_Vision_Cnt = 0;
				}
			}

		}
	 }
	else if(Vision_Info_mode == 0)
	{
		Fire_Vision_Cnt = 0;
		
//		if(ABS(remote_ctrl.rc.ch[1]) > 0 && Shoot_Info->mode == SHOOT_ON)
//		{
//			Shoot_Info->target.trigger_Angle -= remote_ctrl.rc.ch[1] / ABS(remote_ctrl.rc.ch[1])*158.f;
//			FIRE_SINGLE_Flag = true;
//		}
//		
//		if(FIRE_SINGLE_Flag == true)
//		{
//			Fire_Vision_Cnt++;
//			if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100)
//			{
//				if(Fire_Vision_Cnt++ > 3000)
//				{
//					if(ABS(Shoot_Pid[1][1].Err[0]) < 5.f && ABS(remote_ctrl.rc.ch[1]) == 0)
//					{
//						FIRE_SINGLE_Flag = false;
//				}
//				Fire_Vision_Cnt = 0;
//			}
//		  }	
//		}

	}
	
	if(chassis.ctrl == 1)//遥控
	{
		if(Shoot_speed_mode == 0)
		{
			Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_10M_S;
		}
		else if(Shoot_speed_mode == 1)
		{
			Last_Speed = robot.bullet_speed;
			if(robot.bullet_speed != Last_Speed)
			{
				Speed_Add += SpeedAdapt_16M(robot.bullet_speed,15.f,15.8f,15.f,20.f);
			}
				Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_16M_S + Speed_Add;
		}
		
		if(ABS(remote_ctrl.rc.ch[1]) > 0  && FIRE_SINGLE_Flag == false)
		{
			Shoot_Info->target.trigger_Angle -= remote_ctrl.rc.ch[1] / ABS(remote_ctrl.rc.ch[1])*158.f;
			FIRE_SINGLE_Flag = true;
		}
		
		if(FIRE_SINGLE_Flag == true)
		{
			if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100)
			{
				if(ABS(Shoot_Pid[1][1].Err[0]) < 5.f && ABS(remote_ctrl.rc.ch[1]) == 0)
				{
					FIRE_SINGLE_Flag = false;
				}
		  }	
		}
	}
	
	else if(chassis.ctrl == 2 && Shoot_Info->mode == SHOOT_ON)//键盘
	{
		if(Shoot_Info->mode==SHOOT_ON)
		{
			switch(robot.mode)
			{
				case INITIAL:
						Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_16M_S + Gimbal_Speed_Update;

						break;
				case BURST:
						Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_10M_S;

						break;
				case RATE:
						Shoot_Info->target.target_wheel_speed = SHOOT_SPEED_16M_S + + Gimbal_Speed_Update;
				
						break;          
				
				default:
					Shoot_Info->target.target_wheel_speed = 0;
						break;
			 }
		}

		if(Key_MOUSE_L() == false) IF_SHOOT = true;
		
		if(IF_SHOOT == true && Key_MOUSE_L() == true)
		{
			if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100 || Unlimited_flag)//发射受枪口热量限制
			{
				Shoot_Info->target.trigger_Angle -=  158.f;
			}
			
			IF_SHOOT = false;
		}
			
	}
	
	if(Shoot_Info->mode == SHOOT_OFF)
	{
		Shoot_Info->target.target_wheel_speed = 0;
		Shoot_Info->target.trigger_Angle = DJI_Motor[TRIGGER].Data.angle;
	}
	
	Shoot_Info->SendValue[0] =  f_PID_Calculate(&Shoot_Pid[0][0], Shoot_Info->target.target_wheel_speed, DJI_Motor[SHOOTL].Data.velocity);
	Shoot_Info->SendValue[1] =  f_PID_Calculate(&Shoot_Pid[0][1],-Shoot_Info->target.target_wheel_speed, DJI_Motor[SHOOTR].Data.velocity);
	f_PID_Calculate(&Shoot_Pid[1][1], Shoot_Info->target.trigger_Angle, DJI_Motor[TRIGGER].Data.angle);
	Shoot_Info->SendValue[2] = f_PID_Calculate(&Shoot_Pid[1][0], Shoot_Pid[1][1].Output, DJI_Motor[TRIGGER].Data.velocity);

	Shoot_TxFrameData[0] =  (uint8_t)(Shoot_Info->SendValue[0] >> 8);
	Shoot_TxFrameData[1] =  (uint8_t)(Shoot_Info->SendValue[0]);
	Shoot_TxFrameData[2] =  (uint8_t)(Shoot_Info->SendValue[1] >> 8);
	Shoot_TxFrameData[3] =  (uint8_t)(Shoot_Info->SendValue[1]);
	
	USER_CAN_TxMessage(CAN1,0x200,Shoot_TxFrameData);    
}


//------------------------------------------------------------------------------




