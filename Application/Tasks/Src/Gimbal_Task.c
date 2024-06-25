/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Gimbal_Task.c
  * @brief          : Gimbal_Task
  * @author         : Wang Jinjing
  * @date           : 2023/05/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Gimbal_Task.h"
#include "Vision_Task.h"
#include "Shoot_Task.h"
#include "System_Task.h"
#include "pid.h"
#include "bsp_can.h"
#include "motor.h"
#include "INS_Task.h"
#include "remote_control.h"

/**
  * @brief 
  */
Gimbal_Info_Typedef Gimbal_Info ={
    .dr16 = &remote_ctrl,
};

/* Private variables ---------------------------------------------------------*/
/**
  * @brief 
  */
PID_Info_TypeDef Gimbal_Pid[2][2];
//P I D DeadBand IntegeralMAX OutputMAX
float Gimbal_Pid_Para[4][PID_PARAMETER_NUM] = {
    [0] = {15, 0, 2, 0, 0, 0},       // YAW_SPEED
    [1] = {15, 0, 2, 0, 0, 0},              // YAW_ANGLE
    [2] = {20, 0, 0, 0, 0, 0},             // PITCH_SPEED
    [3] = {20 , 0, 0, 0, 0, 0},              // PITCH_ANGLE
}; 

uint8_t Gimbal_TxFrameData[8];

int vision_flag = 0;
int fire_flag = 0; 
float pit_motor_angle;//½«pitÖá±àÂëÆ÷Öµ×ª»¯ÎªpitÖáÊÀ½ç½Ç
int Vision_Info_mode = 0;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief 
 */
static void Gimbal_Posture_Ctrl(Gimbal_Info_Typedef *Gimbal_Info);
static void Gimbal_Task_Init(void);
static void Gimbal_Mode_Update(void);


/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the gimbaltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
	Gimbal_Task_Init();
  /* Infinite loop */
  for(;;)
  {
    switch(Gimbal_Info.status)
    {
        case ON:
            Gimbal_Info.target.yaw_Angle -= (Shoot_Info.mode == SHOOT_OFF)*remote_ctrl.rc.ch[0]*0.0002f;
            Gimbal_Info.target.pit_Angle += (Shoot_Info.mode == SHOOT_OFF)*remote_ctrl.rc.ch[1]*0.0002f;
						
        break;

        default:  
            Gimbal_Info.target.pit_Angle = INS_Info.pit_angle;
            Gimbal_Info.target.yaw_Angle = INS_Info.yaw_tolangle;
            Gimbal_Info.target.pit_hang_Angle = INS_Info.pit_angle;
            Gimbal_Info.target.yaw_hang_Angle = INS_Info.yaw_tolangle;
        break;
    }
		
		Key_Q();
		Key_E();
		
		Gimbal_Mode_Update();
		Gimbal_Posture_Ctrl(&Gimbal_Info);

    
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief 
 */
static void Gimbal_Task_Init(void)
{
	PID_Init(&Gimbal_Pid[0][0], PID_POSITION, Gimbal_Pid_Para[0]);  // YAW_SPEED
	PID_Init(&Gimbal_Pid[0][1], PID_POSITION, Gimbal_Pid_Para[1]);	// YAW_ANGLE
	PID_Init(&Gimbal_Pid[1][0], PID_POSITION, Gimbal_Pid_Para[2]);	// PITCH_SPEED
	PID_Init(&Gimbal_Pid[1][1], PID_POSITION, Gimbal_Pid_Para[3]);	// PITCH_ANGLE
	
}

/**
 * @brief 
 */
float Pitch_Angle_Update;
bool Pitch_Angle_Update_FLAG = false;

static void Gimbal_Posture_Ctrl(Gimbal_Info_Typedef *Gimbal_Info)
{		
		Gimbal_Info->target.yaw_Angle += -Mouse_X_Speed() * 0.0002f
															 -Gimbal_Info->dr16->rc.ch[0] * 0.000001f;
		Gimbal_Info->target.pit_Angle += -Mouse_Y_Speed() * 0.0005f
															 +(Shoot_Info.mode == SHOOT_OFF)*Gimbal_Info->dr16->rc.ch[1]*0.000001f;
	
		VAL_LIMIT(Gimbal_Info->target.pit_Angle,-16,45.f);

    f_PID_Calculate(&Gimbal_Pid[0][1], Gimbal_Info->target.yaw_Angle, INS_Info.yaw_tolangle);
    Gimbal_Info->SendValue[0] =  f_PID_Calculate(&Gimbal_Pid[0][0], Gimbal_Pid[0][1].Output, INS_Info.yaw_gyro);
		
    f_PID_Calculate(&Gimbal_Pid[1][1], Gimbal_Info->target.pit_Angle, INS_Info.pit_angle);
    Gimbal_Info->SendValue[1] = f_PID_Calculate(&Gimbal_Pid[1][0],Gimbal_Pid[1][1].Output, INS_Info.pit_gyro);
	
			if(Key_C() == false && Pitch_Angle_Update_FLAG == 0)
			{
				Pitch_Angle_Update_FLAG = 1;
				Pitch_Angle_Update += 0.015f;
			}
			else if(Key_V() == false  && Pitch_Angle_Update_FLAG == 0)
			{
				Pitch_Angle_Update_FLAG = 1;
				Pitch_Angle_Update -= 0.015f;
			}
			else if(Key_V() == true || Key_C() == true)
			{
				Pitch_Angle_Update_FLAG = 0;
			}
			
		if(vision_flag == 1 && Vision_Info.IF_Aiming_Enable == 1)
		{
			Gimbal_Info->target.yaw_Angle = Vision_Info.target_Yaw ;
			Gimbal_Info->target.pit_Angle =  Vision_Info.target_Pitch;
			
			if(fabsf(Gimbal_Info->target.yaw_Angle-INS_Info.yaw_angle)>180.f)
			{
				Gimbal_Info->target.yaw_Angle -=360;
			}
			else if(fabsf(Gimbal_Info->target.yaw_Angle-INS_Info.yaw_angle)<-180.f)
			{
				Gimbal_Info->target.yaw_Angle +=360;
			}
			
			f_PID_Calculate(&Gimbal_Pid[0][1], INS_Info.yaw_angle,Gimbal_Info->target.yaw_Angle);
			Gimbal_Info->SendValue[0] =  f_PID_Calculate(&Gimbal_Pid[0][0], Gimbal_Pid[0][1].Output, INS_Info.yaw_gyro) ;
			
			f_PID_Calculate(&Gimbal_Pid[1][1], Gimbal_Info->target.pit_Angle + Pitch_Angle_Update,INS_Info.pit_angle);
			Gimbal_Info->SendValue[1] = f_PID_Calculate(&Gimbal_Pid[1][0],Gimbal_Pid[1][1].Output, INS_Info.pit_gyro) ;
		}
	
		if(Gimbal_Info->status == OFF)
		{
			Gimbal_Info->SendValue[0] = 0;
			Gimbal_Info->SendValue[1] = 0;
		}
  
    Gimbal_TxFrameData[2] =  (uint8_t)(Gimbal_Info->SendValue[1] >> 8);
    Gimbal_TxFrameData[3] =  (uint8_t)(Gimbal_Info->SendValue[1]);
    USER_CAN_TxMessage(CAN1,0x1ff,Gimbal_TxFrameData);

    Gimbal_TxFrameData[0] =  (uint8_t)(Gimbal_Info->SendValue[0] >> 8);
    Gimbal_TxFrameData[1] =  (uint8_t)(Gimbal_Info->SendValue[0]);
		Gimbal_TxFrameData[6] =  (uint8_t)(Shoot_Info.SendValue[2] >> 8);
		Gimbal_TxFrameData[7] =  (uint8_t)(Shoot_Info.SendValue[2]);
		
		Gimbal_TxFrameData[4] = (uint8_t)((int)((pit_motor_angle+16.30f)*100) >> 8);
		Gimbal_TxFrameData[5] = (uint8_t)((int)((pit_motor_angle+16.30f)*100));
    USER_CAN_TxMessage(CAN2,0x1ff,Gimbal_TxFrameData);
}

/**
 * @brief 
 */
uint32_t currentTime;

//  L      R
//2,3,1  1,3,2
static void Gimbal_Mode_Update(void)
{	
	 currentTime = xTaskGetTickCount();

	 if(currentTime - remote_ctrl.online_cnt > 50)
	 {//Á¬½Ó³¬Ê±
      Gimbal_Info.status = OFF;
      Shoot_Info.mode = SHOOT_OFF;
      chassis.state_Setup(OFF);
		  chassis.mode_Setup(INVA);
   }

	if(chassis.ctrl == 2)//¼üÅÌ
	{
		Gimbal_Info.status = ON;
//		Shoot_Info.mode = SHOOT_ON;
		chassis.state_Setup(ON);
		chassis.mode_Setup(FOLO);
	
	}
	else if(chassis.ctrl == 1)//Ò£¿Ø
	{
		switch(remote_ctrl.rc.s[0])//ÓÒ²¦¸Ë 1£¬3£¬2
		{
			case 1:
				switch(remote_ctrl.rc.s[1])//×ó²¦¸Ë 2£¬3£¬1
				{
					case 2:
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_ON;
						chassis.state_Setup(ON);
						chassis.mode_Setup(INVA);
					  break;
					
					case 3:
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(OFF);
						chassis.mode_Setup(INVA);
					break;
					
					case 1:
						vision_flag = 0;
						Gimbal_Info.status = OFF;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(OFF);
						chassis.mode_Setup(INVA);
						break;
					
					default:
						break;
				}
			break;
				
			case 3:
				switch(remote_ctrl.rc.s[1])
				{
					case 2:					
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_ON;
						chassis.state_Setup(ON);
						chassis.mode_Setup(FOLO);
					  break;
					
					case 3:
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(OFF);
						chassis.mode_Setup(FOLO);
					break;
					
					case 1:
						vision_flag = 0;
						Gimbal_Info.status = OFF;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(OFF);
						chassis.mode_Setup(INVA);
						break;
					
					default:
						break;
				}
			break;
			
			case 2:
				switch(remote_ctrl.rc.s[1])
				{
					case 2:
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(ON);
						chassis.mode_Setup(SPIN);
					  break;
					
					case 3:
						vision_flag = 0;
						Gimbal_Info.status = ON;
						Shoot_Info.mode = SHOOT_OFF;
						chassis.state_Setup(OFF);
						chassis.mode_Setup(FOLO);
					break;
					
					case 1:
						vision_flag =1;
						Shoot_Info.mode = SHOOT_ON;
						Vision_Info_mode = 1;
						Gimbal_Info.status = ON;
//						Shoot_Info.mode = SHOOT_OFF;
//						chassis.state_Setup(OFF);
//						chassis.mode_Setup(FOLO);
						break;
					
					default:
						break;
				}
			break;
				
		default:
			break;
		}
	
	}
}
//------------------------------------------------------------------------------




