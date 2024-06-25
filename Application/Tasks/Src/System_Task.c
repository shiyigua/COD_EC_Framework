/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : System_Task.c
  * @brief          : System_Task
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
#include "System_Task.h"
#include "pid.h"
#include "bsp_can.h"
#include "motor.h"
#include "remote_control.h"
#include "Shoot_Task.h"
#include "Gimbal_Task.h"

/* Private variables ---------------------------------------------------------*/
/**
  * @brief 
  */
ROBOT robot=
{
    .mode=INITIAL,
    .level=LV_1,
};
CHASSIS chassis ={
		.ctrl =1,
    .state=OFF,
    .mode=FOLO,
    .state_Setup=chassis_State_Handoff,
		.mode_Setup=chassis_Mode_Handoff,
};
HEAD head ={
		.shoot_ready=SHOOT_OFF,
		.auto_ready=0,
//	  .supcap_ready=0,
};

bool SUPCAP_FLAG = false;
/* Private function prototypes -----------------------------------------------*/
static void chassis_Ctrl(void);

/**
 * @brief 
 */

/* USER CODE BEGIN Header_System_Task */
/**
* @brief Function implementing the systemtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_System_Task */
void System_Task(void const * argument)
{
  /* USER CODE BEGIN System_Task */
	
  /* Infinite loop */
  for(;;)
  {
		if(robot.mode == INITIAL)
		{
			robot.shooter_id1_42mm.cooling_limit = 100;
			robot.shooter_id1_42mm.cooling_rate = 20;
			robot.shooter_id1_42mm.speed_limit = 10;
		}
		else if(robot.mode == BURST)
		{
			if(robot.level == LV_1)
			{
				robot.shooter_id1_42mm.cooling_limit = 200;
				robot.shooter_id1_42mm.cooling_rate = 40;
				robot.shooter_id1_42mm.speed_limit = 10;
			}else if(robot.level == LV_2)
			{
				robot.shooter_id1_42mm.cooling_limit = 350;
				robot.shooter_id1_42mm.cooling_rate = 80;
				robot.shooter_id1_42mm.speed_limit = 10;
			}else if(robot.level == LV_3)
			{
				robot.shooter_id1_42mm.cooling_limit = 500;
				robot.shooter_id1_42mm.cooling_rate = 120;
				robot.shooter_id1_42mm.speed_limit = 10;
			}
		}else if(robot.mode == RATE)
		{
			if(robot.level == LV_1)
			{
				robot.shooter_id1_42mm.cooling_limit = 100;
				robot.shooter_id1_42mm.cooling_rate = 20;
				robot.shooter_id1_42mm.speed_limit = 16;
			}else if(robot.level == LV_2)
			{
				robot.shooter_id1_42mm.cooling_limit = 200;
				robot.shooter_id1_42mm.cooling_rate = 60;
				robot.shooter_id1_42mm.speed_limit = 16;
			}else if(robot.level == LV_3)
			{
				robot.shooter_id1_42mm.cooling_limit = 300;
				robot.shooter_id1_42mm.cooling_rate = 100;
				robot.shooter_id1_42mm.speed_limit = 16;
			}
		}
		
		chassis_Ctrl();
		
    osDelay(2);
  }
  /* USER CODE END System_Task */
}



/* USER CODE BEGIN Header_Shoot_Task */

//------------------------------------------------------------------------------
/**
 * @brief 
 */
void Referee_Get_Data(uint32_t *StdId, uint8_t *rxBuf)
{
	if(*StdId != 0x302)
	{
		return;
	}
	uint8_t shooter_power= (rxBuf[0] & (1<<2)) >> 2;
	uint8_t mode_L= (rxBuf[0] & (1<<3)) >> 3, mode_H= (rxBuf[0] & (1<<4)) >> 4;
	uint8_t robot_LV_L =(rxBuf[0] & (1<<5)) >> 5,  robot_LV_H= (rxBuf[0] & (1<<6)) >> 6;
	uint8_t robot_id = (rxBuf[0] & (1<<7)) >> 7;
	robot.id = robot_id;//0红，1蓝

	if(robot_LV_H == 0 && robot_LV_L == 0)
	{
		robot.level = LV_1;
	}else if(robot_LV_H == 1 && robot_LV_L == 0)
	{
		robot.level = LV_2;
	}else if(robot_LV_H == 0 && robot_LV_L == 1)
	{
		robot.level = LV_3;
	}

	if(mode_L == 0 && mode_H == 0)
	{
		robot.mode = INITIAL;
	}else if(mode_L == 0 && mode_H == 1)
	{
		robot.mode = BURST;
	}
	else if(mode_L == 1 && mode_H == 0)
	{
		robot.mode = RATE;
	}

	robot.mains_power_shooter_output = shooter_power;
	robot.cooling_heat = (int16_t)rxBuf[1] << 8 | (int16_t)rxBuf[2];//42mm 枪口热量
	robot.bullet_speed = ((int16_t)rxBuf[3] << 8 | (int16_t)rxBuf[4])/100.f; //42mm枪口射速
	robot.shooter_id1_42mm.bullet_remaining_num_42mm=((int16_t)rxBuf[5] << 8 | (int16_t)rxBuf[6]);//42mm剩余发射数目
}

static void chassis_State_Handoff(Gimbal_Status_e state)
{
    if(state!=chassis.state){//防止循环调用出错
    //参数重置（受当前模式影响）
		chassis.state=state;
    }
}

static void chassis_Mode_Handoff(CHASSIS_MODE mode)
{
    if(mode!=chassis.mode){//防止循环调用出错
    //参数重置（受当前模式影响）
		chassis.mode=mode;
    }
}

static void chassis_Ctrl(void)
{
	  static uint8_t chassis_mode_L = 1,chassis_mode_R = 0;
		static uint8_t chassis_act_L = 0,chassis_act_R = 0;
		static uint8_t unlimited_signal=0;
    static uint8_t stuck_signal=0;

	
		if(remote_ctrl.rc.ch[4]==-660){chassis.ctrl=2;}//键盘
		if(remote_ctrl.rc.ch[4]==660){chassis.ctrl=1;}//遥控
		
		if(chassis.ctrl==1){chassis_mode_L=0;chassis_mode_R=1;}
		if(chassis.ctrl==2){chassis_mode_L=1;chassis_mode_R=0;}
		
		if(Key_SHIFT() == true)chassis.mode=SPIN;//小陀螺模式
//		else if(Hanging_flag ==1 )chassis.mode=INVA;//吊射模式，底盘锁死
		
		if(chassis.mode==act_err){chassis_act_L=0;chassis_act_R=0;}
		else if(chassis.mode==INVA){chassis_act_L=0;chassis_act_R=1;}
		else if(chassis.mode==FOLO){chassis_act_L=1;chassis_act_R=0;}
		else if(chassis.mode==SPIN){chassis_act_L=1;chassis_act_R=1;}

		if(Shoot_Info.mode == SHOOT_ON) head.shoot_ready = SHOOT_ON;
		else if(Shoot_Info.mode == SHOOT_OFF) head.shoot_ready = SHOOT_OFF;
		
		if(vision_flag == 1) head.auto_ready=1;
		else if(vision_flag == 0) head.auto_ready=0;
		
		if(remote_ctrl.key.set.CTRL  == 0) stuck_signal=0;
		else if(remote_ctrl.key.set.CTRL == 1) stuck_signal=1;
		
		if(Unlimited_flag==0) unlimited_signal=0;
		else if(Unlimited_flag==1) unlimited_signal=1;
		
    chassis.vx=remote_ctrl.rc.ch[2];
    chassis.vy=remote_ctrl.rc.ch[3];
    chassis.key=remote_ctrl.key.v;
//发送装载
		chassis.Data[0] = (uint8_t)(chassis_mode_L)<<7 | (uint8_t)(chassis_mode_R)<< 6 | (uint8_t)(chassis_act_L)<<5 | (uint8_t)chassis_act_R<<4
															| (uint8_t)(head.shoot_ready)<<3 | (uint8_t)(head.auto_ready) << 2 | (uint8_t)(stuck_signal)<<1;
		chassis.Data[1] = (uint8_t)unlimited_signal;
    chassis.Data[2] = (uint8_t)(chassis.vx>>8);
    chassis.Data[3] = (uint8_t)(chassis.vx);
    chassis.Data[4] = (uint8_t)(chassis.vy>>8);
    chassis.Data[5] = (uint8_t)(chassis.vy);
    chassis.Data[6] = (uint8_t)(chassis.key>>8);
    chassis.Data[7] = (uint8_t)(chassis.key);
		
		USER_CAN_TxMessage(CAN2,0x300,chassis.Data);
		
//		head.Data[4] = (uint8_t)((int)((pit_motor_angle+16.30f)*100) >> 8);
//		head.Data[5] = (uint8_t)((int)((pit_motor_angle+16.30f)*100));
//		USER_CAN_TxMessage(CAN2,0x1ff,head.Data);

		
}



//------------------------------------------------------------------------------




