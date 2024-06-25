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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#define IS_STANDARD
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "bsp_can.h"
#include "Gimbal_Task.h"
#include "remote_control.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 
 * @note  
 */
typedef enum{
    INITIAL=0,
    BURST,//爆发
    RATE,//弹速
    ROBOT_MODE_NUM,
}ROBOT_MODE;

typedef enum{
    LV_1,
    LV_2,
    LV_3,
    ROBOT_LEVEL_NUM,
}ROBOT_LEVEL;

typedef  struct
{
	uint16_t	cooling_rate;		//枪口每秒冷却值
	uint16_t	cooling_limit;  //枪口热量上限
	uint16_t	speed_limit;    //枪口上限速度 单位 m/s
	uint16_t  bullet_remaining_num_42mm;//42mm弹丸剩余发射数目
}ROBOT_SHOOTER_T;



typedef  struct 
{
		uint8_t id; //机器人Id 0红1蓝
    ROBOT_LEVEL level; //等级
		ROBOT_MODE mode; //发射机构类型
		uint8_t mains_power_shooter_output;//主控输出情况，0 无输出 1 24V
		uint16_t cooling_heat;   //枪口热量
		float bullet_speed;//实时射速
		ROBOT_SHOOTER_T shooter_id1_42mm;		//发射机构信息
		float Chas_Gyro;
}ROBOT;

extern ROBOT robot;

/**
 * @brief 
 */
typedef enum
{
		act_err,
    INVA,//卸力
    FOLO,//跟随云台     follow
    SPIN,//小陀螺       spin
}CHASSIS_MODE;

typedef struct
{
    uint8_t state;

    CHASSIS_MODE mode;
		
		uint8_t ctrl;

		uint8_t Data[8];
    
    int16_t vx,vy,key;

    Remote_Info_Typedef *dr16;
    
    void (*state_Setup)(Gimbal_Status_e);
		void (*mode_Setup)(CHASSIS_MODE);
}CHASSIS;

typedef struct
{
	uint8_t shoot_ready;
	uint8_t auto_ready;
	uint8_t supcap_ready;
	uint8_t Data[8];
	
}HEAD;

extern CHASSIS chassis;
extern HEAD head;
extern bool SUPCAP_FLAG;

/* Exported functions prototypes ---------------------------------------------*/
void Referee_Get_Data(uint32_t *StdId, uint8_t *rxBuf);
static void chassis_Ctrl(void);
static void chassis_Mode_Handoff(CHASSIS_MODE mode);
static void chassis_State_Handoff(Gimbal_Status_e state);


#endif //SYSTEM_TASK_H

