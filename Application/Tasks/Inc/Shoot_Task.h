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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "Gimbal_Task.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 
 */
typedef enum
{
	SHOOT_OFF,
  SHOOT_ON,
  SHOOT_MODE_NUM,
}Shoot_Mode_e;

/**
 * @brief 
 * @note  
 */
typedef struct 
{
  Shoot_Mode_e mode;
	
	struct
	{
		float trigger_Angle;
		float target_wheel_speed;
  }target;
	
	int16_t SendValue[3];

		
}Shoot_Info_Typedef;

extern Shoot_Info_Typedef Shoot_Info; 

/**
 * @brief 
 * @note  
 */


/**
 * @brief 
 */



/* Exported functions prototypes ---------------------------------------------*/
static float SpeedAdapt_16M(float real_S , float min_S, float max_S,float up_num , float down_num);

extern int Unlimited_flag;

#endif //SHOOT_TASK_H

