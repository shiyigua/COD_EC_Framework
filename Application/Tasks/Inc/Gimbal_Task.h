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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "remote_control.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the information for the GIMBAL status.
 */
typedef enum
{
  OFF,
  ON,
  AUTO_AIM,
  Hanging,
  Gimbal_Status_NUM,
}Gimbal_Status_e;

/**
 * @brief typedef structure that contains the information for the Gravity Compensation.
 * @note  output = k * (0 - pitchAngle) + offset.
 */
typedef struct
{
  float k;
  float offset;
  float output;
}Gravity_Compensation_Typedef;

/**
 * @brief typedef structure that contains the information for the Feed Forward.
 * @note  output = (input - inputlast) / dt + offset.
 */
typedef struct
{
  float input_last;
  float dt;
  float output;
  float offset;
}Feed_Forward_Typedef;

/**
 * @brief typedef structure that contains the information for the GIMBAL.
 */
typedef struct
{
  Gimbal_Status_e status;
  Gravity_Compensation_Typedef Gravity_Compensation ;
  Feed_Forward_Typedef Feed_Forward;

  struct{
    float pit_Angle;
    float yaw_Angle;
    float pit_hang_Angle;
    float yaw_hang_Angle;
  }target;
	
	Remote_Info_Typedef *dr16;
		
  int16_t SendValue[2];
  
}Gimbal_Info_Typedef;

extern Gimbal_Info_Typedef Gimbal_Info;
extern int vision_flag;
extern int Vision_Info_mode;
/* Exported functions prototypes ---------------------------------------------*/
extern float pit_motor_angle;

#endif //GIMBAL_TASK_H

