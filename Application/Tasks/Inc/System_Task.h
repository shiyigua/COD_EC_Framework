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
    BURST,//����
    RATE,//����
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
	uint16_t	cooling_rate;		//ǹ��ÿ����ȴֵ
	uint16_t	cooling_limit;  //ǹ����������
	uint16_t	speed_limit;    //ǹ�������ٶ� ��λ m/s
	uint16_t  bullet_remaining_num_42mm;//42mm����ʣ�෢����Ŀ
}ROBOT_SHOOTER_T;



typedef  struct 
{
		uint8_t id; //������Id 0��1��
    ROBOT_LEVEL level; //�ȼ�
		ROBOT_MODE mode; //�����������
		uint8_t mains_power_shooter_output;//������������0 ����� 1 24V
		uint16_t cooling_heat;   //ǹ������
		float bullet_speed;//ʵʱ����
		ROBOT_SHOOTER_T shooter_id1_42mm;		//���������Ϣ
		float Chas_Gyro;
}ROBOT;

extern ROBOT robot;

/**
 * @brief 
 */
typedef enum
{
		act_err,
    INVA,//ж��
    FOLO,//������̨     follow
    SPIN,//С����       spin
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

