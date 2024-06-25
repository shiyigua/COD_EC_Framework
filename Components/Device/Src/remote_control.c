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

/* Includes ------------------------------------------------------------------*/
#include "remote_control.h"
#include "string.h"
#include "Gimbal_Task.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl;

KEYBOARD_INFO_T Key_Info;


/**
 * @brief remote control usart RxDMA MultiBuffer
 */
uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */ 
		remote_ctrl->online_cnt = 0xFA;
}
//------------------------------------------------------------------------------

/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
  /* Juege the device status */
  if(remote_ctrl->online_cnt <= 50 )
  {
    /* reset the online count */
    remote_ctrl->online_cnt = 0xFA;

    /* clear the data */
    memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));
  }
  else if(remote_ctrl->online_cnt > 0 )
  {
    /* online count decrements which reseted in received interrupt  */
    remote_ctrl->online_cnt--;
  }
}

/**
  * @brief  
  * @param  
  *         
  * @retval none
  */
static float Sliding_Filter(float t,float *slideFilter)
{
    float res = 0.f;

    for(int i = SF_LENGTH-1; i > 0; i-- )
    {
        slideFilter[i] = slideFilter[i-1];
    }
    slideFilter[0] = t;
    for(int i = 0; i < SF_LENGTH-1; i++)
    {
        res +=slideFilter[i];
    }
    return (res/SF_LENGTH);
}

//extKalman_t KF_Mouse_X_Speed,KF_Mouse_Y_Speed;
//static float sliding_mouse_x[SF_LENGTH]={0.f,},sliding_mouse_y[SF_LENGTH]={0.f};

float Mouse_X_Speed(void)
{
    float res=0.f;

    if(abs(MOUSE_X_MOVE_SPEED) > Xmax){
        res = 0;
    }else{
        res = remote_ctrl.mouse.x;
			//Sliding_Filter(KalmanFilter(&KF_Mouse_X_Speed,(float)MOUSE_X_MOVE_SPEED),sliding_mouse_x);
    } 
    return (float)res;
}

float Mouse_Y_Speed(void)
{
    float res=0.f;

    if(abs(MOUSE_Y_MOVE_SPEED) > Ymax){
        res = 0;
    }else{
			res = remote_ctrl.mouse.y;
//        res = Sliding_Filter(KalmanFilter(&KF_Mouse_Y_Speed,(float)MOUSE_Y_MOVE_SPEED),sliding_mouse_y);
    }
    return (float)res;
}

/**
  * @brief  
  * @param  
  *         
  * @retval none
  */
void FirstGetInto_KEY_PRESS(KEY_SET_INFO_T *key_set_Info)
{
  if(key_set_Info->prev_KEY_PRESS != key_set_Info->KEY_PRESS)
  {
    key_set_Info->state_cnt = 0;
    key_set_Info->prev_KEY_PRESS = key_set_Info->KEY_PRESS;
  }
}

void KEY_State_Judge(KEY_SET_INFO_T *key_set_Info , uint8_t KEY_PRESS , int change_tim ,int long_change_tim)
{
    key_set_Info->KEY_PRESS = KEY_PRESS;
    FirstGetInto_KEY_PRESS(key_set_Info);
    if(KEY_PRESS == KEY_UP)
    {
        if(key_set_Info->prev_State != UP)
        {
            key_set_Info->state_cnt++;
            if(key_set_Info->state_cnt >= change_tim + 1)
            {
                key_set_Info->State = UP;
                key_set_Info->prev_State = UP;
            }
            else if(key_set_Info->state_cnt >= change_tim)
            {
                key_set_Info->State = RELAX;
                key_set_Info->prev_State = RELAX;
            }
        }else
        {
            key_set_Info->state_cnt = 0;
        }
    }
    else if(KEY_PRESS == KEY_DOWN)
    {
        if(key_set_Info->prev_State != DOWN) 
        {
            key_set_Info->state_cnt++;
            if(key_set_Info->state_cnt >= long_change_tim)  
            {
                key_set_Info->State = DOWN;
                key_set_Info->prev_State = DOWN;
            }
            else if(key_set_Info->state_cnt >= change_tim + 1)
            {
                key_set_Info->State = SHORT_DOWN;
                key_set_Info->prev_State = SHORT_DOWN;
            }
            else if(key_set_Info->state_cnt >= change_tim)  
            {
            key_set_Info->State = PRESS;
            key_set_Info->prev_State = PRESS;
            }
        }else
        {
            key_set_Info->state_cnt = 0;
        }
    }
}

bool Key_SHIFT(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.SHIFT ,KEY_PRESSED_SHIFT, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.SHIFT.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_MOUSE_L(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.MOUSE_L ,MOUSE_PRESSED_LEFT, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_L);

	switch(Key_Info.MOUSE_L.State)
	{
	case UP:		res = true;
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_MOUSE_R(void)
{
    bool res = false;

    KEY_State_Judge(&Key_Info.MOUSE_R ,MOUSE_PRESSED_RIGH, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_R);

    switch(Key_Info.MOUSE_R.State)
    {
        case UP:
        break;
        case PRESS:	
        break;
        case SHORT_DOWN:
        break;
        case DOWN:    res = true;
        break;
        case RELAX:
            res = false;
        break;
    }
		return res;
}

bool Key_R(void)
{
	static bool res = false;

  KEY_State_Judge(&Key_Info.R ,KEY_PRESSED_R, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_R);
  switch(Key_Info.R.State)
  {
    case UP:
				res=true;
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
    break;
    case DOWN:
    break;
    case RELAX:
				res=false;
    break;
  }
	return res;
}

void Key_Q(void)
{
	static bool res = true;
	KEY_State_Judge(&Key_Info.Q ,KEY_PRESSED_Q, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_Q);
	switch(Key_Info.Q.State)
	{
	case UP:
	break;
	case PRESS:
        res = true;
	break;
	case SHORT_DOWN:
        if(res == true)
        {
						Gimbal_Info.target.yaw_Angle += 90.f;
            res = false;
        }
	break;
	case DOWN:
	break;
	case RELAX:
	break;
	}
}

void Key_E(void)
{
    static bool res = true;

    KEY_State_Judge(&Key_Info.E ,KEY_PRESSED_E, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_E);

    switch(Key_Info.E.State)
    {
    case UP:
    break;
    case PRESS:
        res = true;
    break;
    case SHORT_DOWN:
        if(res == true)
        {
            Gimbal_Info.target.yaw_Angle -= 90.f;
            res = false;
        }
    break;
    case DOWN:
    break;
    case RELAX:
    break;
    }
}

bool Key_B(void)
{
	static bool res = false;
	static bool flag = true;
  KEY_State_Judge(&Key_Info.B ,KEY_PRESSED_B, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_B);
  switch(Key_Info.B.State)
  {
    case UP:
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
			if(flag == true)
			{
				if(res == false)
				{
					res = true;
				}else if(res == true)
				{
					res = false;
				}
				flag = false;
			}
    break;
    case DOWN:
    break;
    case RELAX:
			flag = true;
    break;
  }
	return res;
}

bool Key_Z(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.Z ,KEY_PRESSED_Z, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_Z);

	switch(Key_Info.Z.State)
	{
	case UP:res = false;		
	break;
	case PRESS: 
		
	break;
	case SHORT_DOWN:res = true;
	break;
	case DOWN:
	break;
	case RELAX: 
		
	break;
	}
	return res;
}

bool Key_X(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.X ,KEY_PRESSED_X, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_X);

	switch(Key_Info.X.State)
	{
	case UP:res = false;
	break;
	case PRESS: 
	break;
	case SHORT_DOWN:res = true;
	break;
	case DOWN:
	break;
	case RELAX: 
	break;
	}
	return res;
}

bool Key_C(void)
{
	static bool res = true;
  KEY_State_Judge(&Key_Info.C ,KEY_PRESSED_C, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_C);
  switch(Key_Info.C.State)
  {
    case UP:
			res=true;
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
			res = false;
    break;
    case DOWN:
    break;
    case RELAX:

    break;
  }
	return res;
}

bool Key_V(void)
{
	static bool res = true;
  KEY_State_Judge(&Key_Info.V ,KEY_PRESSED_V, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_V);
  switch(Key_Info.V.State)
  {
    case UP:			res=true;
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
			res = false;
    break;
    case DOWN:
    break;
    case RELAX:

    break;
  }
	return res;
}

bool Key_F(void)
{
	static bool res = false;
	static bool flag = true;
  KEY_State_Judge(&Key_Info.F ,KEY_PRESSED_F, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_F);
  switch(Key_Info.F.State)
  {
    case UP:
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
			if(flag == true)
			{
				if(res == false)
				{
					res = true;
				}else if(res == true)
				{
					res = false;
				}
				flag = false;
			}
    break;
    case DOWN:
    break;
    case RELAX:
			flag = true;
    break;
  }
	return res;
}

bool Key_G(void)
{
	static bool res = false;
	static bool flag = true;
	
  KEY_State_Judge(&Key_Info.G ,KEY_PRESSED_G, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_G);
  switch(Key_Info.G.State)
  {
    case UP:
    break;
    case PRESS:
    break;
    case SHORT_DOWN:
			if(flag == true)
			{
				if(res == false)
				{
					res = true;
				}else if(res == true)
				{
					res = false;
				}
				flag = false;
			}
    break;
    case DOWN:
    break;
    case RELAX:
			flag = true;
    break;
  }
	return res;
}

//------------------------------------------------------------------------------

