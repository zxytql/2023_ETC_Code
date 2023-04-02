/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       bsp_servo.c/h
  * @version    V1.0.0
  * @brief      舵机控制支持包
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2022-01-10       zxy            First version
  * @verbatim
  * =============================================================================
  *                                 A板 使用说明
  * =============================================================================
  * 1. PWM输出通道配置为TIM5->CH1.2.3.4，控制频率为50HZ
  * A -> TIM5_CH4
  * B -> TIM5_CH3
  * C -> TIM5_CH2
  * D -> TIM5_CH1
  * 控制舵机前需要开启对应通道PWM
	* =============================================================================
	*                                 C板 使用说明
	* =============================================================================
	* DBUS -------------------------- PWM
	*    7           6     5     4       3  2  1
	* TIM8_CH3      CH2   CH1 TIM1_CH4
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */

#include "bsp_servo.h"

/**
 * @brief 线性映射函数
 * @param val：要进行转换的值
 * @param sour_min/max: 原值取值范围
 * @param dst_min/max : 目标值取值范围
 * @return 映射值
 * @note   2022.2.20 参数类型如果更改为int，会出现映射不准确的情况
 */
int16_t Linear_Map(float val, float sour_min, float sour_max, float dst_min, float dst_max)
{
	float i = 0;
	i = (dst_max - dst_min) / (sour_max - sour_min);
	return ((val - sour_min) * i + dst_min);
}

/**
 * @brief 舵机初始化
 * @param NULL
 * @note  需要注意舵机所接端口通道是否包含在函数内
 */
void Servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);	
}

/**
 * @brief 舵机控制函数
 * @param ch: RM-A板上舵机所接端口名称
 * @param angle: 舵机角度
 * @note
 */
void Servo_Ctrl_A(char ch, uint8_t angle)
{
	int map_angle = Linear_Map(angle,0,180,50,250);
	if(ch == 'A')
	{
		TIM5->CCR4 = map_angle;
	}
	else if(ch == 'B')
	{
		TIM5->CCR3 = map_angle;
	}
	else if(ch == 'C')
	{
		TIM5->CCR2 = map_angle;
	}
	else if(ch == 'D')
	{
		TIM5->CCR1 = map_angle;
	}
	else
	{
		;
	}
}

/**
 * @brief 舵机控制函数
 * @param n: C板上舵机所接端口顺序 参考本文件开头说明
 * @param angle: 舵机角度
 * @note
 */
void Servo_Ctrl_C(int n, uint16_t angle)
{
	int map_angle = Linear_Map(angle,0,270,100,500);
	if(n == 7)
	{
		TIM8->CCR3 = map_angle;
	}
	else
	{
		;
	}
}
