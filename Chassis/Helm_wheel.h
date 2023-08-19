/*
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-19     xzq       the first version
 */
 
#ifndef __HELM_WHEEL_H
#define __HELM_WHEEL_H

#include "math.h"

#define THREE_WHEEL_CHASSIS
//#define FOUR_WHEEL_CHASSIS

#define pi 3.1415926535898f

#define ANGLE2RAD(x) (x/180.0f*pi)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/pi*180.0f)
//轮系中心到底盘中心的距离
#define DIS_WHEEL2CENTER 0.09269f
//轮子半径
#define HELM_WHEEL_RADIUS 0.02825f
//驱动轮电机减速比
#define VEL_REDUCTION_RATIO 1.0f
//电机极对数
#define MotorRatio 21.0f
// 实际速度转换到电机转速
//#define Speed2Rpm(x) (x*60.0f*VEL_REDUCTION_RATIO*MotorRatio/(pi*HELM_WHEEL_RADIUS*2)) //VESC
#define Speed2Rpm(x) (x*60.0f*VEL_REDUCTION_RATIO/(pi*HELM_WHEEL_RADIUS*2)) //Mini_Odrive

//轮系结构体
typedef struct
{
	float angle_actually; //轮系舵的实际累积角度
	float angle_target;   //轮系舵的期望角度
	float VN2X;           //轮系线速度与X轴的夹角 -180°~180°

	float v_target;       //轮系期望速度
}helm_wheel_t;

//底盘控制方式枚举
typedef enum
{
	GLOBAL_COORDINATE,	//基于全局坐标系控制
	CHASSIS_COORDINATE,	//基于机器人底盘坐标系控制
	LOCK_CHASSIS
}control_type;

typedef struct
{
	helm_wheel_t wheel1;
	helm_wheel_t wheel2;
	helm_wheel_t wheel3;
#ifdef FOUR_WHEEL_CHASSIS
	helm_wheel_t wheel4;
#endif
	float dir_chassis;
	control_type chassis_control_type;
}helm_chassis_t;

void helm_chassis_init(control_type);
void angle_limit(float *angle);
void helm_chassis_control(float vx_input, float vy_input, float w_input, helm_chassis_t *chassis_ptr);
void v_dir2wheel_angle(helm_wheel_t *wheel, float dir);
void helm_wheel_control(float vx, float vy, float w, helm_wheel_t *wheel_ptr);

extern helm_chassis_t helm_chassis;

#endif

