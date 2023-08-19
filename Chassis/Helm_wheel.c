/*
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-19     xzq       the first version
 *
 * @note 三/四轮舵轮底盘运动学逆解
 *        1		1----4
 *       / \	|    |
 *      2---3	2----3
 *      轮序编号
 *      ^y
 *      |
 *      |
 *      |_______>x
 *      坐标系方向
 *  轮系0方向与y轴相同
 */
 
#include "Helm_wheel.h"
//#include "bsp_ops.h"

helm_chassis_t helm_chassis;

/**
 * @brief 舵轮底盘参数初始化
 * @param void
 * 
 */

void helm_chassis_init(control_type type)
{
	helm_chassis.chassis_control_type = type;
#ifdef THREE_WHEEL_CHASSIS
	helm_chassis.wheel1.VN2X = 180;
	helm_chassis.wheel1.angle_actually = 0;
	helm_chassis.wheel1.angle_target = 0;
	helm_chassis.wheel1.v_target = 0;
	
	helm_chassis.wheel2.VN2X = -60;
	helm_chassis.wheel2.angle_actually = 0;
	helm_chassis.wheel2.angle_target = 0;
	helm_chassis.wheel2.v_target = 0;
	
	helm_chassis.wheel3.VN2X = 60;
	helm_chassis.wheel3.angle_actually = 0;
	helm_chassis.wheel3.angle_target = 0;
	helm_chassis.wheel3.v_target = 0;
#endif
	
#ifdef FOUR_WHEEL_CHASSIS
	helm_chassis.wheel1.VN2X = -135;
	helm_chassis.wheel1.angle_actually = 0;
	helm_chassis.wheel1.angle_target = 0;
	helm_chassis.wheel1.v_target = 0;
	
	helm_chassis.wheel2.VN2X = -45;
	helm_chassis.wheel2.angle_actually = 0;
	helm_chassis.wheel2.angle_target = 0;
	helm_chassis.wheel2.v_target = 0;
	
	helm_chassis.wheel3.VN2X = 45;
	helm_chassis.wheel3.angle_actually = 0;
	helm_chassis.wheel3.angle_target = 0;
	helm_chassis.wheel3.v_target = 0;
	
	helm_chassis.wheel4.VN2X = 135;
	helm_chassis.wheel4.angle_actually = 0;
	helm_chassis.wheel4.angle_target = 0;
	helm_chassis.wheel4.v_target = 0;
#endif
}

/**
 * @brief 角度限制，将角度限制在（-180,180）
 * @param *angle 需要限制的角度
 */

void angle_limit(float *angle)
{
	if(*angle>180.0f)
	{
		*angle-=360.0f;
		angle_limit(angle);
	}
	else if(*angle<=-180.0f)
	{
		*angle+=360.0f;
		angle_limit(angle);
	}
}

/**
 * @brief 舵轮底盘控制
 * @param vx_input x方向期望速度
 *        vy_input y方向期望速度
 *        w_input  角速度期望速度（逆时针为正）
 *        *chassis_ptr 舵轮底盘对象
 */

void helm_chassis_control(float vx_input, float vy_input, float w_input, helm_chassis_t *chassis_ptr)
{
	static float vx;
	static float vy;
//	chassis_ptr->dir_chassis = -ops_data.val[0];
	switch(chassis_ptr->chassis_control_type)
	{
		case CHASSIS_COORDINATE:
			vx = vx_input;
			vy = vy_input;
			break;
		case GLOBAL_COORDINATE:
			vx = vx_input*cosf(ANGLE2RAD(chassis_ptr->dir_chassis)) - vy_input*sinf(ANGLE2RAD(chassis_ptr->dir_chassis));
			vy = vy_input*cosf(ANGLE2RAD(chassis_ptr->dir_chassis)) + vx_input*sinf(ANGLE2RAD(chassis_ptr->dir_chassis));
			break;
		case LOCK_CHASSIS:
			vx = 0;
			vy = 0;
			break;
	}
	
	if(fabsf(vx) <= 0.001f && fabsf(vy) <= 0.001f && fabsf(w_input) <= 0.1f)
	{
		chassis_ptr->wheel1.v_target = 0;
		chassis_ptr->wheel2.v_target = 0;
		chassis_ptr->wheel3.v_target = 0;
#ifdef FOUR_WHEEL_CHASSIS
		chassis_ptr->wheel4.v_target = 0;
#endif
	}
	else
	{
		helm_wheel_control(vx, vy, w_input, &chassis_ptr->wheel1);
		helm_wheel_control(vx, vy, w_input, &chassis_ptr->wheel2);
		helm_wheel_control(vx, vy, w_input, &chassis_ptr->wheel3);
#ifdef FOUR_WHEEL_CHASSIS
		helm_wheel_control(vx, vy, w_input, &chassis_ptr->wheel4);
#endif
	}
}

/**
 * @brief 舵轮轮系控制
 * @param vx x方向期望速度
 *        vy y方向期望速度
 *        w  角速度期望速度（逆时针为正）
 *        *wheel_ptr 轮系结构体指针
 */

void helm_wheel_control(float vx, float vy, float w, helm_wheel_t *wheel_ptr)
{
	float vx_wheel = 0;
	float vy_wheel = 0;
	float VN = 0;
	float v_target = 0;
	float diraction = 0;
	
	VN = ANGLE2RAD(w) * DIS_WHEEL2CENTER;
	
	vx_wheel = vx + VN*cosf(ANGLE2RAD(wheel_ptr->VN2X));
	vy_wheel = vy + VN*sinf(ANGLE2RAD(wheel_ptr->VN2X));
	
	v_target = sqrtf(vx_wheel*vx_wheel + vy_wheel*vy_wheel);
	wheel_ptr->v_target = v_target;
	diraction = atan2f(vy_wheel, vx_wheel);
	diraction = RAD2ANGLE(diraction) - 90.0f;
	angle_limit(&diraction);
	v_dir2wheel_angle(wheel_ptr, diraction);
}

/**
 * @brief 劣弧选择 如果是劣弧直接转 如果是优弧则反向转且轮电机换向
 * @param *wheel 轮系结构体指针
 * @param *dir 期望方向
 */
void v_dir2wheel_angle(helm_wheel_t *wheel, float dir)
{
	float err_angle;
	float now_angle = wheel->angle_actually;
	angle_limit(&now_angle);

	err_angle = dir - now_angle;
	angle_limit(&err_angle);//获得目标方向相对于当前轮系方向的角度
	if(fabsf(err_angle) > 90.0f) //取劣弧
	{
		wheel->v_target = -wheel->v_target;
		if(err_angle > 0)
		{
			err_angle = err_angle - 180.0f;
		}
		else
		{
			err_angle = err_angle + 180.0f;
		}
	}	

	wheel->angle_target = wheel->angle_actually + err_angle;
	wheel->angle_actually = wheel->angle_target;
}

