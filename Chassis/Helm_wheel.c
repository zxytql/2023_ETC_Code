/*
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-19     xzq       the first version
 *
 * @note ��/���ֶ��ֵ����˶�ѧ���
 *        1		1----4
 *       / \	|    |
 *      2---3	2----3
 *      ������
 *      ^y
 *      |
 *      |
 *      |_______>x
 *      ����ϵ����
 *  ��ϵ0������y����ͬ
 */
 
#include "Helm_wheel.h"
//#include "bsp_ops.h"

helm_chassis_t helm_chassis;

/**
 * @brief ���ֵ��̲�����ʼ��
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
 * @brief �Ƕ����ƣ����Ƕ������ڣ�-180,180��
 * @param *angle ��Ҫ���ƵĽǶ�
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
 * @brief ���ֵ��̿���
 * @param vx_input x���������ٶ�
 *        vy_input y���������ٶ�
 *        w_input  ���ٶ������ٶȣ���ʱ��Ϊ����
 *        *chassis_ptr ���ֵ��̶���
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
 * @brief ������ϵ����
 * @param vx x���������ٶ�
 *        vy y���������ٶ�
 *        w  ���ٶ������ٶȣ���ʱ��Ϊ����
 *        *wheel_ptr ��ϵ�ṹ��ָ��
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
 * @brief �ӻ�ѡ�� ������ӻ�ֱ��ת ������Ż�����ת���ֵ������
 * @param *wheel ��ϵ�ṹ��ָ��
 * @param *dir ��������
 */
void v_dir2wheel_angle(helm_wheel_t *wheel, float dir)
{
	float err_angle;
	float now_angle = wheel->angle_actually;
	angle_limit(&now_angle);

	err_angle = dir - now_angle;
	angle_limit(&err_angle);//���Ŀ�귽������ڵ�ǰ��ϵ����ĽǶ�
	if(fabsf(err_angle) > 90.0f) //ȡ�ӻ�
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

