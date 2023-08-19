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
//������ת��Ϊ�Ƕ���
#define RAD2ANGLE(x) (x/pi*180.0f)
//��ϵ���ĵ��������ĵľ���
#define DIS_WHEEL2CENTER 0.09269f
//���Ӱ뾶
#define HELM_WHEEL_RADIUS 0.02825f
//�����ֵ�����ٱ�
#define VEL_REDUCTION_RATIO 1.0f
//���������
#define MotorRatio 21.0f
// ʵ���ٶ�ת�������ת��
//#define Speed2Rpm(x) (x*60.0f*VEL_REDUCTION_RATIO*MotorRatio/(pi*HELM_WHEEL_RADIUS*2)) //VESC
#define Speed2Rpm(x) (x*60.0f*VEL_REDUCTION_RATIO/(pi*HELM_WHEEL_RADIUS*2)) //Mini_Odrive

//��ϵ�ṹ��
typedef struct
{
	float angle_actually; //��ϵ���ʵ���ۻ��Ƕ�
	float angle_target;   //��ϵ��������Ƕ�
	float VN2X;           //��ϵ���ٶ���X��ļн� -180��~180��

	float v_target;       //��ϵ�����ٶ�
}helm_wheel_t;

//���̿��Ʒ�ʽö��
typedef enum
{
	GLOBAL_COORDINATE,	//����ȫ������ϵ����
	CHASSIS_COORDINATE,	//���ڻ����˵�������ϵ����
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

