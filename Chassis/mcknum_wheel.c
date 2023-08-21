/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       mcknum_wheel.c/h
  * @version    V1.0.0
  * @brief      �����ķ�����ֵ����˶�ѧ�����
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2023-08-19         zxy            First version
  * @verbatim
	* @reference  [1].https://mp.weixin.qq.com/s/GWhlXsuY6QYyoZydaSYpjQ
  * =============================================================================
  *                                 ʹ��˵��
  * =============================================================================
  * 1. �˶�ѧ�������ο�������[1]��ʹ��ǰ��ȷ��A B�֡�����������һ��
	
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
	
	
#include "mcknum_wheel.h"
#include "DJI_Motor_Ctrl.h"

/**
 * @brief �����˶�ѧ��� �ɸ��������ٶȼ��������ת��
 * @reference [1].(18)
 * @param ptr: �����˶��ṹ��
 */
void Mcknum_Kinematic_Inverse(chassis_t *ptr)
{
	if (ptr == NULL) { return; }
	
	ptr->wheel_speed[0] = 1 / WHEEL_RADIUS * (ptr->chassis_ct.ct_vx - ptr->chassis_ct.ct_vy - ptr->chassis_ct.ct_wz * (LX + LY));
	ptr->wheel_speed[1] = 1 / WHEEL_RADIUS * (ptr->chassis_ct.ct_vx + ptr->chassis_ct.ct_vy - ptr->chassis_ct.ct_wz * (LX + LY));
	ptr->wheel_speed[2] = 1 / WHEEL_RADIUS * (ptr->chassis_ct.ct_vx - ptr->chassis_ct.ct_vy + ptr->chassis_ct.ct_wz * (LX + LY));
	ptr->wheel_speed[3] = 1 / WHEEL_RADIUS * (ptr->chassis_ct.ct_vx + ptr->chassis_ct.ct_vy + ptr->chassis_ct.ct_wz * (LX + LY));
}

/**
 * @brief �����˶�ѧ���� �������ת�ٵõ������ٶ�
 * @reference [1].(19)
 * @param ptr: �����˶��ṹ��
 */
void Mcknum_Kinematic_Positive(chassis_t *ptr)
{
	if (ptr == NULL) { return; }
	/** Motor->vel���ص���ת��ת�� ��Ҫ�������ٱȻ������ת�� **/
	for(int i = 0; i < 4; i++)
	{
		ptr->wheel_fdb_speed[i] = Motor[i].vel / 36;
	}
	float w1 = ptr->wheel_fdb_speed[0];
	float w2 = ptr->wheel_fdb_speed[1];
	float w3 = -ptr->wheel_fdb_speed[2];
	float w4 = -ptr->wheel_fdb_speed[3];
	
	ptr->chassis_ct_fdb.ct_fdb_vx = WHEEL_RADIUS / 4 * (w1 + w2 + w3 + w4);
	ptr->chassis_ct_fdb.ct_fdv_vy = WHEEL_RADIUS / 4 * (-w1 + w2 - w3 + w4);
	ptr->chassis_ct_fdb.ct_fdb_wz = WHEEL_RADIUS / 4 * (-_1_LXLY * w1 - _1_LXLY * w2 + _1_LXLY * w3 + _1_LXLY * w4); 
}

float Get_Now_X_Speed(void)
{
	return chassis.chassis_ct_fdb.ct_fdb_vx;
}

float Get_Now_Y_Speed(void)
{
	return chassis.chassis_ct_fdb.ct_fdv_vy;
}

float Get_Now_W_Speed(void)
{
	return chassis.chassis_ct_fdb.ct_fdb_wz;
}
