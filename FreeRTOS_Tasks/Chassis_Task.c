/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Chassis_Task.c/h
  * @brief      �����˶���������
  * @note
  * @history
  *  Version        Date            Author          Modification
  *  V1.0.0     2023-04-09          zxy            First version
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "OD_CAN_Com.h"
#include "simple_filter.h"
#include "RC_Task.h"
#include "DJI_Motor_Ctrl.h"
#include "mcknum_wheel.h"
#include "task_flow.h"


/**** Global Variables ****/
float debug_vx = 0;
float debug_vy = 0;
float debug_w  = 0;
chassis_t chassis;
float g_zxy1 = 0;

/******** Acc Filters ********/
first_order_filter_type_t rc_vx_slow_fliter; //ң��vx���ٶ�ƽ��
first_order_filter_type_t rc_vy_slow_fliter; //ң��vy���ٶ�ƽ��
first_order_filter_type_t rc_wz_slow_fliter; //ң��wz���ٶ�ƽ��
const static float chassis_vx_order_filter[1] = { 0.2f };
const static float chassis_vy_order_filter[1] = { 0.2f };
//const static float chassis_wz_order_filter[1] = { 0.003f };

/****************************************************************/

/* USER CODE BEGIN Header_Chassis_Task_Entry */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task_Entry */
void Chassis_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task_Entry */
	  //һ���˲���������ʼ�����
  first_order_filter_init(&rc_vx_slow_fliter, 0.01f, chassis_vx_order_filter);
  first_order_filter_init(&rc_vy_slow_fliter, 0.01f, chassis_vy_order_filter);

  /* Infinite loop */
  for(;;)
  {
		if(task_flow.chassis_ctrl_mode == REMOTE_CTRL_MODE)
		{
		//ң������ֵӳ�䵽�����ٶ�
			debug_vx = -rc.rc_KeyValue.right_vir_roll/200.0f;
			debug_vy = -rc.rc_KeyValue.right_horz_roll/200.0f;
			debug_w = -rc.rc_KeyValue.left_horz_roll/28.0f;
		
			first_order_filter_cali(&rc_vx_slow_fliter, debug_vx);
			first_order_filter_cali(&rc_vy_slow_fliter, debug_vy);

			chassis.chassis_ct.ct_vx = rc_vx_slow_fliter.out;
			chassis.chassis_ct.ct_vy = rc_vy_slow_fliter.out;
			chassis.chassis_ct.ct_wz = debug_w;
		}
		else if(task_flow.chassis_ctrl_mode == NAVIGATION_MODE)
		{
			chassis.chassis_ct.ct_vx = nav_vel.vx;
			chassis.chassis_ct.ct_vy = nav_vel.vy;
			chassis.chassis_ct.ct_wz = nav_vel.ang_w;
		}
		else if(task_flow.chassis_ctrl_mode == START_UP_MODE)
		{
			// START_UP_MODE��Ϊ�˷�ֹ�ڿ���ʱ˲�����ң�������ݵ��µ��̵���鶯
			// ��2S��, task_flow.c�л��Զ��л���REMOTE_CTRL_MODE����������
			chassis.chassis_ct.ct_vx = 0;
			chassis.chassis_ct.ct_vy = 0;
			chassis.chassis_ct.ct_wz = 0;			
		}
		/** �������� **/
		if(chassis.chassis_ct.ct_vx < 0.01f && chassis.chassis_ct.ct_vx > -0.01f)
		{
			chassis.chassis_ct.ct_vx = 0;
		}
		if(chassis.chassis_ct.ct_vy < 0.01f && chassis.chassis_ct.ct_vy > -0.01f)
		{
			chassis.chassis_ct.ct_vy = 0;
		}
		
		Global_Cor_Trans(&chassis); //תȫ������ϵ����
		Mcknum_Kinematic_Inverse(&chassis); //�˶�ѧ���
		
		//����ķ������˶�ѧ�޹أ�ֻ�Ƿ�ת���ת��
		SetSpeed(0,chassis.wheel_speed[0]);
		SetSpeed(1,chassis.wheel_speed[1]);
		SetSpeed(2,-chassis.wheel_speed[2]);
		SetSpeed(3,-chassis.wheel_speed[3]);
		
    osDelay(5);
  }
  /* USER CODE END Chassis_Task_Entry */
}

void Global_Cor_Trans(chassis_t *ptr)
{
	float vx;
	float vy;
	float vx_input = ptr->chassis_ct.ct_vx;
	float vy_input = ptr->chassis_ct.ct_vy;
	
	ptr->chassis_ct.ct_yaw = Ops_Get_Yaw();
	vx = vx_input*cosf(ANGLE2RAD(-ptr->chassis_ct.ct_yaw)) - vy_input*sinf(ANGLE2RAD(-ptr->chassis_ct.ct_yaw));
	vy = vy_input*cosf(ANGLE2RAD(-ptr->chassis_ct.ct_yaw)) + vx_input*sinf(ANGLE2RAD(-ptr->chassis_ct.ct_yaw));
	
	ptr->chassis_ct.ct_vx = vx;
	ptr->chassis_ct.ct_vy = vy;
}
