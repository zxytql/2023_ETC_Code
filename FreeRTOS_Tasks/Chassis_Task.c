/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Chassis_Task.c/h
  * @brief      底盘运动控制任务
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
#include "Helm_wheel.h"
#include "DJI_Motor_Ctrl.h"

/**** Global Variables ****/
float debug_vx = 0;
float debug_vy = 0;
float debug_w  = 0;
chassis_t chassis;

/******** Acc Filters ********/
first_order_filter_type_t rc_vx_slow_fliter; //遥控vx加速度平滑
first_order_filter_type_t rc_vy_slow_fliter; //遥控vy加速度平滑
first_order_filter_type_t rc_wz_slow_fliter; //遥控wz加速度平滑
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
	osDelay(4000);
	OD_Axis_Init();
	
	  //一阶滤波器参数初始化填充
  first_order_filter_init(&rc_vx_slow_fliter, 0.01f, chassis_vx_order_filter);
  first_order_filter_init(&rc_vy_slow_fliter, 0.01f, chassis_vy_order_filter);
//  first_order_filter_init(&rc_wz_slow_fliter, 0.01f, chassis_wz_order_filter);
	helm_chassis_init(CHASSIS_COORDINATE);
	
  /* Infinite loop */
  for(;;)
  {
		//键值映射到底盘速度
		debug_vx = rc.rc_KeyValue.left_vir_roll/300.0f;
		debug_vy = rc.rc_KeyValue.right_vir_roll/300.0f;
		debug_w = rc.rc_KeyValue.left_horz_roll/7.0f;
		
		first_order_filter_cali(&rc_vx_slow_fliter, debug_vx);
		first_order_filter_cali(&rc_vy_slow_fliter, debug_vy);
		
		chassis.chassis_ct.ct_vx = rc_vx_slow_fliter.out;
		chassis.chassis_ct.ct_vy = rc_vy_slow_fliter.out;
		
		helm_chassis_control(chassis.chassis_ct.ct_vx, chassis.chassis_ct.ct_vy, debug_w, &helm_chassis);
		
		OD_Set_Input_Vel(OD_AXIS1,Speed2Rpm(helm_chassis.wheel1.v_target)/60.0f);
		OD_Set_Input_Vel(OD_AXIS2,Speed2Rpm(helm_chassis.wheel2.v_target)/60.0f);
		OD_Set_Input_Vel(OD_AXIS3,Speed2Rpm(helm_chassis.wheel3.v_target)/60.0f);
//		
		SetPos(0,(helm_chassis.wheel1.angle_target/360)*2.5f*8192*36);
		SetPos(1,(helm_chassis.wheel2.angle_target/360)*2.5f*8192*36);
		SetPos(2,(helm_chassis.wheel3.angle_target/360)*2.5f*8192*36);
    osDelay(3);
  }
  /* USER CODE END Chassis_Task_Entry */
}

void OD_Axis_Init(void)
{
	OD_Clear_Errors(OD_AXIS1);
	OD_Set_Ctrl_Mode(OD_AXIS1,CONTROL_MODE_VELOCITY_CONTROL,INPUT_MODE_PASSTHROUGH);
	OD_Axis_Set_CloseLoop(OD_AXIS1);	
	
	OD_Clear_Errors(OD_AXIS2);
	OD_Set_Ctrl_Mode(OD_AXIS2,CONTROL_MODE_VELOCITY_CONTROL,INPUT_MODE_PASSTHROUGH);
	OD_Axis_Set_CloseLoop(OD_AXIS2);	
	
	OD_Clear_Errors(OD_AXIS3);
	OD_Set_Ctrl_Mode(OD_AXIS3,CONTROL_MODE_VELOCITY_CONTROL,INPUT_MODE_PASSTHROUGH);
	OD_Axis_Set_CloseLoop(OD_AXIS3);	
}
