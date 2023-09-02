/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Task_flow.c/h
  * @brief      工训小车任务流程
  * @note
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2022-08-19         zxy            First version
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
	
	
#include "Task_Flow.h"
#include "bsp_buzzer.h"
#include "bsp_hmi.h"
#include "tjc_usart_hmi.h"
#include "usart.h"
#include "bsp_GM65.h"
#include "bsp_servo.h"
#include "OD_CAN_Com.h"

/**** Global ****/
task_flow_t task_flow;
uint8_t temp_buff_start;
int g_zxy_servo_test1 = 0;
int g_zxy_servo_test2 = 0;

/* USER CODE BEGIN Header_Task_Flow_Entry */
/**
* @brief Function implementing the Task_Flow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Flow_Entry */
void Task_Flow_Entry(void const * argument)
{
	Servo_Init();
	HAL_UART_Receive_IT(&huart7,(uint8_t *)&temp_buff_start,1);
	
	/** 使能 MINI ODRIVE **/
	osDelay(3000);
	OD_Clear_Errors(OD_AXIS1);
	OD_Set_Ctrl_Mode(OD_AXIS1,CONTROL_MODE_POSITION_CONTROL,INPUT_MODE_TRAP_TRAJ);
	OD_Axis_Set_CloseLoop(OD_AXIS1);
	/** ------------------- **/
	
	task_flow.task_process = 0;
	BeginWarnBuzzer();
	
	task_flow.chassis_ctrl_mode = REMOTE_CTRL_MODE;
	HMI_Write_txt(HMI_TASK_CODE,666666);
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
		Servo_Ctrl('B',g_zxy_servo_test1);
		Servo_Ctrl('C',g_zxy_servo_test2);
		switch(task_flow.task_process)
		{
			case 0:
				break;
			
			case 1:
				GM65_Scan();
				break;
			
			case 2:
				HMI_Write_txt(HMI_TASK_CODE,GM_65_num_1*1000+GM_65_num_2);
				task_flow.task_process++;
				break;
			
			case 3:
				
				break;
			
			case 4:
				task_flow.chassis_ctrl_mode = NAVIGATION_MODE;
				break;
			
			default:
				break;
		}
		HMI_Update();
    osDelay(50);
  }
  /* USER CODE END Task_Flow_Entry */
}

void HMI_Update(void)
{
	static int i = 0;
	if(i == 0 && Ops_Get_X() != 0) //开始无ops数据时不发送ops数据
	{
		i = 1;
	}
	else
	{
		HMI_Write_txt(HMI_POS_X,PID_nav.x_now);
		HMI_Write_txt(HMI_POS_Y,PID_nav.y_now);
		HMI_Write_txt(HMI_POS_YAW,PID_nav.yaw_now);
		HMI_Write_txt(HMI_TASK_PCS,task_flow.task_process);
	}
}
