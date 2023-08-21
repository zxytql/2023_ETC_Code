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

/**** Global ****/
task_flow_t task_flow;

/* USER CODE BEGIN Header_Task_Flow_Entry */
/**
* @brief Function implementing the Task_Flow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Flow_Entry */
void Task_Flow_Entry(void const * argument)
{
	//HMI_Write_txt(HMI_START_CODE,0);
	task_flow.task_process = 0;
	task_flow.chassis_ctrl_mode = START_UP_MODE;
	BeginWarnBuzzer();
	task_flow.chassis_ctrl_mode = REMOTE_CTRL_MODE;
	
	HMI_Write_txt(HMI_TASK_CODE,123123);
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
		switch(task_flow.task_process)
		{
			case 0:
				break;
			
			case 1:
				task_flow.chassis_ctrl_mode = NAVIGATION_MODE;
				break;
			
			case 2:
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
	}
}
