/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       DJI_Motor_Task.c/h
  * @brief      DJI电机控制任务
  * @note
	*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2022-01-08       zxy            First version
	*  V1.1.0     2023-04-02       zxy            移植到工训控制板上
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
	

#include "DJI_Motor_Ctrl_Task.h"

/* USER CODE BEGIN Header_DJI_Motor_Ctrl_Task_Entry */
/**
* @brief Function implementing the DJI_Motor_Ctrl_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_Motor_Ctrl_Task_Entry */
void DJI_Motor_Ctrl_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN DJI_Motor_Ctrl_Task_Entry */
	CanId_Init();
	DriverInit(0,M_2006,POSITION_CONTROL_MODE);
	DriverInit(1,M_2006,POSITION_CONTROL_MODE);
	DriverInit(2,M_2006,POSITION_CONTROL_MODE);

  /* Infinite loop */
  for(;;)
  {
		MotorCtrl();
    osDelay(1);
  }
  /* USER CODE END DJI_Motor_Ctrl_Task_Entry */
}
