/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Mec_Arm_Ctrl_Task.c/h
  * @brief      机械臂控制任务
  * @note
	*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-04-05       zxy            First version
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
	
#include "Mec_Arm_Ctrl_Task.h"
#include "bsp_servo.h"


int zxy1 = 0;
int zxy2 = 0;

/* USER CODE BEGIN Header_Mec_Arm_Ctrl_Task_Entry */
/**
* @brief Function implementing the Mec_Arm_Ctrl_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mec_Arm_Ctrl_Task_Entry */
void Mec_Arm_Ctrl_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Mec_Arm_Ctrl_Task_Entry */
	Servo_Init();
	
	//Mec arm init position
	Set_Current_XYZ(0,40,10);
	osDelay(1000);
	//--------------------
	
  /* Infinite loop */
  for(;;)
  {
		Set_Current_XYZ(0,150,150);
		osDelay(500);
		Set_Current_XYZ(0,150,0);
    osDelay(500);
  }
  /* USER CODE END Mec_Arm_Ctrl_Task_Entry */
}
