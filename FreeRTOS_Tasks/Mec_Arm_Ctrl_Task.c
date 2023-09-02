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


int zxy1 = 40;
int zxy2 = 60;

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
	
	vTaskDelete(NULL); //删除自身，防止误启动
	
	//Servo_Init();
	//Servo_Ctrl(SERVO_LEFT_NUM,0);
//	Servo_Ctrl(SERVO_RIGHT_NUM,0);
	//Mec arm init position
	//Set_Current_XYZ(0,40,60);
	//osDelay(1000);

	//--------------------
	
  /* Infinite loop */
  for(;;)
  {
		//Move_To(0,160,160,200);
		Move_To(0,135,140,200);
		Move_To(0,100,120,200);
		Move_To(0,90,70,200);
		Move_To(0,130,50,200);
		Move_To(0,165,30,200);
		Move_To(0,180,30,200);
		
		Move_To(0,165,30,200);
		Move_To(0,130,50,200);
		Move_To(0,90,70,200);
		Move_To(0,100,120,200);
		//Move_To(0,135,140,200);
		osDelay(10);
  }
  /* USER CODE END Mec_Arm_Ctrl_Task_Entry */
}
