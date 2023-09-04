/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Task_flow.c/h
  * @brief      工训小车任务流程
  * @note
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2022-08-19         zxy            First version
	* @verbatim
	* [V1.0.0]
	* 1. 为什么会有OD_START_POS? 因为SPI磁编码器是绝对式的，重新上电，零位不变，
	*    因此，设置朝正X方向（右手坐标系）为0位，作差即可得到相对零点。
	* 2. 舵机B口为爪子，C口为载物盘
	*
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  */
	
#include "Task_Flow.h"
#include "bsp_buzzer.h"
#include "bsp_hmi.h"
#include "usart.h"
#include "bsp_GM65.h"
#include "bsp_servo.h"
#include "OD_CAN_Com.h"
#include "bsp_usart.h"
#include "DJI_Motor_Ctrl.h"


#define OD_START_POS 		(0.4369)
#define BELT_MOTOR_ID   (4)

#define GROUND_HEIGHT 	(-8192*46)
#define CARRIER_HEIGHT 	(-8192*3)
#define VERTEX_HEIGHT 	(0)

#define CARRIER_1_ANGLE (-65)
#define CARRIER_2_ANGLE (15)
#define CARRIER_3_ANGLE (90)

#define PM_ANGLE_0			(OD_START_POS)
#define PM_ANGLE_1			(OD_START_POS - 0.46)
#define PM_ANGLE_2			(OD_START_POS - 0.46)
#define PM_ANGLE_3			(OD_START_POS - 0.465)

#define CLAW_ANGLE_OPEN 	(0)
#define CLAW_ANGLE_CATCH 	(50)
#define CLAW_ANGLE_SOPEN	(30)

/**** Global ****/
task_flow_t task_flow;
uint8_t temp_buff_start;
const uint8_t g_vision_change02_cmd[3] = {0xAA, 0x02, 0xAE};
const uint8_t g_vision_change01_cmd[3] = {0xAA, 0x01, 0xAE};

int g_zxy_servo_test1 = 0;
int g_zxy_servo_test2 = 0;
int g_zxy_3508_height = 0;
float g_OD_zxy99 = 0;

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
	Servo_Ctrl('C',0);

	/** 使能 MINI ODRIVE 回到相对零位 **/
	osDelay(3000);
	OD_Clear_Errors(OD_AXIS1);
	OD_Set_Ctrl_Mode(OD_AXIS1,CONTROL_MODE_POSITION_CONTROL,INPUT_MODE_TRAP_TRAJ);
	OD_Axis_Set_CloseLoop(OD_AXIS1);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	/** ------------------- **/
	
	task_flow.task_process = 0;
	BeginWarnBuzzer();
	
	task_flow.chassis_ctrl_mode = REMOTE_CTRL_MODE; //开机默认为遥控器模式
	HMI_Write_txt(HMI_TASK_CODE,666666);
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
		switch(task_flow.task_process)
		{
			case 1:
				Pick_Ground_1();
				task_flow.task_process = 99;
				break;
			
			case 2:
				Pick_Ground_2();
				task_flow.task_process = 99;
				//GM65_Scan();
				break;
			
			case 3:
				Pick_Ground_3();
				task_flow.task_process = 99;
				break;
			
			case 4:
				HMI_Write_txt(HMI_TASK_CODE,GM_65_num_1*1000+GM_65_num_2);
//				HAL_UART_Transmit_IT(&huart2,g_vision_change_cmd,sizeof(g_vision_change_cmd));
				task_flow.task_process++;
				break;
			
			case 5:
				break;
			
			case 6:
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
	
	if(ras_rx.frame[14] == 0x01)
	{
		HMI_Write_txt(HMI_VISION_STA,1);
	}
	else if(ras_rx.frame[14] == 0x02)
	{
		HMI_Write_txt(HMI_VISION_STA,2);
	}
	else
	{
		HMI_Write_txt(HMI_VISION_STA,0);
	}
}

//从地面上取第一个物料到载物盘上，然后回到初始位置
void Pick_Ground_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,GROUND_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

//从地面上取第二个物料到载物盘上，然后回到初始位置
void Pick_Ground_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,GROUND_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

//从地面上取第三个物料到载物盘上，然后回到初始位置
void Pick_Ground_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,GROUND_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}
