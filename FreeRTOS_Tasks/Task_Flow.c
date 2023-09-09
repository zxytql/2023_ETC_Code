/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       Task_flow.c/h
  * @brief      ��ѵС����������
  * @note
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2022-08-19         zxy            First version
	* @verbatim
	* [V1.0.0]
	* 1. Ϊʲô����OD_START_POS? ��ΪSPI�ű������Ǿ���ʽ�ģ������ϵ磬��λ���䣬
	*    ��ˣ����ó���X������������ϵ��Ϊ0λ������ɵõ������㡣
	* 2. ���B��Ϊצ�ӣ�C��Ϊ������
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
#include "Navigation_Task.h"


#define OD_START_POS 		(0.4406) //0.4369
#define OD_READY_POS		(0.187) //׼������λ�ã���ֹ�����߽�
#define BELT_MOTOR_ID   (4)

#define PICK_GROUND_HEIGHT 			(-8192*46)
#define PLACE_GROUND_HEIGHT 		(-8192*44.5)
#define PLACE_CARRIER_HEIGHT 		(-8192*3)
#define PICK_CARRIER_HEIGHT			(-8192*6.5)
#define TURNTABLE_HEIGHT 				(-8192*12)
#define VERTEX_HEIGHT 					(0)

#define CARRIER_1_ANGLE (-95)  //-95
#define CARRIER_2_ANGLE (-17)   //-17
#define CARRIER_3_ANGLE (65)   //65

#define PM_ANGLE_0			(OD_START_POS)
#define PM_ANGLE_1			(OD_START_POS - 0.4737)
#define PM_ANGLE_2			(OD_START_POS - 0.4737)
#define PM_ANGLE_3			(OD_START_POS - 0.4737)

#define PICK_PM_ANGLE_1			(OD_START_POS - 0.4676)
#define PICK_PM_ANGLE_2			(OD_START_POS - 0.4676)
#define PICK_PM_ANGLE_3			(OD_START_POS - 0.4676)

#define CLAW_ANGLE_OPEN 	(-15)
#define CLAW_ANGLE_CATCH 	(35) 
#define CLAW_ANGLE_SOPEN	(-5) 
#define CLAW_ANGLE_MOPEN	(-10) 

/**** Global ****/
task_flow_t task_flow;
uint8_t temp_buff_start;
const uint8_t g_vision_change02_cmd[3] = {0xAA, 0x02, 0xAE};
const uint8_t g_vision_change01_cmd[3] = {0xAA, 0x01, 0xAE};

int place_order[6] = {0}; //���Ϸ���˳�� ��ɨ����ɺ�ֵ

int g_zxy_servo_test1 = 0;
int g_zxy_servo_test2 = 0;
int g_zxy_3508_height = 0;
float g_OD_zxy99 = 0;
float g_ring_err_x = 0;
float g_ring_err_y = 0;
float g_ring_err_yaw = 0;

/** PD��ƫ��־���� **/
int8_t g_pd_yaw_flag = 0;
int8_t g_pd_x_flag = 0;
int8_t g_pd_y_flag = 0;

/** PD��ƫ��׼λ�ñ��� **/
float g_pd_base_x = 0;
float g_pd_base_y = 0;
float g_pd_base_yaw = 0;

/* USER CODE BEGIN Header_Task_Flow_Entry */
/**
* @brief Function implementing the Task_Flow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Flow_Entry */
void Task_Flow_Entry(void const * argument)
{
	HAL_UART_Receive_IT(&huart7,(uint8_t *)&temp_buff_start,1);
	Servo_Init();

	/** ʹ�� MINI ODRIVE �ص������λ **/
	osDelay(3000);
	OD_Clear_Errors(OD_AXIS1);
	OD_Set_Ctrl_Mode(OD_AXIS1,CONTROL_MODE_POSITION_CONTROL,INPUT_MODE_TRAP_TRAJ);
	OD_Axis_Set_CloseLoop(OD_AXIS1);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_READY_POS);
	/** ------------------- **/
	Servo_Ctrl('C',0);
	Servo_Ctrl('B',0);
	
	task_flow.task_process = 0;
	BeginWarnBuzzer();
	
	task_flow.chassis_ctrl_mode = REMOTE_CTRL_MODE; //����Ĭ��Ϊң����ģʽ
	HMI_Write_txt(HMI_TASK_CODE,666666);
	
	PID_nav.err_cir_x = 1.0f; //�����������Բ����Ϊ1mm
	PID_nav.err_cir_y = 1.0f;
	
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
		/**--------- ������ -����Ҫ��ע��-----**/
//		Servo_Ctrl('B',g_zxy_servo_test1);
//		Servo_Ctrl('C',g_zxy_servo_test2);
//		SetPos(BELT_MOTOR_ID,g_zxy_3508_height);
//		OD_Set_Input_Pos(OD_AXIS1,g_OD_zxy99);
		/*------------------------------------*/
		
		switch(task_flow.task_process)
		{
			case 1: //���� ȥɨ��
				task_flow.chassis_ctrl_mode = NAVIGATION_MODE;
				PID_nav.x_set = 550;
				PID_nav.y_set = 220;
				GM65_Scan();
				break;
			
			case 2: //��ʾɨ����������
				HMI_Write_txt(HMI_TASK_CODE,GM_65_num_1*1000+GM_65_num_2);
			
				//��¼����˳�� 1�� 2�� 3��
				place_order[0] = GM_65_num_1 / 100;
				place_order[1] = GM_65_num_1 % 100 / 10;
				place_order[2] = GM_65_num_1 % 10;
				place_order[3] = GM_65_num_2 / 100;
				place_order[4] = GM_65_num_2 % 100 / 10;
				place_order[5] = GM_65_num_2 % 10;		
			
				task_flow.task_process++;
				break;
			
			case 3: //ɨ�赽�� ���ڲ��� ��ֹ��תʱ�������
				PID_nav.y_set = 120;
				if(PID_nav.y_now <= 125)
				{
					task_flow.task_process++;
				}
				break;
			
			case 4: //ǰ��ת��
				PID_nav.x_set = 1440;
				PID_nav.y_set = 200;
				PID_nav.yaw_set = -90.0f;
				OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
			
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					task_flow.task_process = 10;
				}
				break;
			
			case 5: 
				
				break;
			
			
			case 6:
				
				break;
			
			case 10: //ȡ�� ǰ��ת��
				PID_nav.x_set = 1850;
				PID_nav.y_set = 130;
				PID_nav.yaw_set = 0.0f;
				OD_Set_Input_Pos(OD_AXIS1,OD_READY_POS); //�ջ�צ�� ��Сת������
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					task_flow.task_process++;
				}
				 
				break;
			
			case 11:  //����ݮ�ɷ���**ɫ��ʶ��**����
				HAL_UART_Transmit_IT(&huart2,g_vision_change02_cmd,sizeof(g_vision_change02_cmd));
				OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
				task_flow.task_process++;
				break;
			
			case 12: //ǰ���ּӹ��� �ֶ�λ
				PID_nav.x_set = 1940.0f;
				PID_nav.y_set = 889.6f;
				PID_nav.yaw_set = 0.0f;
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//�趨�Ӿ�ƫ���Ǿ�ƫ��׼ֵ
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}				
				break;
			
			case 13: //�����Ӿ���ƫ
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 14: //�ŵ�һ������
				Place_Ground_Handler();
//				//�궨��ɫɫ��X Y 
//				task_flow.roughing_area_pos.roughing_area_green_x = PID_nav.x_now;
//				task_flow.roughing_area_pos.roughing_area_green_y = PID_nav.y_now;
//				task_flow.roughing_area_pos.roughing_area_green_yaw = PID_nav.yaw_now;
//				task_flow.task_process++;
			
				PID_nav.y_set = PID_nav.y_now + 150.0f; //�趨��һ����������
				task_flow.task_process++;
				break;
			
			case 15: 
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//�趨�Ӿ�ƫ���Ǿ�ƫ��׼ֵ
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}					
				break;
			
			case 16: //����ڶ����Ӿ���ƫ
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 17:
				Place_Ground_Handler();
			
				PID_nav.y_set = PID_nav.y_now + 150.0f; //�趨��һ����������
				task_flow.task_process++;		
				break;
			
			case 18:
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//�趨�Ӿ�ƫ���Ǿ�ƫ��׼ֵ
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}	
				break;

			case 19: //����������Ӿ���ƫ
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 20:
				Place_Ground_Handler();
				task_flow.task_process++;		
				break;

			case 21:
				break;
			
			case 65:
				GM65_Scan();
				break;
			
			case 66:
				//GM65ɨ���������65+1�Ľ���
				break;
			
			case 71:
				Place_Ground_1();
				task_flow.task_process = 99;
				break;
			
			case 91:
				HAL_UART_Transmit_IT(&huart2,g_vision_change01_cmd,sizeof(g_vision_change01_cmd));
				task_flow.task_process = 99;
				break;		
			
			case 92:
				HAL_UART_Transmit_IT(&huart2,g_vision_change02_cmd,sizeof(g_vision_change02_cmd));
				task_flow.task_process = 99;
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
	g_ring_err_x = -ras_pos.val[2];
	g_ring_err_y = -ras_pos.val[1];
	g_ring_err_yaw = ras_pos.val[0];
	
	if(i == 0 && Ops_Get_X() != 0) //��ʼ��ops����ʱ������ops����
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
		HMI_Write_txt(HMI_RAS_POS_ERR_X,g_ring_err_x);
		HMI_Write_txt(HMI_RAS_POS_ERR_Y,g_ring_err_y);
	}
	else if(ras_rx.frame[14] == 0x02)
	{
		HMI_Write_txt(HMI_VISION_STA,2);
		HMI_Write_txt(HMI_RAS_POS_ERR_X,g_ring_err_x);
		HMI_Write_txt(HMI_RAS_POS_ERR_Y,g_ring_err_y);
	}
	else
	{
		HMI_Write_txt(HMI_VISION_STA,0);
	}
	
	
}

//
//**************************************************
//              **ȡ��������**����������
//**************************************************
//
void Pick_Ground_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}


void Pick_Ground_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

void Pick_Ground_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}



//
//**************************************************
//              **��С������**������
//**************************************************
//
void Place_Ground_Handler(void)
{
	static int8_t place_cnt = 0; 		//0�����������ĵ�һ������
	
	switch(place_order[place_cnt++])
	{
		case 1:
			Place_Ground_1();
			break;
		
		case 2:
			Place_Ground_2();
			break;
		
		case 3:
			Place_Ground_3();
			break;
		
		default:
			break;
	}
}
void Place_Ground_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_MOPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(300);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
}

void Place_Ground_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_MOPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(300);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);	
}

void Place_Ground_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_MOPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(300);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1000);	
}


//
//**************************************************
//              **ȡת������**����������
//**************************************************
//
void Pick_Turntable_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,TURNTABLE_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

//��ת����ȡ��һ�����ϵ��������ϣ�Ȼ��ص���ʼλ��
void Pick_Turntable_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,TURNTABLE_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

//��ת����ȡ��һ�����ϵ��������ϣ�Ȼ��ص���ʼλ��
void Pick_Turntable_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,TURNTABLE_HEIGHT);
	osDelay(800);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

/**
 * @brief �Ӿ�ʶ�������PID����
 * @param input:    	�������
 * @param output:    	PID���
 * @param kp, kd:    	PID�����kp, kd
 */
float Vision_PID_Cal(float val_target, float val_present, float kp, float ki, float kd)
{ 
	float output = 0.0f, err = 0.0f;
	static float last_err = 0, err_sum = 0;
	
	//��ǰֵ��ȥĿ��ֵ
	//note �����ǵ�ǰֵ��Ŀ��ֵ ����Ŀ��ֵ����ǰֵ ȡ����output�����÷���
	err = val_present - val_target;
	err_sum += err;
	err_sum > 50 ? err_sum = 50 : err_sum;
	
	output = kp * err + ki * err_sum + (err - last_err) * kd;
	last_err = err;
	
	output > 50 ? output = 50 : output;
	return output;
}

/**
 * @brief �Ӿ�ʶ��ɫ����������
 * @param err_x:    	��ǰxƫ��
 * @param err_y:    	��ǰyƫ��
 * @param err_yaw:    ��ǰyawƫ��
 */
void Vision_Ring_Correction(int vision_x, int vision_y, float vision_yaw)
{
	const static int desired_x = 11; //OD_START_POSȡ0.4406
	const static int desired_y = -11;
	const static float desired_yaw = 0; 
	const static int allowed_err = 2; //�����������3����X��ȡ[8,14]��Y[-8,-14]
	const static float allowed_yaw_err = 1.0f;
	
	int err_x = vision_x - desired_x;
	int err_y = vision_y - desired_y;
	float err_yaw = vision_yaw - desired_yaw;

	static int8_t yaw_correct_cnt = 0;
	static int8_t y_correct_cnt = 0;
	static int8_t x_correct_cnt = 0;
	
	static int last_err_x = 0;
	static int last_err_y = 0;
	static int last_err_yaw = 0;
	static uint32_t start_watchdog_time = 0; //��ʼ����ʱ��

	static int8_t yaw_correct_done_callback = 0;
	static int8_t y_correct_done_callback = 0;
	static int8_t x_correct_done_callback = 0;
	
	if(g_pd_yaw_flag == 1 && g_pd_y_flag == 1 && g_pd_x_flag == 1)
	{
		//���ᶼ������� ��־λ���� ������������
//		g_pd_yaw_flag = 0;
		g_pd_y_flag = 0;
		g_pd_x_flag = 0;
		
		yaw_correct_cnt = 0;
		y_correct_cnt = 0;
		x_correct_cnt = 0;
		
		yaw_correct_done_callback = 0;
		y_correct_done_callback = 0;
		x_correct_done_callback = 0;
		
		task_flow.task_process++;
		return;
	}
	
	//��һ�� ����YAW ����������
	if((fabs(err_yaw) > allowed_yaw_err) && g_pd_yaw_flag == 0) 
	{
		PID_nav.yaw_set = g_pd_base_yaw - Vision_PID_Cal(desired_yaw, vision_yaw, 0.5f, 0, 0.1f);
		
		if(err_yaw != last_err_yaw) //�����ˢ��ʱ ��¼tick û��ˢ�£��������� �Ͳ������tick
		{
			start_watchdog_time = HAL_GetTick();
		}
		
		if(HAL_GetTick() - start_watchdog_time >= 2000) //��������2�� �����趨��׼ֵ
		{
			g_pd_base_yaw = PID_nav.yaw_now;
		}

		yaw_correct_cnt++;
	}
	else //�ⲿ��Ӧִֻ��һ��
	{
		if(yaw_correct_done_callback == 0)
		{
			yaw_correct_done_callback = 1;
			PID_nav.yaw_set = PID_nav.yaw_now;
			g_pd_yaw_flag = 1; //yaw�������
			
			//�趨Y���ƫ��׼ֵ
			g_pd_base_y = PID_nav.y_now;
		}
	}
	
	//YAW��������ٿ�ʼ���� Y���X��
	//Ϊʲô�Ⱦ���Y�� ��ΪY����ƶ������ȶ� ������X��仯
	if(g_pd_yaw_flag == 1)
	{
		if((abs(err_y) > allowed_err) && g_pd_y_flag == 0)
		{
			PID_nav.y_set = g_pd_base_y + Vision_PID_Cal(desired_y, vision_y, 1.0f, 0, 0.2f);
			
			if(err_y != last_err_y) //�����ˢ��ʱ ��¼tick û��ˢ�£��������� �Ͳ������tick
			{
				start_watchdog_time = HAL_GetTick();
			}
			
			if(HAL_GetTick() - start_watchdog_time >= 2000) //��������2�� �����趨��׼ֵ
			{
				g_pd_base_y = PID_nav.y_now;
			}
			y_correct_cnt++;
		}
		else
		{
			if(y_correct_done_callback == 0)
			{
				y_correct_done_callback = 1;
				PID_nav.y_set = PID_nav.y_now;
				g_pd_y_flag = 1; //y�������
				//�趨X���ƫ��׼ֵ
				g_pd_base_x = PID_nav.x_now;
			}
		}
	}
	
	if(g_pd_yaw_flag == 1 && g_pd_y_flag == 1)
	{
		if((abs(err_x) > allowed_err) && g_pd_x_flag == 0)
		{
			PID_nav.x_set = g_pd_base_x + Vision_PID_Cal(desired_x, vision_x, 1.0f, 0, 0.2f);
			
			if(err_x != last_err_x) //�����ˢ��ʱ ��¼tick û��ˢ�£��������� �Ͳ������tick
			{
				start_watchdog_time = HAL_GetTick();
			}
			
			if(HAL_GetTick() - start_watchdog_time >= 2000) //��������2�� �����趨��׼ֵ
			{
				g_pd_base_x = PID_nav.x_now;
			}			
			
			x_correct_cnt++;
		}
		else
		{
			if(x_correct_done_callback == 0)
			{
				x_correct_done_callback = 1;
				PID_nav.x_set = PID_nav.x_now;
				g_pd_x_flag = 1; //yaw�������
			}
		}
	}
	
	//ˢ���ϴε����ֵ
	last_err_x = err_x;
	last_err_y = err_y;
	last_err_yaw = err_yaw;
}
