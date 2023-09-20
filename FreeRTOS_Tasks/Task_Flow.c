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
#include "Navigation_Task.h"


#define OD_START_POS 		(0.4451) 
#define OD_READY_POS		(0.1951) //准备区的位置，防止超出边界
#define BELT_MOTOR_ID   (4)

#define PICK_GROUND_HEIGHT 			(-8192*48) 
#define PLACE_GROUND_HEIGHT 		(-8192*48) //47
#define PLACE_CARRIER_HEIGHT 		(-8192*6)   
#define PICK_CARRIER_HEIGHT			(-8192*10) //10 夹上端
#define TURNTABLE_HEIGHT 				(-8192*12)
#define VERTEX_HEIGHT 					(-8192)

#define CARRIER_1_ANGLE (-83)  //
#define CARRIER_2_ANGLE (-2)   //
#define CARRIER_3_ANGLE (75)   //

#define PM_ANGLE_0			(OD_START_POS)
#define PM_ANGLE_1			(OD_START_POS - 0.4701) //SUM = -0.025
#define PM_ANGLE_2			(OD_START_POS - 0.4701)
#define PM_ANGLE_3			(OD_START_POS - 0.4701)

#define PICK_PM_ANGLE_1			(OD_START_POS - 0.4701)
#define PICK_PM_ANGLE_2			(OD_START_POS - 0.4701)
#define PICK_PM_ANGLE_3			(OD_START_POS - 0.4701)

#define CLAW_ANGLE_OPEN 	(0) //0
#define CLAW_ANGLE_CATCH 	(55) //
#define CLAW_ANGLE_SOPEN	(35) //
#define CLAW_ANGLE_MOPEN	(35) //

/**** Global ****/
task_flow_t task_flow;
uint8_t temp_buff_start;
const uint8_t g_vision_change02_cmd[3] = {0xAA, 0x02, 0xAE};
const uint8_t g_vision_change01_cmd[3] = {0xAA, 0x01, 0xAE};

int place_order[6] = {0}; //物料放置顺序 在扫码完成后赋值

extern uint16_t START_FLAG; //按钮开始标志位 由动态任务Start_RockerKey_Task发送

int g_zxy_servo_test1 = 0;
int g_zxy_servo_test2 = 0;
int g_zxy_3508_height = 0;
float g_OD_zxy99 = OD_START_POS;
float g_ring_err_x = 0;
float g_ring_err_y = 0;
float g_ring_err_yaw = 0;

/** PD纠偏标志变量 **/
int8_t g_pd_yaw_flag = 0;
int8_t g_pd_x_flag = 0;
int8_t g_pd_y_flag = 0;

/** PD纠偏基准位置变量 **/
float g_pd_base_x = 0;
float g_pd_base_y = 0;
float g_pd_base_yaw = 0;

/**
 * @brief 粗加工区位置初始化
 * @param NULL
 */
void Roughing_Area_Point_Init(void)
{
	task_flow.roughing_area_pos.red_x = 1940.0f;
	task_flow.roughing_area_pos.red_y = 889.6f;
	task_flow.roughing_area_pos.red_yaw = 0.0f;
	
	task_flow.roughing_area_pos.green_x = 1940.0f;
	task_flow.roughing_area_pos.green_y = task_flow.roughing_area_pos.red_y + 150.0f;
	task_flow.roughing_area_pos.green_yaw = 0.0f;

	task_flow.roughing_area_pos.blue_x = 1940.0f;
	task_flow.roughing_area_pos.blue_y = task_flow.roughing_area_pos.red_y + 300.0f;
	task_flow.roughing_area_pos.blue_yaw = 0.0f;	
}

/**
 * @brief 精加工区位置初始化
 * @param NULL
 */
void Finishing_Area_Point_Init(void)
{
	task_flow.finishing_area_pos.red_x = 1240.0f;
	task_flow.finishing_area_pos.red_y = 1800.0f;
	task_flow.finishing_area_pos.red_yaw = 90.0f;
	
	task_flow.finishing_area_pos.green_x = task_flow.finishing_area_pos.red_x - 150.0f;
	task_flow.finishing_area_pos.green_y = 1800.0f;
	task_flow.finishing_area_pos.green_yaw = 90.0f;

	task_flow.finishing_area_pos.blue_x = task_flow.finishing_area_pos.red_x - 300.0f;
	task_flow.finishing_area_pos.blue_y = 1800.0f;
	task_flow.finishing_area_pos.blue_yaw = 90.0f;	
}

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
	Roughing_Area_Point_Init();
	Finishing_Area_Point_Init();
	
	/** 使能 MINI ODRIVE 回到相对零位 **/
	osDelay(3000);
	OD_Clear_Errors(OD_AXIS1);
	OD_Set_Ctrl_Mode(OD_AXIS1,CONTROL_MODE_POSITION_CONTROL,INPUT_MODE_TRAP_TRAJ);
	OD_Axis_Set_CloseLoop(OD_AXIS1);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_READY_POS); 
	/** ------------------- **/
	Servo_Ctrl('C',15);
	Servo_Ctrl('B',50);
	
	task_flow.task_process = 0;
	BeginWarnBuzzer();
	
	task_flow.chassis_ctrl_mode = REMOTE_CTRL_MODE; //开机默认为遥控器模式
	HMI_Write_txt(HMI_TASK_CODE,666666);
	
	PID_nav.err_cir_x = 2.0f; //给定导航误差圆精度为2mm
	PID_nav.err_cir_y = 2.0f;
	
	
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
		/**--------- 测试区 -不需要请注释-----**/
		Servo_Ctrl('B',g_zxy_servo_test1);
		Servo_Ctrl('C',g_zxy_servo_test2);
		SetPos(BELT_MOTOR_ID,g_zxy_3508_height);
		OD_Set_Input_Pos(OD_AXIS1,g_OD_zxy99);
		/*------------------------------------*/
		
		if(START_FLAG == 1 && task_flow.task_process == 0)
		{
			START_FLAG = 99;
			task_flow.task_process = 1;
		}
		
		switch(task_flow.task_process)
		{
			case 1: //开机 去扫码
				task_flow.chassis_ctrl_mode = NAVIGATION_MODE;
				PID_nav.x_set = 570;
				PID_nav.y_set = 150;
				GM65_Scan();
				break;
			
			case 2: //显示扫到的任务码
				HMI_Write_txt(HMI_TASK_CODE,GM_65_num_1*1000+GM_65_num_2);
			
				//记录放置顺序 1红 2绿 3蓝
				place_order[0] = GM_65_num_1 / 100;
				place_order[1] = GM_65_num_1 % 100 / 10;
				place_order[2] = GM_65_num_1 % 10;
				place_order[3] = GM_65_num_2 / 100;
				place_order[4] = GM_65_num_2 % 100 / 10;
				place_order[5] = GM_65_num_2 % 10;		
			
				task_flow.task_process++;
				break;
			
			case 3: //现在空，原本是扫描到后 往内侧走 防止旋转时进入禁区
				
				task_flow.task_process++;
			
				break;
			
			case 4: //前往转盘
				PID_nav.x_set = 1440;
				PID_nav.y_set = 150;
				PID_nav.yaw_set = -90.0f;
				
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 10.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 10.0f)
				{
					OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
					task_flow.task_process = 10;
				}
				break;
			
			case 5: 
				
				break;
			
			
			case 6:
				
				break;
			
			case 10: //取完 前往转角
				PID_nav.x_set = 1850;
				PID_nav.y_set = 130;
				PID_nav.yaw_set = 0.0f;
				OD_Set_Input_Pos(OD_AXIS1,OD_READY_POS); //收回爪子 减小转动惯量
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 20.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 20.0f)
				{
					task_flow.task_process++;
				}
				 
				break;
			
			case 11:  //向树莓派发送**色环识别**命令
				OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
				HAL_UART_Transmit_IT(&huart2,g_vision_change02_cmd,sizeof(g_vision_change02_cmd));
				Place_Roughing_Area_Point_Handler(); //前往粗加工区 粗定位
				task_flow.task_process++;
				break;
			
			case 12: 
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}				
				break;
			
			case 13: //进入视觉纠偏
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 14: //放第一个物料
				Place_Ground_Handler();
			
				Place_Roughing_Area_Point_Handler(); //设定下一个物料坐标
				task_flow.task_process++;
				break;
			
			case 15: //到达第二个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}					
				break;
			
			case 16: //进入第二次视觉纠偏
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 17: //放第二个物料
				Place_Ground_Handler();
			
				Place_Roughing_Area_Point_Handler(); //设定下一个物料坐标
				task_flow.task_process++;		
				break;
			
			case 18: //到达第三个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 3.0f
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 3.0f)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}	
				break;

			case 19: //进入第三次视觉纠偏
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 20: //放第三个物料
				Place_Ground_Handler();
				task_flow.task_process++;		
				break;

			case 21: //设定放置的第一个物料坐标
				osDelay(2000);
				Pick_Roughing_Area_Point_Handler(); 
				task_flow.task_process++;
				break;
			
			case 22: //到达第一个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 3.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 3.0f)
				{
					task_flow.task_process++;
				}				
				break;
				
			case 23: //取第一个地面物料
				Pick_Ground_Handler();
				task_flow.task_process++;	
				break;
			
			case 24: //设定放置的第二个物料坐标
				Pick_Roughing_Area_Point_Handler(); 
				task_flow.task_process++;
				break;
			
			case 25: //到达第二个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 3.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 3.0f)
				{
					task_flow.task_process++;
				}				
				break;
				
			case 26: //取第二个地面物料
				Pick_Ground_Handler();
				task_flow.task_process++;	
				break;
			
			case 27: //设定放置的第三个物料坐标
				Pick_Roughing_Area_Point_Handler(); 
				task_flow.task_process++;				
				break;

			case 28: //到达第三个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 3.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 3.0f)
				{
					task_flow.task_process++;
				}				
				break;
				
			case 29: //取第三个地面物料
				Pick_Ground_Handler();
				task_flow.task_process++;	
				break;
			
			case 30: //前往转角 去精加工区
				PID_nav.x_set = 1850.0f;
				PID_nav.y_set = 1700.0f;
				PID_nav.yaw_set = 90.0f;		
				OD_Set_Input_Pos(OD_AXIS1,OD_READY_POS); //收回爪子 减小转动惯量
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 20.0f 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 20.0f)
				{
					
					task_flow.task_process++;
				}			
				break;
			
			case 31: //发送色环识别命令
				OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
				HAL_UART_Transmit_IT(&huart2,g_vision_change02_cmd,sizeof(g_vision_change02_cmd));
				Place_Finishing_Area_Point_Handler(); //前往精加工区 粗定位
				task_flow.task_process++;				
				break;
			
			case 32:
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					g_pd_yaw_flag = 0; //进入yaw校正
					task_flow.task_process++;
				}				
				break;
			
			case 33: //进入视觉纠偏 
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);			
				break;
			
			case 34: //放第一个物料
				Place_Ground_Handler();
			
				Place_Finishing_Area_Point_Handler(); //设定下一个物料坐标
				task_flow.task_process++;
				break;
			
			case 35: //到达第二个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < PID_nav.err_cir_x 
					&& fabs(PID_nav.y_now - PID_nav.y_set) < PID_nav.err_cir_y)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}					
				break;
			
			case 36: //进入第二次视觉纠偏
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 37: //放第二个物料
				Place_Ground_Handler();
			
				Place_Finishing_Area_Point_Handler(); //设定下一个物料坐标
				task_flow.task_process++;		
				break;
			
			case 38: //到达第三个物料坐标
				if(fabs(PID_nav.x_now - PID_nav.x_set) < 3.0f
					&& fabs(PID_nav.y_now - PID_nav.y_set) < 3.0f)
				{
					//设定视觉偏航角纠偏基准值
					g_pd_base_yaw = PID_nav.yaw_now;
					
					task_flow.task_process++;
				}	
				break;

			case 39: //进入第三次视觉纠偏
				Vision_Ring_Correction(g_ring_err_x,g_ring_err_y,g_ring_err_yaw);
				break;
			
			case 40: //放第三个物料
				Place_Ground_Handler();
				task_flow.task_process++;		
				break;
			
			case 41:
				break;
			
			case 65:
				GM65_Scan();
				break;
			
			case 66:
				//GM65扫到码后会进入65+1的进程
				break;
			
			case 71: //测试 放1号
				Place_Ground_1();
				task_flow.task_process = 99;
				break;

			case 72: //测试 放2号
				Place_Ground_2();
				task_flow.task_process = 99;
				break;
			
			case 73: //测试 放3号
				Place_Ground_3();
				task_flow.task_process = 99;
				break;	
			
			case 81: //测试 取地面到1号
				Pick_Ground_1();
				task_flow.task_process = 99;
				break;

			case 82: //测试 取地面到2号
				Pick_Ground_2();
				task_flow.task_process = 99;
				break;

			case 83: //测试 取地面到3号
				Pick_Ground_3();
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

/**
 * @brief 串口屏刷新函数
 * @param NULL
 */
void HMI_Update(void)
{
	static int i = 0;
	g_ring_err_x = -ras_pos.val[2];
	g_ring_err_y = -ras_pos.val[1];
	g_ring_err_yaw = ras_pos.val[0];
	
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


/**
 * @brief 放粗加工区物料导航点选择函数
 * @param NULL
 */
void Place_Roughing_Area_Point_Handler(void)
{
	static int8_t place_cnt = 0;
	
	switch(place_order[place_cnt++])
	{
		case 1:
			PID_nav.x_set = task_flow.roughing_area_pos.red_x;
			PID_nav.y_set = task_flow.roughing_area_pos.red_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.red_yaw;
			break;
		
		case 2:
			PID_nav.x_set = task_flow.roughing_area_pos.green_x;
			PID_nav.y_set = task_flow.roughing_area_pos.green_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.green_yaw;		
			break;
		
		case 3:
			PID_nav.x_set = task_flow.roughing_area_pos.blue_x;
			PID_nav.y_set = task_flow.roughing_area_pos.blue_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.blue_yaw;
			break;
		
		default:
			break;
	}
}

/**
 * @brief 取粗加工区物料导航点选择函数
 * @param NULL
 */
void Pick_Roughing_Area_Point_Handler(void)
{
	static int8_t pick_cnt = 0;
	
	switch(place_order[pick_cnt++])
	{
		case 1:
			PID_nav.x_set = task_flow.roughing_area_pos.red_x;
			PID_nav.y_set = task_flow.roughing_area_pos.red_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.red_yaw;
			break;
		
		case 2:
			PID_nav.x_set = task_flow.roughing_area_pos.green_x;
			PID_nav.y_set = task_flow.roughing_area_pos.green_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.green_yaw;		
			break;
		
		case 3:
			PID_nav.x_set = task_flow.roughing_area_pos.blue_x;
			PID_nav.y_set = task_flow.roughing_area_pos.blue_y;
			PID_nav.yaw_set = task_flow.roughing_area_pos.blue_yaw;
			break;
		
		default:
			break;
	}
}

/**
 * @brief 放精加工区物料导航点选择函数
 * @param NULL
 */
void Place_Finishing_Area_Point_Handler(void)
{
	static int8_t place_cnt = 0;
	
	switch(place_order[place_cnt++])
	{
		case 1:
			PID_nav.x_set = task_flow.finishing_area_pos.red_x;
			PID_nav.y_set = task_flow.finishing_area_pos.red_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.red_yaw;
			break;
		
		case 2:
			PID_nav.x_set = task_flow.finishing_area_pos.green_x;
			PID_nav.y_set = task_flow.finishing_area_pos.green_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.green_yaw;		
			break;
		
		case 3:
			PID_nav.x_set = task_flow.finishing_area_pos.blue_x;
			PID_nav.y_set = task_flow.finishing_area_pos.blue_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.blue_yaw;
			break;
		
		default:
			break;
	}	
}

/**
 * @brief 取精加工区物料导航点选择函数
 * @param NULL
 */
void Pick_Finishing_Area_Point_Handler(void)
{
	static int8_t pick_cnt = 0;
	
	switch(place_order[pick_cnt++])
	{
		case 1:
			PID_nav.x_set = task_flow.finishing_area_pos.red_x;
			PID_nav.y_set = task_flow.finishing_area_pos.red_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.red_yaw;
			break;
		
		case 2:
			PID_nav.x_set = task_flow.finishing_area_pos.green_x;
			PID_nav.y_set = task_flow.finishing_area_pos.green_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.green_yaw;		
			break;
		
		case 3:
			PID_nav.x_set = task_flow.finishing_area_pos.blue_x;
			PID_nav.y_set = task_flow.finishing_area_pos.blue_y;
			PID_nav.yaw_set = task_flow.finishing_area_pos.blue_yaw;
			break;
		
		default:
			break;
	}	
}
//
//**************************************************
//              **取地面物料**到载物盘上
//**************************************************
//
void Pick_Ground_Handler(void)
{
	static int8_t pick_cnt = 0;
	
	switch(place_order[pick_cnt++])
	{
		case 1:
			Pick_Ground_1();
			break;
		
		case 2:
			Pick_Ground_2();
			break;
		
		case 3:
			Pick_Ground_3();
			break;
		
		default:
			break;
	}
	
}
void Pick_Ground_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(700);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}


void Pick_Ground_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(700);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}

void Pick_Ground_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,PICK_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(1500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(700);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
}



//
//**************************************************
//              **放小车物料**到地上
//**************************************************
//
void Place_Ground_Handler(void)
{
	static int8_t place_cnt = 0; 		//0代表放任务码的第一个物料
	
	switch(place_order[place_cnt++])
	{
		case 1:
			Place_Ground_1(); //1为红
			//记录当前坐标到粗加工区坐标结构体
			task_flow.roughing_area_pos.red_x = PID_nav.x_now;
			task_flow.roughing_area_pos.red_y = PID_nav.y_now;
			task_flow.roughing_area_pos.red_yaw = PID_nav.yaw_now;
			break;
		
		case 2:
			Place_Ground_2(); //2为绿
			//记录当前坐标到粗加工区坐标结构体
			task_flow.roughing_area_pos.green_x = PID_nav.x_now;
			task_flow.roughing_area_pos.green_y = PID_nav.y_now;
			task_flow.roughing_area_pos.green_yaw = PID_nav.yaw_now;
			break;
		
		case 3:
			Place_Ground_3(); //3为蓝
			//记录当前坐标到粗加工区坐标结构体
			task_flow.roughing_area_pos.blue_x = PID_nav.x_now;
			task_flow.roughing_area_pos.blue_y = PID_nav.y_now;
			task_flow.roughing_area_pos.blue_yaw = PID_nav.yaw_now;
			break;
		
		default:
			break;
	}
	
	if(place_cnt == 1) //第一次放置后 根据第一次放下的位置 刷新剩下两个位置的坐标
	{
		switch(place_order[0]) //读第一个放的是什么颜色的
		{
			case 1: //如果放的是红色
				task_flow.roughing_area_pos.green_x = task_flow.roughing_area_pos.red_x;
				task_flow.roughing_area_pos.green_y = task_flow.roughing_area_pos.red_y + 150.0f;
				task_flow.roughing_area_pos.green_yaw = task_flow.roughing_area_pos.red_yaw;		
				task_flow.roughing_area_pos.blue_x = task_flow.roughing_area_pos.red_x;;
				task_flow.roughing_area_pos.blue_y = task_flow.roughing_area_pos.red_y + 300.0f;
				task_flow.roughing_area_pos.blue_yaw = task_flow.roughing_area_pos.red_yaw;			
				break;
			
			case 2: //如果放的是绿色
				task_flow.roughing_area_pos.red_x = task_flow.roughing_area_pos.green_x;
				task_flow.roughing_area_pos.red_y = task_flow.roughing_area_pos.green_y - 150.0f;
				task_flow.roughing_area_pos.red_yaw = task_flow.roughing_area_pos.green_yaw;		
				task_flow.roughing_area_pos.blue_x = task_flow.roughing_area_pos.green_x;;
				task_flow.roughing_area_pos.blue_y = task_flow.roughing_area_pos.green_y + 150.0f;
				task_flow.roughing_area_pos.blue_yaw = task_flow.roughing_area_pos.green_yaw;			
				break;				
			
			case 3: //如果放的是蓝色
				task_flow.roughing_area_pos.red_x = task_flow.roughing_area_pos.blue_x;
				task_flow.roughing_area_pos.red_y = task_flow.roughing_area_pos.blue_y - 300.0f;
				task_flow.roughing_area_pos.red_yaw = task_flow.roughing_area_pos.blue_yaw;		
				task_flow.roughing_area_pos.green_x = task_flow.roughing_area_pos.blue_x;;
				task_flow.roughing_area_pos.green_y = task_flow.roughing_area_pos.blue_y - 150.0f;
				task_flow.roughing_area_pos.green_yaw = task_flow.roughing_area_pos.blue_yaw;			
				break;

			default:
				break;
		}
	}
}
void Place_Ground_1(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	Servo_Ctrl('C',CARRIER_1_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(700);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
}

void Place_Ground_2(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	Servo_Ctrl('C',CARRIER_2_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_2);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(700);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
}

void Place_Ground_3(void)
{
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	Servo_Ctrl('C',CARRIER_3_ANGLE);
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	
	OD_Set_Input_Pos(OD_AXIS1,PICK_PM_ANGLE_3);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PICK_CARRIER_HEIGHT);
	osDelay(1000);
	
	Servo_Ctrl('B',CLAW_ANGLE_CATCH);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
	
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_GROUND_HEIGHT);
	osDelay(1500);
	
	Servo_Ctrl('B',CLAW_ANGLE_OPEN);
	osDelay(700);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	osDelay(500);
}


//
//**************************************************
//              **取转盘物料**到载物盘上
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
	osDelay(1500);
	
	OD_Set_Input_Pos(OD_AXIS1,PM_ANGLE_1);
	osDelay(1000);
	
	SetPos(BELT_MOTOR_ID,PLACE_CARRIER_HEIGHT);
	osDelay(500);
	
	Servo_Ctrl('B',CLAW_ANGLE_SOPEN);
	osDelay(500);
	
	SetPos(BELT_MOTOR_ID,VERTEX_HEIGHT);
	OD_Set_Input_Pos(OD_AXIS1,OD_START_POS);
	osDelay(500);
}

//从转盘上取第一个物料到载物盘上，然后回到初始位置
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
	osDelay(500);
}

//从转盘上取第一个物料到载物盘上，然后回到初始位置
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
	osDelay(500);
}

/**
 * @brief 视觉识别纠正用PID函数
 * @param input:    	误差输入
 * @param output:    	PID输出
 * @param kp, kd:    	PID计算的kp, kd
 */
float Vision_PID_Cal(float val_target, float val_present, float kp, float ki, float kd)
{ 
	float output = 0.0f, err = 0.0f;
	static float last_err = 0.0f, err_sum = 0.0f;//, lastAngledTerm = 0.0f;
	//float dTerm = 0.0f, dTermFliter = 0.0f;
	
	//当前值减去目标值
	//note 到底是当前值减目标值 还是目标值减当前值 取决于output的作用方向
	err = val_present - val_target;
	//dTerm = err - last_err;
	
	//低通滤波
	//dTermFliter = 0.5f * dTerm + 0.5f*lastAngledTerm;
	
	
	err_sum += err;
	err_sum > 50 ? err_sum = 50 : err_sum;
	
	//output = kp * err + ki * err_sum + dTermFliter * kd;
	output = kp * err + ki * err_sum + (err - last_err) * kd;
	last_err = err;
	//lastAngledTerm  = dTerm;
	
	output > 50 ? output = 50 : output;
	return output;
}

/**
 * @brief 视觉识别色环纠正函数
 * @param err_x:    	当前x偏差
 * @param err_y:    	当前y偏差
 * @param err_yaw:    当前yaw偏差
 */
void Vision_Ring_Correction(int vision_x, int vision_y, float vision_yaw)
{
	const static int desired_x = 11; //OD_START_POS取0.4406
	const static int desired_y = -11;
	const static float desired_yaw = 0; 
	const static int allowed_err = 2; //允许误差区间2，即X可取[9,13]，Y[-9,-11]
	const static float allowed_yaw_err = 1.1f;
	
	int err_x = vision_x - desired_x;
	int err_y = vision_y - desired_y;
	float err_yaw = vision_yaw - desired_yaw;

	static int8_t yaw_correct_cnt = 0;
	static int8_t y_correct_cnt = 0;
	static int8_t x_correct_cnt = 0;
	
	static int last_err_x = 0;
	static int last_err_y = 0;
	static int last_err_yaw = 0;
	static uint32_t start_watchdog_time = 0; //开始卡死时间

	static int8_t yaw_correct_done_callback = 0;
	static int8_t y_correct_done_callback = 0;
	static int8_t x_correct_done_callback = 0;
	
	if(g_pd_yaw_flag == 1 && g_pd_y_flag == 1 && g_pd_x_flag == 1)
	{
		//三轴都纠正完毕 标志位置零 跳出纠正流程
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
	
	//第一次在粗加工区 纠正YAW 后续不纠正
	if((fabs(err_yaw) > allowed_yaw_err) && g_pd_yaw_flag == 0) 
	{
		PID_nav.yaw_set = g_pd_base_yaw - Vision_PID_Cal(desired_yaw, vision_yaw, 0.5f, 0, 0.1f);
		
		if(err_yaw != last_err_yaw) //误差在刷新时 记录tick 没在刷新（即卡死） 就不会更新tick
		{
			start_watchdog_time = HAL_GetTick();
		}
		
		if(HAL_GetTick() - start_watchdog_time >= 500) //卡死超过0.5秒 重新设定基准值
		{
			g_pd_base_yaw = PID_nav.yaw_now;
		}

		yaw_correct_cnt++;
	}
	else //这部分应只执行一次
	{
		if(yaw_correct_done_callback == 0)
		{
			yaw_correct_done_callback = 1;
			PID_nav.yaw_set = PID_nav.yaw_now;
			g_pd_yaw_flag = 1; //yaw纠正完毕
			
			//设定Y轴纠偏基准值
			g_pd_base_y = PID_nav.y_now;
		}
	}
	
	//YAW纠正完毕再开始纠正 Y轴和X轴
	//为什么先纠正Y轴 因为Y轴的移动并不稳定 常导致X轴变化
	if(g_pd_yaw_flag == 1)
	{
		if((abs(err_y) > allowed_err) && g_pd_y_flag == 0)
		{
			PID_nav.y_set = g_pd_base_y + Vision_PID_Cal(desired_y, vision_y, 0.5f, 0, 0.2f);
			
			if(err_y != last_err_y) //误差在刷新时 记录tick 没在刷新（即卡死） 就不会更新tick
			{
				start_watchdog_time = HAL_GetTick();
			}
			
			if(HAL_GetTick() - start_watchdog_time >= 500) //卡死超过0.5秒 重新设定基准值
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
				g_pd_y_flag = 1; //y纠正完毕
				//设定X轴纠偏基准值
				g_pd_base_x = PID_nav.x_now;
			}
		}
	}
	
	if(g_pd_yaw_flag == 1 && g_pd_y_flag == 1)
	{
		if((abs(err_x) > allowed_err) && g_pd_x_flag == 0)
		{
			PID_nav.x_set = g_pd_base_x + Vision_PID_Cal(desired_x, vision_x, 0.5f, 0, 0.2f);
			
			if(err_x != last_err_x) //误差在刷新时 记录tick 没在刷新（即卡死） 就不会更新tick
			{
				start_watchdog_time = HAL_GetTick();
			}
			
			if(HAL_GetTick() - start_watchdog_time >= 500) //卡死超过0.5秒 重新设定基准值
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
				g_pd_x_flag = 1; //yaw纠正完毕
			}
		}
	}
	
	//刷新上次的误差值
	last_err_x = err_x;
	last_err_y = err_y;
	last_err_yaw = err_yaw;
}
