/**
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  * @file       Navigation_Task.c/h
  * @brief      导航任务
  * @note
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2020-12-15         pfy            First version
	*  V1.1.0     2021-06-28         zxy               Migrate
	*  V1.2.0     2022-07-08				 zxy              2022RC-R1
	*  V2.0.0     2023-08-19         zxy              工训版本
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
	
	
	
#include "Navigation_task.h"
#include "Chassis_Task.h"
#include "mcknum_wheel.h"

PID_nav_t PID_nav;
vel_t nav_vel;
fzy_pid_t fzy_pid_x;
fzy_pid_t fzy_pid_y;
fzy_pid_t fzy_pid_v;
fzy_pid_t fzy_pid_w;

/* USER CODE BEGIN Header_Navigation_Task_Entry */
/**
* @brief Function implementing the Navigation_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation_Task_Entry */
void Navigation_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Navigation_Task_Entry */
	PID_Nav_Init(&PID_nav, &fzy_pid_x, &fzy_pid_y, &fzy_pid_w, &fzy_pid_v);
	PID_nav.x_set = 0;
	PID_nav.y_set = 0;
	PID_nav.yaw_set = 0;
  /* Infinite loop */
  for(;;)
  {
		Ops_Coordinate_Trans(&ops_data, &PID_nav);
		Mcknum_Kinematic_Positive(&chassis);
		
		nav_vel.vx = fuzzypid_cal(&fzy_pid_x,Get_Now_X_Speed() * 1000.0f,PID_nav.x_set, PID_nav.x_now); //单位都是mm
		nav_vel.vy = fuzzypid_cal(&fzy_pid_y,Get_Now_Y_Speed() * 1000.0f,PID_nav.y_set, PID_nav.y_now);
		nav_vel.ang_w = Navigation_Angle_Ctrl(PID_nav.yaw_now, PID_nav.yaw_set, 2.0f, 1.0f);
    osDelay(5);
  }
  /* USER CODE END Navigation_Task_Entry */
}

void Ops_Coordinate_Trans(ops_data_t *ops, PID_nav_t *PID_nav)
{
	PID_nav->x_now = -Ops_Get_Y();
	PID_nav->y_now = Ops_Get_X();
	PID_nav->yaw_now = Ops_Get_Yaw();
}

void PID_Nav_Init(PID_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_ptr_w, fzy_pid_t *fzy_ptr_v)
{
	//             ptr         e_down  e_up           ec_down  ec_up  kp_d kp_up               ki             kd       i_l                  out_limit 
	fuzzy_init(fzy_ptr_x,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 3000.0f / 1000.0f);
	fuzzy_init(fzy_ptr_w,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f, 18.0f, 1.0f,2.0f,  12.0f,15.0f, 10.0f, 50.0f); //航向角不需要转换倍率
	fuzzy_init(fzy_ptr_y,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 3000.0f / 1000.0f);    
	fuzzy_init(fzy_ptr_v,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 500.0f / 1000.0f);   
	//1000是将m -> mm的倍率

	pid_nav->nav_state = NAV_PATH_NOT_DONE;
	pid_nav->track_state = TRACK_POINT_NOT_DONE;

	pid_nav->err_cir_x = 0;
	pid_nav->err_cir_y = 0;
	pid_nav->path_index = 0;
	pid_nav->point_index = 0;
	
	pid_nav->vx_limit = 50.0f;
	pid_nav->vy_limit = 50.0f;	

	pid_nav->x_set = 0;
	pid_nav->y_set = 0;
	pid_nav->yaw_set = 0;
}

float Navigation_Angle_Ctrl(float anglePresent,float angleTarget,float kp,float kd)
{
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f,dTermFliter = 0.0f;
	//PD控制器
	//目标角度减去当前角度
	angleErr = (angleTarget - anglePresent);
	dTerm = (angleErr - lastAngleErr);
	//低通滤波
	dTermFliter = 0.5f*dTerm + 0.5f*lastAngledTerm;

	angularVel = angleErr * kp + dTermFliter * kd;
	
	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	
	angularVelErr = angularVel - chassis.chassis_ct_fdb.ct_fdb_wz;
	angularVel = angularVel + angularVelErr *0.2f;
	
	if(angularVel>10.0f)
	{
		angularVel = 10.0f;
	}
	else if(angularVel<-10.0f)
	{
		angularVel = -10.0f;
	}
	
	return angularVel;
}

