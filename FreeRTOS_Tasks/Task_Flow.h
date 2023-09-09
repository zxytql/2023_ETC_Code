#ifndef TASK_FLOW_H
#define TASK_FLOW_H


#include "cmsis_os.h"
#include "Navigation_Task.h"
#include "Chassis_Task.h"

#define REMOTE_CTRL_MODE (1)
#define NAVIGATION_MODE (3)
#define START_UP_MODE (0)

typedef struct
{
	float roughing_area_green_x;
	float roughing_area_green_y;
	float roughing_area_green_yaw;

	float roughing_area_blue_x;
	float roughing_area_blue_y;
	float roughing_area_blue_yaw;

	float roughing_area_red_x;
	float roughing_area_red_y;
	float roughing_area_red_yaw;	
	
}roughing_area_pos_t;

typedef struct
{
	uint16_t task_process;
	uint16_t chassis_ctrl_mode;
	
	roughing_area_pos_t roughing_area_pos;
	
}task_flow_t;


/******** Extern ********/
extern task_flow_t task_flow;

/**** Function ****/
void HMI_Update(void);
void Pick_Ground_1(void);
void Pick_Ground_2(void);
void Pick_Ground_3(void);

void Pick_Turntable_1(void);
void Pick_Turntable_2(void);
void Pick_Turntable_3(void);

void Place_Ground_Handler(void);
void Place_Ground_1(void);
void Place_Ground_2(void);
void Place_Ground_3(void);

void Vision_Ring_Correction(int err_x, int err_y, float err_yaw);
float Vision_PID_Cal(float val_target, float val_present, float kp, float ki, float kd);

#endif
