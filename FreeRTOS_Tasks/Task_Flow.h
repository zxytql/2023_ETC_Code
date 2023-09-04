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
	uint16_t task_process;
	uint16_t chassis_ctrl_mode;
	
}task_flow_t;

/******** Extern ********/
extern task_flow_t task_flow;

/**** Function ****/
void HMI_Update(void);
void Pick_Ground_1(void);
void Pick_Ground_2(void);
void Pick_Ground_3(void);

#endif
