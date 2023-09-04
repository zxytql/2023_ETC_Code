#ifndef OD_CAN_COM_H
#define OD_CAN_COM_H

#include "stdint.h"
#include "can.h"

#define OD_AXIS1 1
#define OD_AXIS2 2
#define OD_AXIS3 3
#define OD_COM_DELAY_TIME 5

enum ControlMode {
		CONTROL_MODE_VOLTAGE_CONTROL     = 0,
		CONTROL_MODE_TORQUE_CONTROL      = 1,
		CONTROL_MODE_VELOCITY_CONTROL    = 2,
		CONTROL_MODE_POSITION_CONTROL    = 3,
};
enum InputMode {
		INPUT_MODE_INACTIVE              = 0,
		INPUT_MODE_PASSTHROUGH           = 1,
		INPUT_MODE_VEL_RAMP              = 2,
		INPUT_MODE_POS_FILTER            = 3,
		INPUT_MODE_MIX_CHANNELS          = 4,
		INPUT_MODE_TRAP_TRAJ             = 5,
		INPUT_MODE_TORQUE_RAMP           = 6,
		INPUT_MODE_MIRROR                = 7,
		INPUT_MODE_TUNING                = 8,
};

typedef union
{
	float val;
	char byte[4];
}OD_union_t;

uint8_t* Float_to_Byte(float f);
void OD_Send_CAN(CAN_HandleTypeDef *hcan, uint16_t address, uint8_t data[8]);
void OD_Axis_Set_CloseLoop(uint16_t od_axis);
void OD_Axis_Set_Idle(uint16_t od_axis);
void OD_Set_Ctrl_Mode(uint16_t od_axis,uint16_t ctrl_mode, uint16_t input_mode);
void OD_Set_Input_Vel(uint16_t od_axis, float vel);
void OD_Set_Input_Pos(uint16_t od_axis, float pos);
void OD_Set_Limits(uint16_t od_axis, float vel_limit, float current_limit);
void OD_Reboot(uint16_t od_axis);
void OD_Clear_Errors(uint16_t od_axis);

#endif
