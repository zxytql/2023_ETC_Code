#ifndef MCKNUM_WHEEL_H
#define MCKNUM_WHEEL_H

#include "Chassis_Task.h"

#define WHEEL_RADIUS 0.03 //m
#define LX 0.105 
#define LY 0.1195
#define _1_LXLY (1 / (LX + LY))


/**** Function ****/
void Mcknum_Kinematic_Inverse(chassis_t *ptr);
void Mcknum_Kinematic_Positive(chassis_t *ptr);
float Get_Now_X_Speed(void);
float Get_Now_Y_Speed(void);
float Get_Now_W_Speed(void);

#endif
