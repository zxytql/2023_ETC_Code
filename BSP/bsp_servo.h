#ifndef _BSP_SERVO_H_
#define _BSP_SERVO_H_

#include "tim.h"

void Servo_Init(void);
void Servo_Ctrl(char ch, int angle);
#endif
