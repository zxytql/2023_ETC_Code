#ifndef _BSP_SERVO_H_
#define _BSP_SERVO_H_

#include "tim.h"

void Servo_Init(void);
void Servo_Ctrl_A(char ch, uint8_t angle);
void Servo_Ctrl_C(int n, uint16_t angle);
#endif
