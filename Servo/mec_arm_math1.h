#ifndef MEC_ARM_MATH_H_
#define MEC_ARM_MATH_H_

#include "math.h"
#include "cmsis_os.h"
	
#define MATH_PI 3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1 40
#define MATH_L2 120
#define MATH_L3 120
#define MATH_L4 12
#define MATH_L43 MATH_L4/MATH_L3

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          ('A')
#define SERVO_RIGHT_NUM         ('B')

typedef struct
{
	double y_a;
	double y_b;
	double y_c;
	double y_d;
	
	double z_a;
	double z_b;
	double z_c;
	double z_d;	
}interpolate_param_t;

void Coordinate_To_Angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3);
void Set_Current_Angle(double theta_1, double theta_2, double theta_3);
void Set_Current_XYZ(double x, double y, double z);
void Interpolate_YZ(interpolate_param_t *ptr, double y_cur, double z_cur, double y_tgt, double z_tgt, double time);
void Move_To(double x, double y, double z, double time);

#endif
