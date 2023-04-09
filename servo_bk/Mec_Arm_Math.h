#ifndef MEC_ARM_MATH_H_
#define MEC_ARM_MATH_H_

#include "math.h"
#include "stdint.h"

typedef unsigned char byte;
	
#define MATH_PI 3.141592653589793238463
#define MATH_TRANS  57.2958
#define MATH_L1 3.6
#define MATH_L2 0
#define MATH_L3 12
#define MATH_L4 12
#define MATH_L43 MATH_L4/MATH_L3

#define TopOffset 0
#define BottomOffset 0

#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          ('A')
#define SERVO_RIGHT_NUM         ('B')

// movement path types
#define PATH_LINEAR     0   // path based on linear interpolation
#define PATH_ANGLES     1   // path based on interpolation of servo angles

// movement absolute/relative flags
#define F_ABSOLUTE      0
#define F_POSN_RELATIVE 1
#define F_HAND_RELATIVE 2   // standard relative, current + hand parameter
#define F_HAND_ROT_REL  4   // hand keeps orientation relative to rotationxn servo (+/- hand parameter)
// #define F_NEXT_OPT   8   // these are flags, next option is next available bit

// interpolation types
#define INTERP_EASE_INOUT_CUBIC 0  // original cubic ease in/out
#define INTERP_LINEAR           1
#define INTERP_EASE_INOUT       2  // quadratic easing methods
#define INTERP_EASE_IN          3
#define INTERP_EASE_OUT         4

#define LINEAR_INTERCEPT        1
#define LINEAR_SLOPE            2

void Coordinate_To_Angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3);
double Angle_To_Coordinate_Y(double theta_1, double theta_2, double theta_3);
void Interpolate(double start_val, double end_val, double *interp_vals, byte ease_type);
void Set_Current_Angle(double theta_1, double theta_2, double theta_3);
void Get_Current_XYZ(double theta_1, double theta_2, double theta_3);
void Angle_To_Coordinate(double theta_1, double theta_2, double theta_3, double *x, double *y, double *z);
int Write_Servo_Angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle);
void Move_To(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte path_type, byte ease_type);
#endif
