/**
  ****************************(C) COPYRIGHT 2023 HCRT****************************
  * @file       mec_arm_math.c/h
  * @version    V1.0.0
  * @brief      ��е�ۿ����㷨��
  * @history
  *  Version      Date            Author          Modification
  *  V1.0.0     2023-04-05         zxy            First version
	* @note
	*                 ^+Z
	*									|   +X
	*									|   /
	*									|  /
	*									| /
	*									|/-------->+Y
	*
	*  
  * @verbatim
  ****************************(C) COPYRIGHT 2022 HCRT****************************
  */
	
#include "mec_arm_math.h"
#include "cmsis_os.h"
#include "bsp_servo.h"
#include "math.h"

/******** Global Variables ********/
uint16_t INTERP_INTVLS = 0;
double cur_rot = 0;
double cur_left = 0;
double cur_right = 0;
double cur_hand = 0;

double g_current_x = 0;
double g_current_y = 0;
double g_current_z = 0;

void Coordinate_To_Angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3)
{
		if (z > (MATH_L1 + MATH_L3 + TopOffset))
		{
			  z = MATH_L1 + MATH_L3 + TopOffset;
		}

		if (z < (MATH_L1 - MATH_L4 + BottomOffset))
		{
				z = MATH_L1 - MATH_L4 + BottomOffset;
		}

		double x_in = 0.0;
//		double y_in = 0.0;
		double z_in = 0.0;
//		double right_all = 0.0;
		double right_all_2 = 0.0;
		double sqrt_z_x = 0.0;
//		double sqrt_z_y = 0.0;
		double phi = 0.0;

//		y_in = (-y-MATH_L2)/MATH_L3;
		z_in = (z - MATH_L1) / MATH_L3;
//		right_all = (1 - y_in*y_in - z_in*z_in - MATH_L43*MATH_L43) / (2 * MATH_L43);
//		sqrt_z_y = sqrt(z_in*z_in + y_in*y_in);
						// Calculate value of theta 1

		*theta_1 = atan(y / x)*MATH_TRANS;

		if (y / x > 0) {
						*theta_1 = *theta_1;
		}
		if (y / x < 0) {
						*theta_1 = *theta_1 + 180;
		}
		if (y == 0) {
						if (x > 0) *theta_1 = 180;
						else theta_1 = 0;
		}

		// Calculate value of theta 3

		x_in = (-x / cos(*theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;

		if (z_in == 0) { phi = 90; }

		else{ phi = atan(-x_in / z_in)*MATH_TRANS; }

		if (phi > 0) {phi = phi - 180; }

		sqrt_z_x = sqrt(z_in*z_in + x_in*x_in);

		right_all_2 = -1 * (z_in*z_in + x_in*x_in + MATH_L43*MATH_L43 - 1) / (2 * MATH_L43);
		*theta_3 = asin(right_all_2 / sqrt_z_x)*MATH_TRANS;
		*theta_3 = *theta_3 - phi;

		if (theta_1 <0 ) {
						theta_1 = 0;
		}

		// Calculate value of theta 2
		*theta_2 = asin(z_in + MATH_L43*sin(fabs(*theta_3 / MATH_TRANS)))*MATH_TRANS;

		*theta_1 = fabs(*theta_1);
		*theta_2 = fabs(*theta_2);

		if (theta_3 < 0 ) {}
		else{
						if ((Angle_To_Coordinate_Y(*theta_1,*theta_2, *theta_3)>y+0.1)||(Angle_To_Coordinate_Y(*theta_1,*theta_2, *theta_3)<y-0.1))
						{
										*theta_2 = 180 - *theta_2;
						}
		}
}

double Angle_To_Coordinate_Y(double theta_1, double theta_2, double theta_3)
{
		double l5_2 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));
		return -sin(fabs(theta_1 / MATH_TRANS))*l5_2;	
}

void Interpolate(double start_val, double end_val, double *interp_vals, byte ease_type)
{
//		double delta = end_val - start_val;
//		for (byte f = 0; f < INTERP_INTVLS; f++) {
//						switch (ease_type) {
//						case INTERP_LINEAR:
//										*(interp_vals+f) = delta * f / INTERP_INTVLS + start_val;
//										break;
//						case INTERP_EASE_INOUT:
//						{
//										float t = f / (INTERP_INTVLS / 2.0);
//										if (t < 1) {
//														*(interp_vals+f) = delta / 2 * t * t + start_val;
//										} else {
//														t--;
//														*(interp_vals+f)= -delta / 2 * (t * (t - 2) - 1) + start_val;
//										}
//						}
//						break;
//						case INTERP_EASE_IN:
//						{
//										float t = (float)f / INTERP_INTVLS;
//										*(interp_vals+f) = delta * t * t + start_val;
//						}
//						break;
//						case INTERP_EASE_OUT:
//						{
//										float t = (float)f / INTERP_INTVLS;
//										*(interp_vals+f) = -delta * t * (t - 2) + start_val;
//						}
//						break;
//						case INTERP_EASE_INOUT_CUBIC: // this is a compact version of Joey's original cubic ease-in/out
//						{
//										float t = (float)f / INTERP_INTVLS;
//										*(interp_vals+f) = start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t);
//						}
//						break;
//						}
//		}	
}

void Get_Current_XYZ(double theta_1, double theta_2, double theta_3)
{
		double l5 = (MATH_L2 + MATH_L3*cos(theta_2 / MATH_TRANS) + MATH_L4*cos(theta_3 / MATH_TRANS));

		g_current_x = -cos(fabs(theta_1 / MATH_TRANS))*l5;
		g_current_y = -sin(fabs(theta_1 / MATH_TRANS))*l5;
		g_current_z = MATH_L1 + MATH_L3*sin(fabs(theta_2 / MATH_TRANS)) - MATH_L4*sin(fabs(theta_3 / MATH_TRANS));	
}

void Set_Current_Angle(double theta_1, double theta_2, double theta_3)
{
	Servo_Ctrl(SERVO_LEFT_NUM,theta_2);
	Servo_Ctrl(SERVO_RIGHT_NUM,theta_3);
	
	// refresh logical servo angle cache
	cur_rot = theta_1;
	cur_left = theta_2;
	cur_right = theta_3;
}

void Angle_To_Coordinate(double theta_1, double theta_2, double theta_3, double *x, double *y, double *z)
{
		Get_Current_XYZ(theta_1, theta_2, theta_3); *x = g_current_x; *y = g_current_y; *z = g_current_z;	
}

int Write_Servo_Angle(double servo_rot_angle, double servo_left_angle, double servo_right_angle)
{
		if(servo_left_angle < 10) servo_left_angle = 10;
		if(servo_left_angle > 120) servo_left_angle = 120;
		if(servo_right_angle < 10) servo_right_angle = 10;
		if(servo_right_angle > 110) servo_right_angle = 110;

		if(servo_left_angle + servo_right_angle > 160)
		{
						servo_right_angle = 160 - servo_left_angle;
						return -1;
		}
		//write_servo_angle(SERVO_ROT_NUM,servo_rot_angle,true);
		Servo_Ctrl(SERVO_LEFT_NUM, servo_left_angle);
		Servo_Ctrl(SERVO_RIGHT_NUM, servo_right_angle);

		// refresh logical servo angle cache
		cur_rot = servo_rot_angle;
		cur_left = servo_left_angle;
		cur_right = servo_right_angle;
		
		return 1;
}

void Move_To(double x, double y, double z, double hand_angle, byte relative_flags, double time, byte path_type, byte ease_type)
{
		float limit = sqrt((x*x + y*y));
		if (limit > 32)
		{
						float k = 32/limit;
						x = x * k;
						y = y * k;
		}
		// attach_all();
		// setServoStatus(true, SERVO_ROT_NUM);
		// setServoStatus(true, SERVO_LEFT_NUM);
		// setServoStatus(true, SERVO_RIGHT_NUM);
		// if(enable_hand)
		//         setServoStatus(true, SERVO_HAND_ROT_NUM);

		// find current position using cached servo values
		double current_x;
		double current_y;
		double current_z;
		Angle_To_Coordinate(cur_rot, cur_left, cur_right, &current_x, &current_y, &current_z);


		// deal with relative xyz positioning
//		byte posn_relative = (relative_flags & F_POSN_RELATIVE) ? 1 : 0;
//		x = current_x * posn_relative + x;
//		y = current_y * posn_relative + y;
//		z = current_z * posn_relative + z;

		// find target angles
		double tgt_rot;
		double tgt_left;
		double tgt_right;
		Coordinate_To_Angle(x, y, z, &tgt_rot, &tgt_left, &tgt_right);

		//calculate the length
		unsigned int delta_rot=fabs(tgt_rot-cur_rot);
		unsigned int delta_left=fabs(tgt_left-cur_left);
		unsigned int delta_right=fabs(tgt_right-cur_right);

		//Serial.println(tgt_rot,DEC);

		INTERP_INTVLS = fmax(delta_rot,delta_left);
		INTERP_INTVLS = fmax(INTERP_INTVLS,delta_right);

		//Serial.println(INTERP_INTVLS,DEC);
		INTERP_INTVLS=(INTERP_INTVLS<80) ? INTERP_INTVLS : 80;

		// deal with relative hand orientation
//		if (relative_flags & F_HAND_RELATIVE) {
//						hand_angle += cur_hand;             // rotates a relative amount, ignoring base rotation
//		} else if (relative_flags & F_HAND_ROT_REL) {
//						hand_angle = hand_angle + cur_hand + (tgt_rot - cur_rot); // rotates relative to base servo, 0 value keeps an object aligned through movement
//		}


		if (time > 0) {
						if (path_type == PATH_ANGLES) {
										// we will calculate angle value targets
										double rot_array[INTERP_INTVLS];
										double left_array[INTERP_INTVLS];
										double right_array[INTERP_INTVLS];
//										double hand_array[INTERP_INTVLS];

										Interpolate(cur_rot, tgt_rot, rot_array, ease_type);
										Interpolate(cur_left, tgt_left, left_array, ease_type);
										Interpolate(cur_right, tgt_right, right_array, ease_type);
//										Interpolate(cur_hand, hand_angle, hand_array, ease_type);

										for (byte i = 0; i < INTERP_INTVLS; i++)
										{
												Write_Servo_Angle(rot_array[i], left_array[i], right_array[i]);
												osDelay(time * 1000 / INTERP_INTVLS);
										}
						} else if (path_type == PATH_LINEAR) {
										// we will calculate linear path targets
										double x_array[INTERP_INTVLS];
										double y_array[INTERP_INTVLS];
										double z_array[INTERP_INTVLS];
//										double hand_array[INTERP_INTVLS];

										Interpolate(current_x, x, x_array, ease_type);
										Interpolate(current_y, y, y_array, ease_type);
										Interpolate(current_z, z, z_array, ease_type);
//										Interpolate(cur_hand, hand_angle, hand_array, ease_type);

										for (byte i = 0; i < INTERP_INTVLS; i++)
										{
														double rot,left, right;
														Coordinate_To_Angle(x_array[i], y_array[i], z_array[i], &rot, &left, &right);
														Write_Servo_Angle(rot, left, right);
														osDelay(time * 1000 / INTERP_INTVLS);
										}
						}
		}

		// set final target position at end of interpolation or atOnce
		Write_Servo_Angle(tgt_rot, tgt_left, tgt_right);
}


