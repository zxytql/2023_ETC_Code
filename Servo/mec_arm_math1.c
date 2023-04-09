#include "mec_arm_math1.h"
#include "bsp_servo.h"


/******** Global Variables ********/
uint16_t INTERP_INTVLS = 0;
interpolate_param_t interpolate_param;

double cur_rot = 0;
double cur_left = 0;
double cur_right = 0;

double tgt_rot;
double tgt_left;
double tgt_right;

double temp_rot;
double temp_left;
double temp_right;

double y_cur = 0;
double z_cur = 0;
double y_tgt = 0;
double z_tgt = 0;

double zxy3 = 0;
double zxy4 = 0;

void Coordinate_To_Angle(double x, double y, double z, double *theta_1, double *theta_2, double *theta_3)
{
		double l = sqrt(pow(y,2)+pow(z,2));
		double alpha = acos((pow(MATH_L3,2)+pow(l,2)-pow(MATH_L2,2)) / (2.0f * MATH_L3 * l));  
		double beta = acos((pow(l,2)+pow(MATH_L2,2)-pow(MATH_L3,2)) / (2.0f * l * MATH_L2)); 
		
//	zxy3 = alpha;
//	zxy4 = beta;
		*theta_1 = 90;
		*theta_2 = -(alpha - asin(z / l)) * MATH_TRANS;
		*theta_3 = (MATH_PI / 2 - asin(z / l) - beta) * MATH_TRANS;
	
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

void Set_Current_XYZ(double x, double y, double z)
{
		Coordinate_To_Angle(x, y, z, &tgt_rot, &tgt_left, &tgt_right);
	
		Set_Current_Angle(tgt_rot, tgt_left, tgt_right);		
		
		//Flash the current values
		y_cur = y;
		z_cur = z;
}
void Interpolate_YZ(interpolate_param_t *ptr, double y_cur, double z_cur, double y_tgt, double z_tgt, double time)
{
	// current -> target
	static double v_begin = 0.0f;
	static double v_end = 0.0f;
	
	//Calculate Y axis interpolate parameters
	ptr->y_a = y_cur;
	ptr->y_b = v_begin;
	ptr->y_c = pow(time,-2) * (3 * (y_tgt - y_cur)) - (2 * v_begin + v_end) / time;
	ptr->y_d = -pow(time,-3) * (2 * (y_tgt - y_cur)) + (v_end + v_begin) / pow(time,2);
	
	//Calculate Z axis interpolate parameters
	ptr->z_a = z_cur;
	ptr->z_b = v_begin;
	ptr->z_c = pow(time,-2) * (3 * (z_tgt - z_cur)) - (2 * v_begin + v_end) / time;
	ptr->z_d = -pow(time,-3) * (2 * (z_tgt - z_cur)) + (v_end + v_begin) / pow(time,2);	
	
	INTERP_INTVLS = time / 20; //Servo control frequency is 50Hz, 20ms.
	
	for(int i = 0; i < (int)INTERP_INTVLS; ++i)
	{
		uint16_t now_time = 20 * i;
		y_cur = ptr->y_a + ptr->y_b * now_time + ptr->y_c * pow(now_time,2) + ptr->y_d * pow(now_time,3);
		z_cur = ptr->z_a + ptr->z_b * now_time + ptr->z_c * pow(now_time,2) + ptr->z_d * pow(now_time,3);
		
		Coordinate_To_Angle(0, y_cur, z_cur, &temp_rot, &temp_left, &temp_right);
		Set_Current_Angle(temp_rot, temp_left, temp_right);
		
		osDelay(20);
	}
	
}
void Move_To(double x, double y, double z, double time)
{
		Coordinate_To_Angle(x, y, z, &tgt_rot, &tgt_left, &tgt_right);
		Interpolate_YZ(&interpolate_param,y_cur,z_cur,y,z,time);

		Set_Current_Angle(tgt_rot, tgt_left, tgt_right);
}


