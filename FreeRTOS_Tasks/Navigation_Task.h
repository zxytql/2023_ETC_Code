#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

#include "cmsis_os.h"
#include "bsp_ops.h"
#include "fuzzy_pid.h"

#define MAX_NAV_V 1.5f
#define MAX_NAV_W 0.5f

#define OPS2CEN 200 //mm

#define NAV_PATH_DONE (10)
#define NAV_PATH_NOT_DONE (11)
#define NAV_TRACK_FINAL (12)

#define TRACK_POINT_DONE (20)
#define TRACK_POINT_NOT_DONE (21)

typedef struct
{
    float x_set, x_now;
    float y_set, y_now;
    float yaw_set, yaw_now;

    float v_set;
    float v_plan;

    float yaw_tgt;

    uint8_t path_index;
    uint8_t point_index;
    uint8_t track_state; //µã¸ú×Ù×´Ì¬ DONE/NOT DONE
    uint8_t nav_state;   //¹ì¼£¸ú×Ù×´Ì¬ DONE/ NOT DONE

    float err_cir_x;
    float err_cir_y;

    float vx_limit;
    float vy_limit;

    // point_t now_point;
    // point_t plan_point;
}PID_nav_t;


typedef struct 
{
    float x;
    float y;
}point_t;

typedef struct 
{
    float vx;
    float vy;
    float ang_w;
}vel_t;

/**** Extern ****/
extern vel_t nav_vel;
extern PID_nav_t PID_nav;

void Ops_Coordinate_Trans(ops_data_t *ops, PID_nav_t *PID_nav);
void PID_Nav_Init(PID_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_ptr_w, fzy_pid_t *fzy_ptr_v);
float Navigation_Angle_Ctrl(float anglePresent,float angleTarget,float kp,float kd);
#endif
