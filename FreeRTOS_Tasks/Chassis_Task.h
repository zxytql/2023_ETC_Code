#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/******** Struct ********/
typedef struct
{
    /** ���������ٶ� m/s **/
    float ct_vx;
    float ct_vy;
    float ct_wz;
}chassis_ct_t;

typedef struct
{
    /** �����ٶ� m/s **/
    float ct_fdb_vx;
    float ct_fdv_vy;
    float ct_fdb_wz;
}chassis_ct_fdb_t;

typedef struct
{
    /** ң������ֵ������˶�����ϵ�� **/
    float vx_co;
    float vy_co;
    float wz_co;
}chassis_rc_co_t;

typedef struct
{
    chassis_ct_t chassis_ct;
    chassis_ct_fdb_t chassis_ct_fdb;
    chassis_rc_co_t chassis_rc_co;

    float wheel_speed[4]; /** ������ٶ� m/s **/
    float wheel_fdb_speed[4]; /** ����ת�� m/s **/
}chassis_t;

extern chassis_t chassis;

void OD_Axis_Init(void);

#endif
