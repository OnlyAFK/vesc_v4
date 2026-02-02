/**
 * @file    position_loop.c
 * @brief   ?????? (???)
 * @note    ?????????????????? foc_core.c
 */

#include "position_loop.h"
#include <math.h>

#define _2PI  6.2831853f
#define POS_LOOP_DT  0.001f
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/*============================================================================*/
/*                    ??????????                                      */
/*============================================================================*/

Pos_PID_t pid_pos = {0};

/*============================================================================*/
/*                    ????????                                          */
/*============================================================================*/

/**
 * @brief  ??????
 */
void Pos_Loop_Init(Pos_PID_t *pid)
{
    pid->Kp = 500.0f;
    pid->Max_RPM = 2000.0f;
    pid->Target_Pos = 3.14f;
    pid->Current_Pos = 0.0f;
    pid->Total_Turns = 0;
    pid->Last_Theta = 0.0f;
    pid->Last_Target_Pos = 0.0f;
}

/**
 * @brief  ????????? (????)
 */
void Pos_Update_Sensor(Pos_PID_t *pid, float raw_theta)
{
    float delta = raw_theta - pid->Last_Theta;
    
    /* ????? */
    if (delta < -5.0f) {
        pid->Total_Turns++;
    }
    else if (delta > 5.0f) {
        pid->Total_Turns--;
    }
    
    /* ?????? */
    pid->Current_Pos = (float)pid->Total_Turns * _2PI + raw_theta;
    pid->Last_Theta = raw_theta;
}

/**
 * @brief  ??? PID ?? (????S??)
 */
float Pos_PID_Calc(Pos_PID_t *pid)
{
    /* ???? */
    float delta_target = pid->Target_Pos - pid->Last_Target_Pos;
    float ff_velocity_rad_s = delta_target / POS_LOOP_DT;
    float ff_velocity_rpm = ff_velocity_rad_s * 9.5493f;
    pid->Last_Target_Pos = pid->Target_Pos;
    
    /* ???? */
    float error = pid->Target_Pos - pid->Current_Pos;
    float sign = (error > 0) ? 1.0f : -1.0f;
    float abs_err = fabsf(error);
    
    /* ???? */
    if (abs_err < 0.002f) abs_err = 0;
    
    /* ???? */
    float max_speed_rpm = pid->Max_RPM;
    float max_accel_rad_s2 = 500.0f;
    float landing_kp = 5.0f;
    
    float v_cruise = max_speed_rpm * 0.10472f;
    float v_brake = sqrtf(2.0f * max_accel_rad_s2 * abs_err);
    float v_land = landing_kp * abs_err;
    
    float fb_rad_s = MIN(v_cruise, MIN(v_brake, v_land));
    float fb_rpm = sign * fb_rad_s * 9.5493f;
    float final_target_rpm = fb_rpm + ff_velocity_rpm;
    
    /* ???? */
    static float last_output_rpm = 0.0f;
    float max_jerk_per_tick = 50.0f;
    float rpm_diff = final_target_rpm - last_output_rpm;
    if (rpm_diff > max_jerk_per_tick) final_target_rpm = last_output_rpm + max_jerk_per_tick;
    else if (rpm_diff < -max_jerk_per_tick) final_target_rpm = last_output_rpm - max_jerk_per_tick;
    
    last_output_rpm = final_target_rpm;
    
    /* ???? */
    if (final_target_rpm > max_speed_rpm) final_target_rpm = max_speed_rpm;
    if (final_target_rpm < -max_speed_rpm) final_target_rpm = -max_speed_rpm;
    
    return final_target_rpm;
}
