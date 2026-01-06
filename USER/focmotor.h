#ifndef _FOCMOTOR_H_
#define _FOCMOTOR_H_
#include "as5047p.h"
#include "foc_math.h"
#include "svpwm.h"
#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"

typedef struct {
    // 1. 状态机
    enum { MOTOR_IDLE, MOTOR_CALIB, MOTOR_RUN, MOTOR_ERROR } State;
    
    // 2. 传感器对象
//    AS5047_Handle_t Encoder;
    
    // 3. FOC 计算中间量
    clarke_t Clarke;
    park_t Park;
    ipark_t IPark;
    svpwm_t SVPWM;
    
    // 4. PID 控制器
    PID_t Pid_Id;
    PID_t Pid_Iq;
    PID_t Pid_Speed;
    PID_t Pid_Pos;
    
    // 5. 目标值与反馈
    float Target_Speed;
    float Target_Pos;
    float Iu, Iv, Iw;
    float ElecTheta;
    
    // 6. 硬件接口 (解耦关键!)
    void (*SetPWM)(uint32_t u, uint32_t v, uint32_t w);
    
} FOC_Motor_t;

#endif
