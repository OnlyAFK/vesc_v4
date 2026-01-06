#include "speed_loop.h"
#include "foc_math.h" 

#define PI 3.14159265358f

Speed_PID_t pid_spd;

float speed_filter_const = 0.03f; // 滤波系数，越小滤波越强，延迟越大

// 内部变量用于差分计算
static float theta_prev = 0.0f;

void Speed_Loop_Init(void) {
    pid_spd.Kp = 0.006f;//0.03f;       
    pid_spd.Ki = 0.00001f; //  0.0003f;    
    pid_spd.Out_Max = 3.0f;   
    pid_spd.Out_Min = -3.0f;
    pid_spd.Integral = 0.0f;
    pid_spd.TargetRPM = 0.0f;
}

// theta_now: 当前机械角度 (0 ~ 6.28)
void Speed_Calc(float theta_now) {
    float delta_theta = theta_now - theta_prev;

    if (delta_theta > PI) {
        delta_theta -= 2.0f * PI; 
    }
    else if (delta_theta < -PI) {
        delta_theta += 2.0f * PI;
    }
    
    theta_prev = theta_now;


    // omega = d_theta / dt
    float omega_inst = delta_theta / 0.00005f; 

    // RPM: RPM = (rad/s) * 60 / (2*PI)
    float rpm_inst = omega_inst * 9.549296f;

    // --- (LPF) ---
    // y[k] = alpha * x[k] + (1-alpha) * y[k-1]
    pid_spd.FilteredRPM = speed_filter_const * rpm_inst + 
                          (1.0f - speed_filter_const) * pid_spd.FilteredRPM;
    
    pid_spd.ActualRPM = pid_spd.FilteredRPM;
}

void Speed_PID_Calc(void) {
    float error = pid_spd.TargetRPM - pid_spd.ActualRPM;
    
    pid_spd.Integral += pid_spd.Ki * error;
    
    if (pid_spd.Integral > pid_spd.Out_Max) pid_spd.Integral = pid_spd.Out_Max;
    if (pid_spd.Integral < pid_spd.Out_Min) pid_spd.Integral = pid_spd.Out_Min;

    pid_spd.Out = pid_spd.Kp * error + pid_spd.Integral;

    if (pid_spd.Out > pid_spd.Out_Max) pid_spd.Out = pid_spd.Out_Max;
    if (pid_spd.Out < pid_spd.Out_Min) pid_spd.Out = pid_spd.Out_Min;
}


typedef struct {
    float angle_est;      // 估算角度
    float speed_est;      // 估算速度 (rad/s)
    float Kp;             // PLL 比例增益 (带宽)
    float Ki;             // PLL 积分增益
} PLL_State_t;

PLL_State_t pll = {0, 0, 200.0f, 40000.0f}; // 默认参数，适合大多数情况

// PLL 速度计算函数
// theta_now: 当前机械角度 (0 ~ 6.28)
void Speed_Calc_PLL(float theta_now, float Ts) {
    float angle_err = theta_now - pll.angle_est;
    if (angle_err > 3.1415926f)  angle_err -= 6.2831853f;
    if (angle_err < -3.1415926f) angle_err += 6.2831853f;

    pll.speed_est += pll.Ki * angle_err * Ts;
    
    pll.angle_est += (pll.speed_est + pll.Kp * angle_err) * Ts;

    if (pll.angle_est > 6.2831853f) pll.angle_est -= 6.2831853f;
    if (pll.angle_est < 0.0f)       pll.angle_est += 6.2831853f;

    float rpm_clean = pll.speed_est * 9.549296f;
    
    pid_spd.ActualRPM = rpm_clean;
}


