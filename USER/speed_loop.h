#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

typedef struct {
    float TargetRPM;    // 目标转速 (RPM)
    float ActualRPM;    // 实际转速 (RPM)
    float FilteredRPM;  // 滤波后的转速 (RPM)
    
    float Kp;
    float Ki;
    float Integral;
    float Out_Max;      // 输出限制 (即最大允许电流 A)
    float Out_Min;
    
    float Out;          // PID输出 (目标电流 Iq_ref)
} Speed_PID_t;

extern Speed_PID_t pid_spd;
extern float speed_filter_const; // 滤波系数 (0.0~1.0)

void Speed_Loop_Init(void);
void Speed_Calc(float theta_mech_now); 
void Speed_PID_Calc(void);
void Speed_Calc_PLL(float theta_now, float Ts);

#endif


