#ifndef __CURRENT_LOOP_H_
#define __CURRENT_LOOP_H_

typedef struct {
    float Ref;          // 目标值 (期望电流, Unit: A)
    float Fdb;          // 反馈值 (实际电流, Unit: A)
    
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    
    float Integral;     // 积分项累加器
    
    float Out;          // PID输出 (电压, Unit: V)
    float Out_Max;      // 输出上限 (通常是 Udc / sqrt(3))
    float Out_Min;      // 输出下限
} PI_Controller_t;

void current_pi_init(void);
void current_calc(PI_Controller_t *pPI);

extern PI_Controller_t pid_id;
extern PI_Controller_t pid_iq;

#endif

