#include "position_loop.h"
#include "main.h" // 假设里面定义了 PI = 3.1415926f

#define _2PI  6.2831853f

Pos_PID_t pid_pos;

// 1. 初始化
void Pos_Loop_Init(Pos_PID_t *pid) {
    pid->Kp = 10.0f;          // 初始 P 值，建议从小开始测
    pid->Max_RPM = 700.0f;    // 限制最高转速 200RPM，安全第一
    pid->Target_Pos = 0.0f;
    pid->Current_Pos = 0.0f;
    pid->Total_Turns = 0;
    pid->Last_Theta = 0.0f;
}

// 2. 传感器更新：把 0~2PI 变成无限累加的角度
// raw_theta: 编码器读出来的原始值 (0 ~ 6.28)
void Pos_Update_Sensor(Pos_PID_t *pid, float raw_theta) {
    float delta = raw_theta - pid->Last_Theta;

    // --- 核心：检测过零点 ---
    // 如果一次变化超过了 80% 的圆 (即 > 0.8 * 2PI)，说明刚才发生了一次跳变
    if (delta < -5.0f) { 
        // 比如从 6.20 变到 0.10，delta = -6.1，说明正转了一圈
        pid->Total_Turns++;
    } 
    else if (delta > 5.0f) {
        // 比如从 0.10 变到 6.20，delta = +6.1，说明反转了一圈
        pid->Total_Turns--;
    }

    // 计算累计角度 = 圈数 * 2PI + 当前单圈角度
    pid->Current_Pos = (float)pid->Total_Turns * _2PI + raw_theta;
    
    // 保存历史值
    pid->Last_Theta = raw_theta;
}

// 3. 位置环 PID 计算 (P控制)
// 返回值：给速度环的目标转速 (Target RPM)
float Pos_PID_Calc(Pos_PID_t *pid) {
    float error = pid->Target_Pos - pid->Current_Pos;

    // P 控制: 误差越大，给的速度越快
    float target_rpm = pid->Kp * error;

    // --- 输出限幅 (安全保护) ---
    if (target_rpm > pid->Max_RPM)  target_rpm = pid->Max_RPM;
    if (target_rpm < -pid->Max_RPM) target_rpm = -pid->Max_RPM;

    return target_rpm;
}
