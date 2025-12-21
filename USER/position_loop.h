#ifndef __POSITION_LOOP_H
#define __POSITION_LOOP_H

typedef struct {
    // 参数区
    float Kp;           // 位置环比例系数
    float Max_RPM;      // 限制最大转速 (非常重要，防止飞车)
    
    // 状态区
    float Target_Pos;   // 目标累计角度 (rad)
    float Current_Pos;  // 当前累计角度 (rad)
    
    // 内部计算用
    int   Total_Turns;  // 圈数计数器
    float Last_Theta;   // 上一次的机械角度 (0~2PI)
} Pos_PID_t;

void Pos_Loop_Init(Pos_PID_t *pid);
void Pos_Update_Sensor(Pos_PID_t *pid, float raw_theta);
float Pos_PID_Calc(Pos_PID_t *pid);

extern Pos_PID_t pid_pos;

#endif
