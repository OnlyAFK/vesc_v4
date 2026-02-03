/**
 * @file    position_loop.h
 * @brief   位置环控制器 (兼容层)
 * @note    此文件保留用于向后兼容，新代码请使用 foc_core.h
 */

#ifndef __POSITION_LOOP_H
#define __POSITION_LOOP_H

/*============================================================================*/
/*                    兼容旧代码的类型定义                                      */
/*============================================================================*/

/**
 * @brief 位置环 PID 结构体 (兼容旧版)
 */
typedef struct {
    float Kp;           // 位置环比例系数
    float Max_RPM;      // 限制最大转速 (防止飞车)
    
    float Target_Pos;   // 目标累计角度 (rad)
    float Current_Pos;  // 当前累计角度 (rad)
    
    int   Total_Turns;  // 圈数计数器
    float Last_Theta;   // 上一次的机械角度 (0~2PI)
    float Last_Target_Pos; // 上一次的目标位置 (用于计算前馈)
} Pos_PID_t;

/*============================================================================*/
/*                    兼容旧代码的全局变量                                      */
/*============================================================================*/

extern Pos_PID_t pid_pos;

/*============================================================================*/
/*                    兼容旧代码的函数                                          */
/*============================================================================*/

/**
 * @brief  初始化位置环
 * @param  pid: 位置环结构体指针
 */
void Pos_Loop_Init(Pos_PID_t *pid);

/**
 * @brief  更新位置传感器数据
 * @param  pid: 位置环结构体指针
 * @param  raw_theta: 原始机械角度 (0~2π)
 */
void Pos_Update_Sensor(Pos_PID_t *pid, float raw_theta);

/**
 * @brief  位置环 PID 计算
 * @param  pid: 位置环结构体指针
 * @return 目标转速 (RPM)
 */
float Pos_PID_Calc(Pos_PID_t *pid);

#endif /* __POSITION_LOOP_H */
