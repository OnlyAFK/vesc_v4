/**
 * @file    foc_core.h
 * @brief   FOC 核心控制器
 * @note    封装完整的 FOC 控制逻辑，包含状态机和三环级联控制
 */

#ifndef __FOC_CORE_H
#define __FOC_CORE_H

#include <stdint.h>
#include "foc_math.h"
#include "pid.h"
#include "pll.h"
#include "svpwm.h"
#include "motor_hw.h"

/*============================================================================*/
/*                              配置参数                                       */
/*============================================================================*/

/* 控制模式 */
typedef enum {
    FOC_MODE_IDLE = 0,          // 空闲模式
    FOC_MODE_CURRENT,           // 电流环模式 (力矩控制)
    FOC_MODE_SPEED,             // 速度环模式
    FOC_MODE_POSITION,          // 位置环模式
    FOC_MODE_OPENLOOP,          // 开环 V/F 控制 (调试用)
} FOC_Mode_t;

/* 电机状态 */
typedef enum {
    MOTOR_STATE_IDLE = 0,       // 空闲
    MOTOR_STATE_CALIBRATING,    // 校准中
    MOTOR_STATE_RUNNING,        // 运行中
    MOTOR_STATE_ERROR,          // 故障
} Motor_State_t;

/*============================================================================*/
/*                              电机对象结构体                                  */
/*============================================================================*/

/**
 * @brief 位置环控制器 (带前馈和S曲线)
 */
typedef struct {
    PID_Controller_t PID;       // PID 控制器
    
    float TargetPos;            // 目标位置 (rad, 累计)
    float CurrentPos;           // 当前位置 (rad, 累计)
    float LastTargetPos;        // 上次目标位置 (前馈用)
    
    int32_t TotalTurns;         // 圈数累计
    float LastTheta;            // 上次机械角度 (0~2π)
    
    float MaxRPM;               // 最大转速限制
    float MaxAccel;             // 最大加速度限制 (rad/s²)
    float MaxJerk;              // 最大跃变限制 (RPM/tick)
    float LastOutputRPM;        // 上次输出 (平滑用)
} PosController_t;

/**
 * @brief 电机对象结构体 - 封装所有 FOC 相关数据
 */
typedef struct {
    /*--- 状态 ---*/
    Motor_State_t State;        // 电机状态
    FOC_Mode_t Mode;            // 控制模式
    
    /*--- 硬件数据 ---*/
    EncoderData_t Encoder;      // 编码器数据
    PhaseCurrents_t Currents;   // 三相电流
    CurrentOffset_t CurOffset;  // 电流偏移
    
    /*--- 坐标变换 ---*/
    Clarke_t Clarke;            // Clarke 变换
    Park_t Park;                // Park 变换
    InvPark_t InvPark;          // 逆 Park 变换
    SVPWM_t SVPWM;              // SVPWM 调制
    
    /*--- 控制器 ---*/
    PID_Controller_t PID_Id;    // d轴电流环
    PID_Controller_t PID_Iq;    // q轴电流环
    PID_Controller_t PID_Speed; // 速度环
    PosController_t  PosCtrl;   // 位置环
    PLL_t SpeedPLL;             // PLL 速度估算
    
    /*--- 目标值 ---*/
    float TargetId;             // d轴目标电流 (A)
    float TargetIq;             // q轴目标电流 (A)
    float TargetRPM;            // 目标转速 (RPM)
    float TargetPos;            // 目标位置 (rad)
    
    /*--- 反馈值 ---*/
    float ActualId;             // d轴实际电流 (A)
    float ActualIq;             // q轴实际电流 (A)
    float ActualRPM;            // 实际转速 (RPM)
    float ActualPos;            // 实际位置 (rad)
    
    /*--- 电源参数 ---*/
    float Vdc;                  // 母线电压 (V)
    uint32_t PwmPeriod;         // PWM 周期 (ARR值)
    
    /*--- 分频计数 ---*/
    uint8_t SpeedLoopCnt;       // 速度环分频计数
    uint8_t PosLoopCnt;         // 位置环分频计数
    
} Motor_t;

/*============================================================================*/
/*                              函数接口                                       */
/*============================================================================*/

/**
 * @brief  初始化电机对象
 * @param  motor: 电机对象指针
 */
void FOC_Init(Motor_t *motor);

/**
 * @brief  设置控制模式
 * @param  motor: 电机对象指针
 * @param  mode: 控制模式
 */
void FOC_SetMode(Motor_t *motor, FOC_Mode_t mode);

/**
 * @brief  设置目标电流 (电流环模式)
 * @param  motor: 电机对象指针
 * @param  id: d轴目标电流 (A)
 * @param  iq: q轴目标电流 (A)
 */
void FOC_SetTargetCurrent(Motor_t *motor, float id, float iq);

/**
 * @brief  设置目标转速 (速度环模式)
 * @param  motor: 电机对象指针
 * @param  rpm: 目标转速 (RPM)
 */
void FOC_SetTargetSpeed(Motor_t *motor, float rpm);

/**
 * @brief  设置目标位置 (位置环模式)
 * @param  motor: 电机对象指针
 * @param  pos_rad: 目标位置 (rad)
 */
void FOC_SetTargetPosition(Motor_t *motor, float pos_rad);

/**
 * @brief  FOC 主控制循环 (在 ADC 中断中调用, 20kHz)
 * @param  motor: 电机对象指针
 * @note   此函数执行完整的 FOC 控制流程:
 *         1. 电流采样与处理
 *         2. 编码器读取
 *         3. 坐标变换 (Clarke/Park)
 *         4. 速度估算 (PLL)
 *         5. 速度环 (分频执行)
 *         6. 位置环 (分频执行)
 *         7. 电流环
 *         8. 逆变换 + SVPWM
 *         9. PWM 输出
 */
void FOC_ControlLoop(Motor_t *motor);

/**
 * @brief  电流偏移校准处理 (在 FOC_ControlLoop 之前调用)
 * @param  motor: 电机对象指针
 * @return 1=校准完成, 0=校准中
 */
uint8_t FOC_CalibrateCurrentOffset(Motor_t *motor);

/**
 * @brief  编码器数据回调 (在 SPI DMA 完成中断中调用)
 * @param  motor: 电机对象指针
 */
void FOC_EncoderCallback(Motor_t *motor);

/**
 * @brief  启动电机
 * @param  motor: 电机对象指针
 */
void FOC_Start(Motor_t *motor);

/**
 * @brief  停止电机
 * @param  motor: 电机对象指针
 */
void FOC_Stop(Motor_t *motor);

/**
 * @brief  急停电机
 * @param  motor: 电机对象指针
 */
void FOC_EmergencyStop(Motor_t *motor);

/*============================================================================*/
/*                              全局电机实例                                   */
/*============================================================================*/

extern Motor_t g_Motor;     // 全局电机对象

#endif /* __FOC_CORE_H */
