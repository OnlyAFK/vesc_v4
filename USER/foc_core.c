/**
 * @file    foc_core.c
 * @brief   FOC 核心控制器实现
 */

#include "foc_core.h"
#include <math.h>

/*============================================================================*/
/*                              默认参数                                       */
/*============================================================================*/

/* 电流环参数 */
#define DEFAULT_ID_KP           0.07037f
#define DEFAULT_ID_KI           0.01423f
#define DEFAULT_IQ_KP           0.07037f
#define DEFAULT_IQ_KI           0.01423f
#define DEFAULT_CURRENT_LIMIT   12.0f       // 输出电压限幅 (V)

/* 速度环参数 */
#define DEFAULT_SPD_KP          0.006f
#define DEFAULT_SPD_KI          0.00001f
#define DEFAULT_SPEED_LIMIT     3.0f        // 输出电流限幅 (A)

/* 位置环参数 */
#define DEFAULT_POS_KP          500.0f
#define DEFAULT_POS_KI          0.0f
#define DEFAULT_MAX_RPM         2000.0f
#define DEFAULT_MAX_ACCEL       500.0f      // rad/s²
#define DEFAULT_MAX_JERK        50.0f       // RPM/tick

/* PLL 参数 */
#define DEFAULT_PLL_KP          200.0f
#define DEFAULT_PLL_KI          40000.0f

/* 控制周期 */
#define CONTROL_DT              0.00005f    // 50us
#define SPEED_LOOP_DT           0.001f      // 1ms
#define POS_LOOP_DT             0.001f      // 1ms

/* 常量 */
#define _2PI                    6.2831853f
#define RAD_S_TO_RPM            9.5492965855f

/*============================================================================*/
/*                              全局电机实例                                   */
/*============================================================================*/

Motor_t g_Motor = { .State = MOTOR_STATE_IDLE, .Mode = FOC_MODE_IDLE };

/*============================================================================*/
/*                              内部函数                                       */
/*============================================================================*/

/**
 * @brief  位置环初始化
 */
static void PosController_Init(PosController_t *pos)
{
    PID_Init(&pos->PID, DEFAULT_POS_KP, DEFAULT_POS_KI, 0.0f, 
             DEFAULT_MAX_RPM, -DEFAULT_MAX_RPM);
    
    pos->TargetPos = 0.0f;
    pos->CurrentPos = 0.0f;
    pos->LastTargetPos = 0.0f;
    pos->TotalTurns = 0;
    pos->LastTheta = 0.0f;
    pos->MaxRPM = DEFAULT_MAX_RPM;
    pos->MaxAccel = DEFAULT_MAX_ACCEL;
    pos->MaxJerk = DEFAULT_MAX_JERK;
    pos->LastOutputRPM = 0.0f;
}

/**
 * @brief  位置环传感器更新 (多圈累计)
 */
static void PosController_UpdateSensor(PosController_t *pos, float mech_theta)
{
    float delta = mech_theta - pos->LastTheta;
    
    /* 检测跨零点 */
    if (delta < -5.0f) {
        pos->TotalTurns++;
    } 
    else if (delta > 5.0f) {
        pos->TotalTurns--;
    }
    
    /* 计算累计位置 */
    pos->CurrentPos = (float)pos->TotalTurns * _2PI + mech_theta;
    pos->LastTheta = mech_theta;
}

/**
 * @brief  位置环计算 (带前馈和S曲线)
 */
static float PosController_Calc(PosController_t *pos)
{
    /* 前馈速度 */
    float delta_target = pos->TargetPos - pos->LastTargetPos;
    float ff_velocity_rpm = (delta_target / POS_LOOP_DT) * RAD_S_TO_RPM;
    pos->LastTargetPos = pos->TargetPos;
    
    /* 位置误差 */
    float error = pos->TargetPos - pos->CurrentPos;
    float sign = (error > 0.0f) ? 1.0f : -1.0f;
    float abs_err = fabsf(error);
    
    /* 死区处理 */
    if (abs_err < 0.002f) abs_err = 0.0f;
    
    /* 速度规划: 取三者最小值 */
    float v_cruise = pos->MaxRPM * 0.10472f;                        // 巡航速度 (rad/s)
    float v_brake = sqrtf(2.0f * pos->MaxAccel * abs_err);          // 制动速度
    float v_land = 5.0f * abs_err;                                   // 着陆速度
    
    float fb_rad_s = fminf(v_cruise, fminf(v_brake, v_land));
    float fb_rpm = sign * fb_rad_s * RAD_S_TO_RPM;
    
    /* 叠加前馈 */
    float final_rpm = fb_rpm + ff_velocity_rpm;
    
    /* 跃变限制 (平滑) */
    float rpm_diff = final_rpm - pos->LastOutputRPM;
    if (rpm_diff > pos->MaxJerk) {
        final_rpm = pos->LastOutputRPM + pos->MaxJerk;
    } else if (rpm_diff < -pos->MaxJerk) {
        final_rpm = pos->LastOutputRPM - pos->MaxJerk;
    }
    pos->LastOutputRPM = final_rpm;
    
    /* 速度限幅 */
    if (final_rpm > pos->MaxRPM)  final_rpm = pos->MaxRPM;
    if (final_rpm < -pos->MaxRPM) final_rpm = -pos->MaxRPM;
    
    return final_rpm;
}

/*============================================================================*/
/*                              公开接口                                       */
/*============================================================================*/

/**
 * @brief  初始化电机对象
 */
void FOC_Init(Motor_t *motor)
{
    /* 清零所有数据 */
    motor->State = MOTOR_STATE_IDLE;
    motor->Mode = FOC_MODE_IDLE;
    
    /* 初始化编码器方向 */
    motor->Encoder.Direction = 1;
    
    /* 初始化电源参数 */
    motor->Vdc = 12.0f;
    motor->PwmPeriod = HW_PWM_PERIOD;
    
    /* 初始化电流环 */
    PID_Init(&motor->PID_Id, DEFAULT_ID_KP, DEFAULT_ID_KI, 0.0f,
             DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
    PID_Init(&motor->PID_Iq, DEFAULT_IQ_KP, DEFAULT_IQ_KI, 0.0f,
             DEFAULT_CURRENT_LIMIT, -DEFAULT_CURRENT_LIMIT);
    
    /* 初始化速度环 */
    PID_Init(&motor->PID_Speed, DEFAULT_SPD_KP, DEFAULT_SPD_KI, 0.0f,
             DEFAULT_SPEED_LIMIT, -DEFAULT_SPEED_LIMIT);
    
    /* 初始化位置环 */
    PosController_Init(&motor->PosCtrl);
    
    /* 初始化 PLL */
    PLL_Init(&motor->SpeedPLL, DEFAULT_PLL_KP, DEFAULT_PLL_KI);
    
    /* 初始化 SVPWM */
    motor->SVPWM.Ts = motor->PwmPeriod;
    motor->SVPWM.Udc = motor->Vdc;
    
    /* 初始化分频计数 */
    motor->SpeedLoopCnt = 0;
    motor->PosLoopCnt = 0;
    
    /* 初始化硬件层 */
    MotorHW_Init();
}

/**
 * @brief  设置控制模式
 */
void FOC_SetMode(Motor_t *motor, FOC_Mode_t mode)
{
    if (motor->State == MOTOR_STATE_RUNNING) {
        /* 切换模式时复位控制器 */
        PID_Reset(&motor->PID_Id);
        PID_Reset(&motor->PID_Iq);
        PID_Reset(&motor->PID_Speed);
        PID_Reset(&motor->PosCtrl.PID);
    }
    motor->Mode = mode;
}

/**
 * @brief  设置目标电流
 */
void FOC_SetTargetCurrent(Motor_t *motor, float id, float iq)
{
    motor->TargetId = id;
    motor->TargetIq = iq;
}

/**
 * @brief  设置目标转速
 */
void FOC_SetTargetSpeed(Motor_t *motor, float rpm)
{
    motor->TargetRPM = rpm;
}

/**
 * @brief  设置目标位置
 */
void FOC_SetTargetPosition(Motor_t *motor, float pos_rad)
{
    motor->TargetPos = pos_rad;
    motor->PosCtrl.TargetPos = pos_rad;
}

/**
 * @brief  电流偏移校准处理
 */
uint8_t FOC_CalibrateCurrentOffset(Motor_t *motor)
{
    uint32_t adc_u, adc_v, adc_w;
    MotorHW_GetCurrentADC(&adc_u, &adc_v, &adc_w);
    
    return MotorHW_CalibrateCurrentOffset(&motor->CurOffset, adc_u, adc_v, adc_w);
}

/**
 * @brief  编码器数据回调
 */
void FOC_EncoderCallback(Motor_t *motor)
{
    MotorHW_ProcessEncoderData(&motor->Encoder);
}

/**
 * @brief  FOC 主控制循环
 */
void FOC_ControlLoop(Motor_t *motor)
{
    /* 调试引脚置高 */
    MotorHW_DebugPin(1);
    
    /*--- 1. 电流采样 ---*/
    MotorHW_GetCurrents(&motor->Currents, &motor->CurOffset);
    
    /*--- 2. 启动编码器读取 ---*/
    MotorHW_StartEncoderRead();
    
    /*--- 3. Clarke 变换: Iabc → Iαβ ---*/
    motor->Clarke.Ia = motor->Currents.Iu;
    motor->Clarke.Ib = motor->Currents.Iv;
    motor->Clarke.Ic = motor->Currents.Iw;
    Clarke_Calc(&motor->Clarke);
    
    /*--- 4. Park 变换: Iαβ → Idq ---*/
    motor->Park.Alpha = motor->Clarke.Alpha;
    motor->Park.Beta = motor->Clarke.Beta;
    motor->Park.Theta = motor->Encoder.ElecAngle;
    Park_Calc(&motor->Park);
    
    motor->ActualId = motor->Park.D;
    motor->ActualIq = motor->Park.Q;
    
    /*--- 5. PLL 速度估算 ---*/
    PLL_Update(&motor->SpeedPLL, motor->Encoder.MechAngle, CONTROL_DT);
    motor->ActualRPM = motor->SpeedPLL.SpeedRPM;
    
    /*--- 6. 速度环 (分频执行) ---*/
    motor->SpeedLoopCnt++;
    if (motor->SpeedLoopCnt >= HW_SPEED_LOOP_DIV) {
        motor->SpeedLoopCnt = 0;
        
        if (motor->Mode == FOC_MODE_SPEED || motor->Mode == FOC_MODE_POSITION) {
            float speed_out = PI_Calc(&motor->PID_Speed, motor->TargetRPM, motor->ActualRPM);
            motor->TargetIq = speed_out;
        }
    }
    
    /*--- 7. 位置环 (分频执行) ---*/
    motor->PosLoopCnt++;
    if (motor->PosLoopCnt >= HW_POS_LOOP_DIV) {
        motor->PosLoopCnt = 0;
        
        PosController_UpdateSensor(&motor->PosCtrl, motor->Encoder.MechAngle);
        motor->ActualPos = motor->PosCtrl.CurrentPos;
        
        if (motor->Mode == FOC_MODE_POSITION) {
            motor->TargetRPM = PosController_Calc(&motor->PosCtrl);
        }
    }
    
    /*--- 8. 电流环 ---*/
    float vd_out, vq_out;
    
    if (motor->Mode == FOC_MODE_IDLE) {
        vd_out = 0.0f;
        vq_out = 0.0f;
    } else {
        vd_out = PI_Calc(&motor->PID_Id, motor->TargetId, motor->ActualId);
        vq_out = PI_Calc(&motor->PID_Iq, motor->TargetIq, motor->ActualIq);
    }
    
    /*--- 9. 逆 Park 变换: Vdq → Vαβ ---*/
    motor->InvPark.D = vd_out;
    motor->InvPark.Q = vq_out;
    motor->InvPark.Theta = motor->Encoder.ElecAngle;
    InvPark_Calc(&motor->InvPark);
    
    /*--- 10. SVPWM 调制 ---*/
    motor->SVPWM.Alpha = motor->InvPark.Alpha;
    motor->SVPWM.Beta = motor->InvPark.Beta;
    motor->SVPWM.Udc = motor->Vdc;
    motor->SVPWM.Ts = motor->PwmPeriod;
    SVPWM_Calc(&motor->SVPWM);
    
    /*--- 11. PWM 输出 ---*/
    MotorHW_SetPWM(motor->SVPWM.CCR1, motor->SVPWM.CCR2, motor->SVPWM.CCR3);
    
    /* 调试引脚置低 */
    MotorHW_DebugPin(0);
}

/**
 * @brief  启动电机
 */
void FOC_Start(Motor_t *motor)
{
    motor->State = MOTOR_STATE_RUNNING;
    MotorHW_EnableDriver();
}

/**
 * @brief  停止电机
 */
void FOC_Stop(Motor_t *motor)
{
    motor->State = MOTOR_STATE_IDLE;
    motor->Mode = FOC_MODE_IDLE;
    
    /* 复位控制器 */
    PID_Reset(&motor->PID_Id);
    PID_Reset(&motor->PID_Iq);
    PID_Reset(&motor->PID_Speed);
    
    /* 设置 PWM 为 50% (刹车) */
    MotorHW_SetPWMBrake();
}

/**
 * @brief  急停电机
 */
void FOC_EmergencyStop(Motor_t *motor)
{
    motor->State = MOTOR_STATE_ERROR;
    motor->Mode = FOC_MODE_IDLE;
    
    /* 禁用驱动 */
    MotorHW_DisableDriver();
    MotorHW_SetPWMBrake();
}
