#include "interrupt.h"
#include "adc.h"
#include "foc_math.h"
#include "svpwm.h"
#include "vofa.h" 
#include "gpio.h"
#include "arm_math.h"
#include "current_loop.h"
#include "as5047p.h"
#include "speed_loop.h"
#include "filter.h"
#include "position_loop.h"

float test_elec_theta;
float freq_hz = 0.0f;
float Ts = 0.00005f;
uint8_t test_flag = 1;

#define GAIN_OPAMP   20.0f           // AD8418A 增益
#define VREF         3.3f            // ADC 参考电压
#define ADC_MAX      4096.0f         // 12-bit
#define I_OFFSET 2050
#define R_SHUNT      0.005f          // 5 mΩ
#define CUR_A_PER_LSB  (VREF / (GAIN_OPAMP * R_SHUNT * ADC_MAX))
float Iu,Iv,Iw;

float offset_Iu = 2048.0f;
float offset_Iv = 2048.0f;
float offset_Iw = 2048.0f;
uint8_t ICalibrate_flag = 0;

float test_cnt = 0;
static uint8_t speed_loop_divider = 0;
static uint8_t position_loop_divider = 0;


static inline float adc_to_cur(uint16_t code, float offset)
{
    return (((float)code - offset) * CUR_A_PER_LSB);
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        test_cnt++;
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
        uint32_t iu_code = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        uint32_t iv_code = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        uint32_t iw_code = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        if(ICalibrate_flag == 0)
        {
            static uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
            static int i = 0;
            if(i < 1000)
            {
                sum_u += iu_code; // 读取注入组原始值
                sum_v += iv_code;
                sum_w += iw_code;
                i++;
            }
            else
            {
                offset_Iu = (float)sum_u / 1000.0f;
                offset_Iv = (float)sum_v / 1000.0f;
                offset_Iw = (float)sum_w / 1000.0f;
                ICalibrate_flag = 1;  
            }
        }
        else
        {
            
            Iu = adc_to_cur(iu_code, offset_Iu);
            Iv = adc_to_cur(iv_code, offset_Iv);
            Iw = -Iu - Iv;
    //        Iw = adc_to_cur(iw_code);
            as5047p_read_dma_start();
            clarke1_t.Ia = Iu;
            clarke1_t.Ib = Iv;
            clarke1_t.Ic = Iw;
            clarke_calc(&clarke1_t);
            
            
            park1_t.theta = elec_theta;
            park1_t.Alpha = clarke1_t.Alpha;
            park1_t.Beta = clarke1_t.Beta;
            park_calc(&park1_t);
            
            Speed_Calc(mech_theta);
            
            speed_loop_divider++;
            if (speed_loop_divider >= 20)
            {
                speed_loop_divider = 0;
                
                // 设定目标速度 (可以在 main 里改)
                // pid_spd.TargetRPM = 1000.0f; 
                
                // 计算速度环 PID
                Speed_PID_Calc();
                
                // 【关键级联】：速度环输出 -> 赋值给 Iq 的目标值
                pid_iq.Ref = pid_spd.Out; 
            }
            
            position_loop_divider++;
            if (position_loop_divider >= 40)
            {
                Pos_Update_Sensor(&pid_pos, mech_theta);
                pid_spd.TargetRPM = Pos_PID_Calc(&pid_pos);
            }
            
            pid_id.Fdb = park1_t.D;
            pid_iq.Fdb = park1_t.Q;
            current_calc(&pid_id);
            current_calc(&pid_iq);
            ipark1_t.theta = elec_theta;
            ipark1_t.D = pid_id.Out;
            ipark1_t.Q = pid_iq.Out;
            
//            ipark1_t.D = 1.0f;
//            ipark1_t.Q = 0.0f;
//            if(test_cnt<90000)
//            {
//                park1_t.theta = 0;
//                ipark1_t.theta = 0;////////////////
//                pid_id.Ref = 0.6f;
//                pid_iq.Ref = 0.0f;
//            }
//            if(test_cnt>=90000)
//            {
//                freq_hz = 50;
//                pid_id.Ref = 0.0f;
//                pid_iq.Ref = 2.8f;
//                test_elec_theta += 2.0f * PI * freq_hz * Ts;
//                if (test_elec_theta > 2.0f * PI) test_elec_theta -= 2.0f * PI;
//                if(test_flag == 1)
//                {
//                    ipark1_t.theta = test_elec_theta;
//                    park1_t.theta = test_elec_theta;
//                }
//                if(test_cnt>100000)
//                {
//                    test_cnt = 90000;
//                }
//            }

            vofa_data[20] = test_cnt;
//            ipark1_t.theta = 0;////////////////
            ipark_calc(&ipark1_t);
            
            svpwm1_t.Ts = 4200;
            svpwm1_t.Udc = 12;
            svpwm1_t.Alpha = ipark1_t.Alpha;
            svpwm1_t.Beta = ipark1_t.Beta;
            svpwm_calc(&svpwm1_t);
            pwm_update(&svpwm1_t);
        }
        
        
        
        vofa_data[0] = ipark1_t.D;
        vofa_data[1] = ipark1_t.Q;
        vofa_data[2] = mech_theta;//(float)ipark1_t.theta; 
        vofa_data[3] = svpwm1_t.Alpha;
        vofa_data[4] = svpwm1_t.Beta;
        vofa_data[5] = svpwm1_t.CCR1;
        vofa_data[6] = svpwm1_t.CCR2;
        vofa_data[7] = svpwm1_t.CCR3;
        vofa_data[8] = svpwm1_t.sector;
        vofa_data[9] = Iu;
        vofa_data[10] = Iv;
        vofa_data[11] = Iw;
        vofa_data[12] = clarke1_t.Alpha;
        vofa_data[13] = clarke1_t.Beta;
        vofa_data[14] = park1_t.D;
        vofa_data[15] = park1_t.Q;
        vofa_data[16] = pid_id.Ref;
        vofa_data[17] = pid_iq.Ref;
//        vofa_data[18] = pid_id.Out;
//        vofa_data[19] = pid_iq.Out;
        vofa_data[18] = mech_theta;
        vofa_data[19] = elec_theta;
        vofa_data[21] = pid_spd.ActualRPM;


        
        

        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
    }
    
    
}


