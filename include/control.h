/*
 * control.h
 *
 *  Created on: 2026年3月16日
 *      Author: L.YF
 */

#ifndef INCLUDE_CONTROL_H_
#define INCLUDE_CONTROL_H_

#include "DSP2833x_Device.h"
#include "clark.h"
#include "park.h"
#include "ipark.h"
#include "pid_reg.h"

// 宏定义
#define PI 3.14159265358979f
#define PI2 6.28318530717958f
#define INV_PI2 0.159154943091896f
#define ISR_FREQ    10000.0f                 // 10kHz
#define POLE_PAIRS          1.0f             // 转矩绕组极对数
#define CURRENT1_SCALE      -2.5f            // 转矩电压转电流系数：10/4, I = (V - Offset) * SCALE
#define CURRENT2_SCALE      -3.3333333333f   // 悬浮电压转电流系数：10/3
#define VOLTAGE_SCALE       10.0f            // 母线电压转换系数：10/1
#define SENSOR_POS_SCALE    0.0004f          // 涡流传感器电压转位移(mm)系数，待调整！！！

// 无轴承电机部分参数定义 (力-电流解耦使用)
#define MOTOR_L1Q    0.0019f       // 转矩绕组 q 轴电感 (H)
#define MOTOR_PHI_F  0.0252f       // 永磁体等效磁链 (Wb)
#define MOTOR_KM     311.0f        // 悬浮力耦合系数 Km
// 预先计算磁链的平方，节省 DSP 算力
#define MOTOR_PHI_F_SQ  (MOTOR_PHI_F * MOTOR_PHI_F)

// 定义一个结构体来保存ADC转换后的物理量
typedef struct
{
    // --- ADCINA Group (0-7) ---
    float Filtered_U;       // ADCINA0
    float Filtered_V;       // ADCINA1
    float Filtered_W;       // ADCINA2
    float Filtered_A;       // ADCINA3
    float Filtered_B;       // ADCINA4
    float Filtered_C;       // ADCINA5
    float ADC_VT;           // ADCINA6 (母线电压)
    float ADC_VS;           // ADCINA7

    // --- ADCINB Group (0-7) ---
    float Hall_A;           // ADCINB0
    float Hall_B;           // ADCINB1
    float Hall_C;           // ADCINB2
    float Eddy_C;           // ADCINB3 (涡流传感器)
    float Eddy_B;           // ADCINB4
    float Eddy_A;           // ADCINB5

} AdcData_t;

// 全局变量声明，供外部调用
extern AdcData_t g_AdcData;

// 函数原型声明
float one_order_LPF(float cutoff_fre, float input, Uint16 i);
float Speed_Filter(float input);
void Read_Adc(void);
void Convert_Adc_Data(void);
void Calculate_Feedback(void);
void Control_Suspension(void);
void Control_Torque(void);
void Stop_Suspension(void);
void Stop_Torque(void);
void Stop_System(void);
void Resume_System(void);
void Control_Align(void);

#endif /* INCLUDE_CONTROL_H_ */
