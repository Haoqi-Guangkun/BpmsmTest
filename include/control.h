/*
 * control.h
 *
 *  Created on: 2026��3��16��
 *      Author: L.YF
 */

#ifndef INCLUDE_CONTROL_H_
#define INCLUDE_CONTROL_H_

#include "DSP2833x_Device.h"
#include "clark.h"
#include "park.h"
#include "ipark.h"
#include "pid_reg.h"
#include "UFC.h"

// �궨��
#define PI 3.14159265358979f
#define PI2 6.28318530717958f
#define INV_PI2 0.159154943091896f			 // 1/(2*pi)
#define ISR_FREQ    10000.0f                 // 10kHz
#define POLE_PAIRS          1.0f             // ת�����鼫����
#define CURRENT1_SCALE      -2.5f            // ת�ص�ѹת����ϵ����10/4, I = (V - Offset) * SCALE
#define CURRENT2_SCALE      -3.3333333333f   // ������ѹת����ϵ����10/3
#define VOLTAGE_SCALE       10.0f            // ĸ�ߵ�ѹת��ϵ����10/1
#define SENSOR_POS_SCALE    0.0004f          // ������������ѹתλ��(mm)ϵ����������������

// ����е�����ֲ������� (��-��������ʹ��)
#define MOTOR_L1Q    0.0019f       // ת������ q ���� (H)
#define MOTOR_PHI_F  0.0252f       // �������Ч���� (Wb)
#define MOTOR_KM     311.0f        // ���������ϵ�� Km
// Ԥ�ȼ��������ƽ������ʡ DSP ����
#define MOTOR_PHI_F_SQ  (MOTOR_PHI_F * MOTOR_PHI_F)

// ����һ���ṹ��������ADCת�����������
typedef struct
{
    // --- ADCINA Group (0-7) ---
    float Filtered_U;       // ADCINA0
    float Filtered_V;       // ADCINA1
    float Filtered_W;       // ADCINA2
    float Filtered_A;       // ADCINA3
    float Filtered_B;       // ADCINA4
    float Filtered_C;       // ADCINA5
    float ADC_VT;           // ADCINA6 (ĸ�ߵ�ѹ)
    float ADC_VS;           // ADCINA7

    // --- ADCINB Group (0-7) ---
    float Hall_A;           // ADCINB0
    float Hall_B;           // ADCINB1
    float Hall_C;           // ADCINB2
    float Eddy_C;           // ADCINB3 (����������)
    float Eddy_B;           // ADCINB4
    float Eddy_A;           // ADCINB5

} AdcData_t;

// ȫ�ֱ������������ⲿ����
extern AdcData_t g_AdcData;

// ����ԭ������
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
