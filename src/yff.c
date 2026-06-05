/*
 * ddd.c
 *
 *  Created on: 2026年3月4日
 *      Author: L.YF
 */


#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#include <math.h>

// --- 1. 算法结构体定义 ---
typedef struct {
    float Ref, Fdb, Err, ErrPre, Kp, Ki, Out, OutMax, OutMin, Ui;
} PI_CONTROLLER;

typedef struct {
    float Alpha, Beta, Cos, Sin, Ds, Qs;
} PARK_STRUCT;

// --- 2. 全局变量声明 ---
PARK_STRUCT park1;
PI_CONTROLLER pi_id = {0, 0, 0, 0, 0.1f, 0.005f, 0, 0.90f, -0.90f, 0}; // D轴电流环
PI_CONTROLLER pi_iq = {0, 0, 0, 0, 0.1f, 0.005f, 0, 0.90f, -0.90f, 0}; // Q轴电流环 (扭矩)

float ElecTheta = 0;
float Ia, Ib, Ic, Vdc;
Uint16 PwmPeriod = 4500; // 20kHz 频率

// --- 3. 函数原型 ---
void Init_DRV8313_GPIO(void);
void Init_ADC_Motor(void);
void Init_ePWM_Motor(void);
void Update_PI(PI_CONTROLLER *p);
interrupt void motor_control_isr(void);

// --- 4. 主函数 ---
void main(void) {
    InitSysCtrl();          // 初始化系统时钟 90MHz
    DINT;                   // 关全局中断
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT1 = &motor_control_isr; // 挂载中断
    EDIS;

    // 硬件初始化
    Init_DRV8313_GPIO();    // 开启 RESET, SLEEP, EN123
    Init_ADC_Motor();       // 配置 J7 的 A2, A3, A4, A6 采样
    Init_ePWM_Motor();      // 开启三相同步 PWM

    // 中断开启
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER |= M_INT1;
    EINT;                   // 全局中断开启
    ERTM;                   // 调试模式中断开启

    while(1) {
        // 主循环可监控变量
    }
}

// --- 5. 硬件详细配置 ---

// 驱动芯片使能 (针对你的自研板引脚)
void Init_DRV8313_GPIO(void) {
    EALLOW;
    // J5-48,49,50(EN), J6-56(RESET), J6-58(SLEEP)
    GpioCtrlRegs.GPBDIR.all |= 0x05070000;
    GpioCtrlRegs.GPBMUX2.all &= ~0x05070000;
    // 拉高所有控制线使芯片工作
    GpioDataRegs.GPBSET.all = 0x05070000;
    EDIS;
}

// ADC采样配置 (对应 J7 接口)
void Init_ADC_Motor(void) {
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCPWDN = 1;
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;
    // A2=IA, A3=IB, A4=IC, A6=Vdc
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 4;
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 6;
    // 全部由 ePWM1 SOCA 触发
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
    // SOC3 完成后抛出中断
    AdcRegs.INTSEL1N2.bit.INT1SEL = 3;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    EDIS;
}

// 三相 PWM 同步配置
void Init_ePWM_Motor(void) {
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 先锁定

    volatile struct EPWM_REGS *PWM[] = {&EPwm1Regs, &EPwm2Regs, &EPwm3Regs};
    int i;
    for(i=0; i<3; i++) {
        PWM[i]->TBPRD = PwmPeriod;
        PWM[i]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减模式
        PWM[i]->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // 开启死区
        PWM[i]->DBRED = 50; PWM[i]->DBFED = 50;      // 0.5us 死区
        PWM[i]->AQCTLA.bit.CAU = AQ_SET;
        PWM[i]->AQCTLA.bit.CAD = AQ_CLEAR;
    }

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 2; // 计数到 PRD 时采样
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // 释放同步
    EDIS;
}

// --- 6. 核心 FOC 中断 ---
interrupt void motor_control_isr(void) {
    // A. 采样反馈 ( INA199 50倍增益)
    Ia = (float)(AdcResult.ADCRESULT0 - 2048) * 0.0016113f;
    Ib = (float)(AdcResult.ADCRESULT1 - 2048) * 0.0016113f;
    Ic = (float)(AdcResult.ADCRESULT2 - 2048) * 0.0016113f;

    // 关键点：读取触发中断的 SOC3 结果，确保硬件状态机闭环
    Vdc = (float)AdcResult.ADCRESULT3 * 0.0008056f; // 3.3/4096

    // B. 角度计算 (模拟强制旋转)
    ElecTheta += 0.001f; // 低速测试
    if(ElecTheta > 6.283185f) ElecTheta = 0;
    park1.Cos = cosf(ElecTheta);
    park1.Sin = sinf(ElecTheta);

    // C. 电流环 PI 运算 (此处保持你之前的逻辑)
    pi_id.Fdb = 0;
    pi_iq.Fdb = 0;
    Update_PI(&pi_id);
    Update_PI(&pi_iq);

    // D. 逆变换与占空比更新
    float Valpha = -pi_iq.Out * park1.Sin;
    float Vbeta  =  pi_iq.Out * park1.Cos;

    // 更新三相占空比 (对应 EPwm1, 2, 3)
    EPwm1Regs.CMPA.half.CMPA = (Uint16)(PwmPeriod * (Valpha * 0.4f + 0.5f));
    EPwm2Regs.CMPA.half.CMPA = (Uint16)(PwmPeriod * ((-0.5f * Valpha + 0.866f * Vbeta) * 0.4f + 0.5f));
    EPwm3Regs.CMPA.half.CMPA = (Uint16)(PwmPeriod * ((-0.5f * Valpha - 0.866f * Vbeta) * 0.4f + 0.5f));

    // --- 核心：必须加在函数末尾的清除逻辑 ---
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   // 清除 ADC 中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // 响应 PIE 第 1 组中断
}

// PI 算法实现
void Update_PI(PI_CONTROLLER *p) {
    p->Err = p->Ref - p->Fdb;
    p->Ui += p->Ki * p->Err;
    if(p->Ui > p->OutMax) p->Ui = p->OutMax;
    else if(p->Ui < p->OutMin) p->Ui = p->OutMin;
    p->Out = p->Kp * p->Err + p->Ui;
    if(p->Out > p->OutMax) p->Out = p->OutMax;
    else if(p->Out < p->OutMin) p->Out = p->OutMin;
}
