/*
 * main.c - SVPWM Implementation for TMS320F28335
 * 功能：输出三相 SVPWM 波形 (EPWM1, EPWM2, EPWM3)
 * 特性：包含死区配置 (Dead Band)，互补输出
 * 可调参数：通过 CCS Expressions 窗口修改变量 "M" 即可改变电压幅值（占空比）
 */

#include "DSP2833x_Device.h"     // Headerfile Include File
#include "DSP2833x_Examples.h"   // Examples Include File
#include <math.h>

// --- 定义参数 ---
#define PWM_PRD     7500         // PWM周期值 (150MHz / (2 * 10kHz) / 1分频 = 7500)
                                 // 假设开关频率 10kHz，中心对齐
#define DEAD_TIME   300          // 死区时间 (参考 PDF 文档值: 300 TBCLKs)
                                 // 时间计算: 300 * (1/150MHz) = 2us (不分频)

#define PI          3.1415926f
#define ISR_FREQ    10000.0f     // 中断频率 = 开关频率
#define DELTA_THETA (2 * PI * 50 / ISR_FREQ) // 50Hz 正弦波每次中断增加的角度

// --- 全局变量 (在 Expressions 窗口观察/修改) ---
float M = 0.8f;           // 调制比 (0.0 ~ 1.0)
float Theta = 0.0f;       // 当前角度
float Ualpha, Ubeta;      // Alpha-Beta 坐标系电压
float Ta, Tb, Tc;         // 三相比较值
Uint16 Sector = 0;        // 当前扇区

// --- 函数声明 ---
void InitEPwm_SVPWM(void);
interrupt void epwm1_isr(void);

void main(void)
{
    // 1. 系统初始化
    InitSysCtrl();

    // 2. GPIO 初始化 (配置 EPWM1-3 为 PWM 功能)
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    // 3. 中断初始化
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // 4. 映射中断函数
    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;

    // 5. 初始化 PWM 模块
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 关闭 PWM 时钟以便同步配置
    EDIS;

    InitEPwm_SVPWM();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // 开启 PWM 时钟
    EDIS;

    // 6. 开启中断
    IER |= M_INT3;            // 开启 CPU INT3 (对应 EPWM)
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // 开启 PIE 组3 的第1个 (EPWM1)

    EINT;   // 开总中断
    ERTM;   // 开实时中断

    // 7. 主循环
    for(;;)
    {
        // 此处可以放置非实时逻辑
    }
}

// --- PWM 初始化配置 (包含死区配置) ---
void InitEPwm_SVPWM(void)
{
    // ================= EPWM1 设置 =================
    EPwm1Regs.TBPRD = PWM_PRD;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 中心对齐
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // 不分频

    // 动作限定 (AQ):
    // 向上计数到 CMPA 清零 (输出低)，向下计数到 CMPA 置位 (输出高)
    // 结果：中间是高电平脉冲 (Active High)
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    // --- 死区配置 (参考 PDF Page 3) ---
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;       // 输入源选择: A和B的延时都基于 EPWMxA
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;    // 极性选择: 高电平互补模式 (Active High Complementary)
                                                 // EPWMxA: 直通 (含上升沿延时)
                                                 // EPWMxB: 反相 (含下降沿延时)
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // 输出使能: 使能上升沿(RED)和下降沿(FED)延时
    EPwm1Regs.DBFED = DEAD_TIME;                 // 下降沿延时 (Falling Edge Delay)
    EPwm1Regs.DBRED = DEAD_TIME;                 // 上升沿延时 (Rising Edge Delay)

    // 中断配置
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // 计数器回零时触发中断
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // 使能中断
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;          // 每次事件都触发

    // ================= EPWM2 设置 =================
    EPwm2Regs.TBPRD = PWM_PRD;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    // --- 死区配置 (参考 PDF Page 4) ---
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;        //互补模式。EPWMxA 是高电平有效（控制上桥），EPWMxB 是 EPWMxA 的反相（控制下桥），并自动插入死区。
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBFED = DEAD_TIME;
    EPwm2Regs.DBRED = DEAD_TIME;

    // ================= EPWM3 设置 =================
    EPwm3Regs.TBPRD = PWM_PRD;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

    // --- 死区配置 (参考 PDF Page 5) ---
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBFED = DEAD_TIME;
    EPwm3Regs.DBRED = DEAD_TIME;
}

// --- SVPWM 核心算法中断 ---
interrupt void epwm1_isr(void)
{
    float X, Y, Z;
    float t1, t2;
    float Va, Vb, Vc;

    // 1. 角度更新 (50Hz 旋转)
    Theta += DELTA_THETA;
    if (Theta > 2*PI) Theta -= 2*PI;

    // 2. 计算 Ualpha, Ubeta
    Ualpha = M * cos(Theta);
    Ubeta  = M * sin(Theta);

    // 3. 扇区判断
    Va = Ubeta;
    Vb = 0.866025f * Ualpha - 0.5f * Ubeta;
    Vc = -0.866025f * Ualpha - 0.5f * Ubeta;

    Sector = 0;
    if (Va > 0) Sector += 1;
    if (Vb > 0) Sector += 2;
    if (Vc > 0) Sector += 4;

    // 4. 计算基本矢量作用时间
    X = Ubeta;
    Y = 0.866025f * Ualpha + 0.5f * Ubeta;
    Z = -0.866025f * Ualpha + 0.5f * Ubeta;

    switch (Sector)
    {
        case 3: // I
            t1 = -Z; t2 = X; break;
        case 1: // II
            t1 = Z; t2 = Y; break;
        case 5: // III
            t1 = X; t2 = -Y; break;
        case 4: // IV
            t1 = -X; t2 = Z; break;
        case 6: // V
            t1 = -Y; t2 = -Z; break;
        case 2: // VI
            t1 = Y; t2 = -X; break;
        default:
            t1 = 0; t2 = 0; break;
    }

    // 简单限幅
    if((t1+t2) > 1.0f) { t1 /= (t1+t2); t2 /= (t1+t2); }

    // 计算导通时刻 (CMPA 值)
    float T_on1 = (1.0f - t1 - t2) * 0.5f * PWM_PRD;
    float T_on2 = T_on1 + t1 * PWM_PRD;
    float T_on3 = T_on2 + t2 * PWM_PRD;

    // 5. 扇区分配 (注意：死区模式下，A通道为上桥，B通道自动互补生成下桥)
    switch (Sector)
    {
        case 3: // I
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on3;
            break;
        case 1: // II
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on3;
            break;
        case 5: // III
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on2;
            break;
        case 4: // IV
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on1;
            break;
        case 6: // V
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on1;
            break;
        case 2: // VI
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on2;
            break;
    }

    // 清除中断标志位
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
