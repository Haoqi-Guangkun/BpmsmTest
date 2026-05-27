/*
 * 此代码可实现电机部分的开环转动测试20260306
 * 流程：设定Ud/Uq -> 反Park变换 -> SVPWM输出
 */

#include "Adc.h"
#include "Svpwm.h"
#include <math.h>
#include "DSP2833x_Examples.h"

// 定义全局结构体实例
AdcData_t g_AdcData = {0};

// --- 变量定义 ---
float Ud_Ref = 0.0f, Uq_Ref = 0.6f;
float ElecTheta = 0.0f;
float Ualpha = 0.0f, Ubeta = 0.0f;

// V/f 开环参数
int Pole_Pairs = 3;
float Target_Speed_RPM = 1000.0f;
float Current_Speed_RPM = 0.0f;
float Speed_Ramp_Step = 0.01f;
float V_F_Ratio = 0.0005f;
float Voltage_Boost = 0.02f;   //若电机只响不转，再一点点往上加
float Freq_Hz = 0.0f;
float STEP_THETA = 0.0f;

// 状态控制
volatile Uint16 EnDrive = 0, EnSystem = 0;
volatile Uint16 IsrTicker_adc1 = 0, IsrTicker_adc2 = 0;

// 零偏滤波参数 (滑动平均滤波)
float K1 = 0.998f;
float K2 = 0.001999f; // 凑整 K1+K2=1

volatile float offsetA = 0, offsetB = 0, offsetC = 0, offsetU = 0, offsetV = 0, offsetW = 0;
volatile float Iu = 0, Iv = 0, Iw = 0, Ia = 0, Ib = 0, Ic = 0;

/*
 * 函数: Init_Adc
 * 功能: 初始化 ADC 模块，配置为 16 通道级联模式
 */
void Init_Adc_Module(void)
{
    // 1. 基础初始化 (开启时钟、校准等，使用 TI 官方函数)
    InitAdc();
    EALLOW;

    // 2. 配置 ADCTRL1 寄存器
    // ACQ_PS = 0xF (15+1=16个ADCCLK周期采样窗口)，防止信号源阻抗大导致采样不准
    // CONT_RUN = 0 (非连续模式，触发一次采一轮)
    // SEQ_CASC = 1 (级联模式！SEQ1 和 SEQ2 合并成一个 16 步的排序器)
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0F;        // 采样窗口
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;         // 级联
    AdcRegs.ADCTRL1.bit.CPS = 0;              // 预分频 /1
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;         // 单次模式
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;         // 禁用序列覆盖，使用硬件触发

    // 3.添加/修改 ADCTRL2 配置 ---
    AdcRegs.ADCTRL2.all = 0x0000;              // 先清零
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;    // 允许 EPWM_SOCA 信号触发 SEQ1 级联排序器
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;      // 使能 SEQ1 转换完成中断 (产生 ADCINT 信号)
//    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0; // 每次SEQ1完成都触发中断
//    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;     // 复位SEQ1排序器

    // 4. 配置 ADCTRL3 寄存器
    // 假设 HSPCLK = 75MHz (150MHz/2)，ADCCLKPS=3 (分频系数6) -> ADC时钟 = 12.5MHz
    // F28335 ADC 时钟最大不能超过 25MHz
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;  // ADC时钟分频(75MHz/6=12.5MHz)
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0; // 顺序采样模式

    // 5. 配置最大转换通道数 (MAX_CONV)
    AdcRegs.ADCMAXCONV.all = 0x000F;    // 级联模式，最大转换16个

    // 6. 配置通道映射 (ChSel)
    // 按顺序采集所有通道
    // ADCINA0~7 对应通道 0~7，ADCINB0~7 对应通道 8~15
    // CONV00 ~ CONV016
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0; // ADCINA0 -> Filtered_U
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1; // ADCINA1 -> Filtered_V
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2; // ADCINA2 -> Filtered_W
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 3; // ADCINA3 -> Filtered_A
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 4; // ADCINA4 -> Filtered_B
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 5; // ADCINA5 -> Filtered_C
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 6; // ADCINA6 -> VT
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 7; // ADCINA7 -> VS
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 8;  // ADCINB0 -> Hall_A
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 9;  // ADCINB1 -> Hall_B
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 10; // ADCINB2 -> Hall_C
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 11; // ADCINB3 -> Eddy_C
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 12; // ADCINB4 -> Eddy_B
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 13; // ADCINB5 -> Eddy_A
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 14; // ADCINB6 -> R33
    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 15; // ADCINB7 -> R34

    EDIS;
}

// ============================================================================
// 中断 1: 零偏校准中断
// ============================================================================
interrupt void adc_offset_isr(void)
{
    // 1、先根据AD寄存器数值得到电压，再根据电压转换为电流；注意检测相关的零偏数值，观察系统的零偏；
    Read_Adc();

    IsrTicker_adc1++;

    // 2、等待前 5000 次周期，让硬件运算放大器和ADC完全稳定
    if (IsrTicker_adc1 >= 5000)
    {
        // 采用一阶低通滤波计算零偏
        offsetU = K1 * offsetU + K2 * g_AdcData.Filtered_U;  //Phase U offset
        offsetV = K1 * offsetV + K2 * g_AdcData.Filtered_V;  //Phase V offset
        offsetW = K1 * offsetW + K2 * g_AdcData.Filtered_W;  //Phase W offset
        offsetA = K1 * offsetA + K2 * g_AdcData.Filtered_A;  //Phase A offset
        offsetB = K1 * offsetB + K2 * g_AdcData.Filtered_B;  //Phase B offset
        offsetC = K1 * offsetC + K2 * g_AdcData.Filtered_C;  //Phase C offset
        // 此处可加上传感器和母线电压的零偏计算
        // offsetD = K1 * offsetD + K2 * g_AdcData.ADC_VT;
    }

    // 3、校准完成 (执行到第 20000 次中断，也就是 2 秒钟 @10kHz)
    if (IsrTicker_adc1 > 20000)
    {
        EALLOW;
        // 将 ADC 中断向量动态切换为主控制程序！
        PieVectTable.ADCINT = &adc_main_isr;
        EDIS;

        // 允许系统和驱动启动
        EnSystem = 1;
        EnDrive = 1;
    }

    // 4、清除中断标志，准备下一次采样
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// ============================================================================
// 中断 2: 主控制中断 (校准完成后，一直进这个中断)
// ============================================================================
interrupt void adc_main_isr(void)
{
    // 1. 读取 ADC 数据
    Read_Adc();
    IsrTicker_adc2++;

    // 2. 减去零偏，并将电压转换成电流
    Iu = (g_AdcData.Filtered_U - offsetU)*1;   // ×电压-电流转换系数（采样电阻+运放系数）
    Iv = (g_AdcData.Filtered_V - offsetV)*1;
    Iw = (g_AdcData.Filtered_W - offsetW)*1;
    Ia = (g_AdcData.Filtered_A - offsetA)*1;
    Ib = (g_AdcData.Filtered_B - offsetB)*1;
    Ic = (g_AdcData.Filtered_C - offsetC)*1;
    //。。。。。。。。。。。。。其他通道转换

    // 3. 系统启停控制
    if (EnSystem == 1 && EnDrive == 1)
    {
        // === 解除 TZ 锁定 (若之前触发过) ===
        EALLOW;
        if(EPwm1Regs.TZFLG.bit.OST == 1)
        {
            EPwm1Regs.TZCLR.bit.OST = 1; // 清除标志，恢复PWM输出
            EPwm2Regs.TZCLR.bit.OST = 1;
            EPwm3Regs.TZCLR.bit.OST = 1;
        }
        EDIS;

        // --- 正常运行: V/f 开环控制 ---
        if(Current_Speed_RPM < Target_Speed_RPM)
        {
            Current_Speed_RPM += Speed_Ramp_Step;
            if(Current_Speed_RPM > Target_Speed_RPM)
                Current_Speed_RPM = Target_Speed_RPM;
        }
        else if (Current_Speed_RPM > Target_Speed_RPM)
        {
            Current_Speed_RPM -= Speed_Ramp_Step;
            if(Current_Speed_RPM < Target_Speed_RPM)
                Current_Speed_RPM = Target_Speed_RPM;
        }

        //更新电角度（开环强行积分累加）换算频率与角度步长
        Freq_Hz = (Current_Speed_RPM * Pole_Pairs) / 60.0f;
        STEP_THETA = 2.0f * PI * Freq_Hz / ISR_FREQ;

        //更新电角度（开环强行积分累加）
        ElecTheta += STEP_THETA;
        if(ElecTheta > 2.0f * PI) ElecTheta -= 2.0f * PI;
        if(ElecTheta < 0.0f)      ElecTheta += 2.0f * PI;

        //计算电压幅值（V/f曲线给定）
        Ud_Ref = 0.0f;
        Uq_Ref = (Current_Speed_RPM * V_F_Ratio) + Voltage_Boost;

        //幅值限幅保护
        if(Uq_Ref > 1.0f) Uq_Ref = 1.0f;
        if(Uq_Ref < 0.0f) Uq_Ref = 0.0f;

        //反Park变换
        float sin_val = sinf(ElecTheta);
        float cos_val = cosf(ElecTheta);
        Ualpha = Ud_Ref * cos_val - Uq_Ref * sin_val;
        Ubeta  = Ud_Ref * sin_val + Uq_Ref * cos_val;

        // 使用SWVPWM发波
        Update_Svpwm(Ualpha, Ubeta);
        // Update_Suspension_PWM(Ualpha, Ubeta); // 悬浮
    }
    else
    {
        // --- 停机状态: 安全处理 ---
        //Current_Speed_RPM = 0.0f; // 转速清零
        // 软件停机: 占空比设为 0 或者 50% (取决于你的硬件死区和驱动电路)
        Update_Svpwm(0.0f, 0.0f);

        // 硬件停机: 若需彻底关断，参考PDF利用Trip Zone (TZ) 强行封锁 PWM
        EALLOW;
        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZFRC.bit.OST = 1;
        EDIS;
    }

    // 4. 清除中断标志
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void Read_Adc(void)
{
    // 读取结果并转换为电压值 (或实际物理量)
    // 公式: Voltage = (Digital_Value * 3.0V) / 4096.0
    // 注意: F28335 的 ADCRESULTx 是左对齐的，结果在高12位，所以要 >> 4
    const float k = 3.0f / 4096.0f;  //数字量转电压系数

    g_AdcData.Filtered_U = (float)(AdcRegs.ADCRESULT0 >> 4) * k;
    g_AdcData.Filtered_V = (float)(AdcRegs.ADCRESULT1 >> 4) * k;
    g_AdcData.Filtered_W = (float)(AdcRegs.ADCRESULT2 >> 4) * k;
    g_AdcData.Filtered_A = (float)(AdcRegs.ADCRESULT3 >> 4) * k;
    g_AdcData.Filtered_B = (float)(AdcRegs.ADCRESULT4 >> 4) * k;
    g_AdcData.Filtered_C = (float)(AdcRegs.ADCRESULT5 >> 4) * k;
    g_AdcData.ADC_VT     = (float)(AdcRegs.ADCRESULT6 >> 4) * k;
    g_AdcData.ADC_VS     = (float)(AdcRegs.ADCRESULT7 >> 4) * k;
    g_AdcData.Hall_A     = (float)(AdcRegs.ADCRESULT8 >> 4) * k;
    g_AdcData.Hall_B     = (float)(AdcRegs.ADCRESULT9 >> 4) * k;
    g_AdcData.Hall_C     = (float)(AdcRegs.ADCRESULT10 >> 4) * k;
    g_AdcData.Eddy_C     = (float)(AdcRegs.ADCRESULT11 >> 4) * k;
    g_AdcData.Eddy_B     = (float)(AdcRegs.ADCRESULT12 >> 4) * k;
    g_AdcData.Eddy_A     = (float)(AdcRegs.ADCRESULT13 >> 4) * k;
    g_AdcData.R33_Res    = (float)(AdcRegs.ADCRESULT14 >> 4) * k;
    g_AdcData.R34_Res    = (float)(AdcRegs.ADCRESULT15 >> 4) * k;

}
