/*
 * Adc1.c
 *
 *  Created on: 2026年3月8日
 *      Author: L.YF
 */

/*
 * 此代码完成的功能：ADC初始化
 */

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "Adc.h"

/*
 * 函数: Init_Adc
 * 功能: 初始化 ADC 模块，配置为 16 通道级联模式
 */
void Init_Adc(void)
{
    // 1. 基础初始化 (开启时钟、校准等，使用 TI 官方函数)
    InitAdc();
    EALLOW;

    // 开启外部参考电压模式 (ADCREFIN引脚接 REF3020 提供的 2.048V)，接主控板时开启！否则默认内部参考！！！
    AdcRegs.ADCREFSEL.bit.REF_SEL = 1;  // 1: 外部参考, 0: 内部参考

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
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;          // 复位SEQ1排序器，确保从 CONV00 开始转换

    // 4. 配置 ADCTRL3 寄存器
    // HISPCP = 1，HSPCLK = SYSCLKOUT/(HISPCP×2)
    // HSPCLK = 75MHz (150MHz/2)，ADCCLKPS=3 (分频系数6) -> ADC时钟 = 12.5MHz
    // F28335 ADC 时钟最大不能超过 25MHz
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;  // ADC时钟分频(75MHz/6=12.5MHz)
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0; // 顺序采样模式

    // 5. 配置最大转换通道数 (MAX_CONV)
    AdcRegs.ADCMAXCONV.all = 0x000D;    // 级联模式，最大转换14个

    // 6. 配置通道映射 (ChSel)
    // 按顺序采集所有通道
    // ADCINA0~7 对应通道 0~7，ADCINB0~7 对应通道 8~15
    // CONV00 ~ CONV016
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;  // ADCINA0 -> Filtered_U
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;  // ADCINA1 -> Filtered_V
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2;  // ADCINA2 -> Filtered_W
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 3;  // ADCINA3 -> Filtered_A
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 4;  // ADCINA4 -> Filtered_B
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 5;  // ADCINA5 -> Filtered_C
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 6;  // ADCINA6 -> VT
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 7;  // ADCINA7 -> VS
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 8;  // ADCINB0 -> Hall_A
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 9;  // ADCINB1 -> Hall_B
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 10; // ADCINB2 -> Hall_C
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 11; // ADCINB3 -> Eddy_C
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 12; // ADCINB4 -> Eddy_B
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 13; // ADCINB5 -> Eddy_A
//    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 14; // ADCINB6 -> R33
//    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 15; // ADCINB7 -> R34

//    AdcRegs.ADCOFFTRIM.bit.OFFSET_TRIM = 0; //采样结果的偏置为0

    EDIS;
}

/*
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
//    g_AdcData.R33_Res    = (float)(AdcRegs.ADCRESULT14 >> 4) * k;      //闲置ADC管脚
//    g_AdcData.R34_Res    = (float)(AdcRegs.ADCRESULT15 >> 4) * k;      //闲置ADC管脚
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

        // 线性霍尔零偏 (在转子静止时进行！)
        Hall_OffsetA = K1 * Hall_OffsetA + K2 * g_AdcData.Hall_A;
        Hall_OffsetB = K1 * Hall_OffsetB + K2 * g_AdcData.Hall_B;
        Hall_OffsetC = K1 * Hall_OffsetC + K2 * g_AdcData.Hall_C;

        // 电涡流位移传感器零偏 (注意：需要在转子静止且利用辅助轴承机械居中时进行校准)，后续考虑细节！
        // 此处假设 Eddy_A 测 X 轴，Eddy_B 测 Y 轴，后续要根据三个位移传感器进行位移解算！
        offset_EddyX = K1 * offset_EddyX + K2 * g_AdcData.Eddy_A;
        offset_EddyY = K1 * offset_EddyY + K2 * g_AdcData.Eddy_B;

        // 母线电压的零偏计算
//         offsetVT = K1 * offsetVT + K2 * g_AdcData.ADC_VT;
//         offsetVS = K1 * offsetVS + K2 * g_AdcData.ADC_VS;

    }

    // 3、校准完成 (执行到第 20000 次中断，也就是 2 秒钟 @10kHz)
    if (IsrTicker_adc1 > 20000)
    {
        EALLOW;
        // 将 ADC 中断向量动态切换为主控制程序！
        PieVectTable.ADCINT = &adc_main_isr;
        EDIS;
        // 允许系统启动
        EnSystem = 1;
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
    // ========================================================================
    // 步骤 0: 读取最新的 ADC 采样值
    // ========================================================================
    Read_Adc();

    // ========================================================================
    // 步骤 1: 基础数据换算 (去除零偏，转为实际安培/毫米)
    // ========================================================================
    // === 2. 采集电流、母线电压、传感器信号 ===
    // 转矩三相电流
    Iu = (g_AdcData.Filtered_U - offsetU) * CURRENT1_SCALE;   //CURRENT1_SCALE：电压-电流转换系数（采样电阻+运放系数）
    Iv = (g_AdcData.Filtered_V - offsetV) * CURRENT1_SCALE;
    Iw = (g_AdcData.Filtered_W - offsetW) * CURRENT1_SCALE;
    // 悬浮三相电流
    Ia = (g_AdcData.Filtered_A - offsetA) * CURRENT2_SCALE;
    Ib = (g_AdcData.Filtered_B - offsetB) * CURRENT2_SCALE;
    Ic = (g_AdcData.Filtered_C - offsetC) * CURRENT2_SCALE;
    //。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。其他通道转换（母线电压）
    // 计算转子中心实际位移反馈（要根据电路确定解算方法！）
    Pos_X_Fdb = (g_AdcData.Eddy_A - offset_EddyX) * SENSOR_POS_SCALE;
    Pos_Y_Fdb = (g_AdcData.Eddy_B - offset_EddyY) * SENSOR_POS_SCALE;

    // ========================================================================
    // 步骤 2: 硬件与运行安全监控 (保护优先级最高)
    // ========================================================================
    // 检测越限：位移超限（快撞到保护轴承了）或 电流超限（过流）  去掉？？
    if (fabsf(Pos_X_Fdb) > POS_FAULT_THRES || fabsf(Pos_Y_Fdb) > POS_FAULT_THRES ||
        fabsf(Iu) > CUR_FAULT_THRES || fabsf(Iv) > CUR_FAULT_THRES ||
        fabsf(Ia) > CUR_FAULT_THRES || fabsf(Ib) > CUR_FAULT_THRES )
    {
        Fault_Flag = 1; // 触发故障标志
        EnSystem = 0;   // 强制切断系统总使能
    }

    // ========================================================================
    // 步骤 3: 核心状态机控制
    // ========================================================================
    //1、系统使能判断与保护
    if (EnSystem == 1 && Fault_Flag == 0)
    {
        // --------------------------------------------------------------------
        // 3.1 悬浮系统控制 (只要 EnSystem=1 就立刻工作)
        // --------------------------------------------------------------------
        if (EnSus_Flag == 0)
        {
            EALLOW;
            // 悬浮系统由 EPwm4, 5, 6 控制，解除TZ 锁定 (若之前触发过)，恢复运行
            EPwm4Regs.TZCLR.bit.OST = 1;
            EPwm5Regs.TZCLR.bit.OST = 1;
            EPwm6Regs.TZCLR.bit.OST = 1;
            EDIS;
            // 清理悬浮PID积分器
            pid_x.Ui = 0; pid_y.Ui = 0;
            gSuspCurrD_PID.Ui = 0; gSuspCurrQ_PID.Ui = 0;
            EnSus_Flag = 1;
        }

        // --- 悬浮闭环计算 ---
        // 1. 悬浮电流 Clark 变换：Ia, Ib, Ic ---> Ialpha, Ibeta
        clark2.As = Ia;
        clark2.Bs = Ib;
        clark2.calc(&clark2);

        // ---------------------------------------------------------------------
        // 获取转矩 Q 轴电流，以进行力-电流转换
        // ---------------------------------------------------------------------
        // 转矩电流 Clark 变换
        clark1.As = Iu;
        clark1.Bs = Iv;
        clark1.calc(&clark1);
        // 转矩电流 Park 变换
        park1.Alpha = clark1.Alpha;
        park1.Beta = clark1.Beta;
        park1.Cos = cosf(ElecTheta_rad);
        park1.Sin = sinf(ElecTheta_rad);
        park1.calc(&park1);

        // 2. 位移环 PID：输入给定 x*, y*, 反馈 x, y，输出力给定 Fx*, Fy*)
        pid_x.Ref = Pos_X_Ref;
        pid_x.Fdb = Pos_X_Fdb;
        pid_x.calc(&pid_x);
        Force_X_Ref = pid_x.Out;  //得到 Fx*

        pid_y.Ref = Pos_Y_Ref;
        pid_y.Fdb = Pos_Y_Fdb;
        pid_y.calc(&pid_y);
        Force_Y_Ref = pid_y.Out;  //得到 Fy*

        // 3. 力-电流解耦 (公式)，需根据所用模型修改！！！！！
        // 获取转矩绕组 q 轴电流（目前所用为根据杨雄论文搭建的仿真模型），park1.Qs 存储的是当前转矩电流反馈值
        float I1q = park1.Qs;
        // 计算中间变量 u(3) = L1q * I1q
        float u3 = MOTOR_L1Q * I1q;
        // 计算公式公共分母: Km * (Phi_f^2 + u(3)^2)
        float denom = MOTOR_KM * (MOTOR_PHI_F_SQ + u3 * u3);
        // 【硬件保护】防止分母意外变为 0 导致单片机除零死机复位
        if(denom < 1e-6f && denom > -1e-6f) denom = 1e-6f;

        // 求分母倒数 (由于浮点乘法指令通常比除法指令快得多，优化DSP执行效率)
        float inv_denom = 1.0f / denom;
        // i2d* = (u(1)*Phi_f - u(2)*u(3)) / denom
        i_Bd_ref = (Force_X_Ref * MOTOR_PHI_F - Force_Y_Ref * u3) * inv_denom;
        // i2q* = (u(1)*u(3) + u(2)*Phi_f) / denom
        i_Bq_ref = (Force_X_Ref * u3 + Force_Y_Ref * MOTOR_PHI_F) * inv_denom;

        // 4. 获取悬浮旋转角 (基于机械位置)
        // 通过线性霍尔提取绝对角度
        // 减去静态偏置，得到以 0 为中心的交流正弦波
        float ha = g_AdcData.Hall_A - Hall_OffsetA;
        float hb = g_AdcData.Hall_B - Hall_OffsetB;
        float hc = g_AdcData.Hall_C - Hall_OffsetC;

        // 霍尔信号 Clark 变换 (提取直角坐标分量)，利用三相公式可以消除共模噪声，精度更高
        float h_alpha = 0.666666667f * (ha - 0.5f * hb - 0.5f * hc);
        float h_beta  = 0.577350269f * (hb - hc);

        ElecTheta_rad = atan2f(h_beta, h_alpha); // 求解绝对电角度，范围: -PI ~ PI

        // 转换为标幺值 0~1.0
        ElecTheta_pu = ElecTheta_rad * INV_PI2;
        if(ElecTheta_pu < 0.0f) ElecTheta_pu += 1.0f;

        // 加入机械安装补偿角 (对齐D轴)
        ElecTheta_pu += Sensor_Offset_pu;
        if(ElecTheta_pu >= 1.0f) ElecTheta_pu -= 1.0f;

        // 转子位置（弧度）
        ElecTheta_rad = ElecTheta_pu * PI2;

        // 悬浮系统的电角度（极对数差1时，悬浮坐标系以2倍转子电角频率旋转）
        ElecTheta_Sus_rad = 2.0f * ElecTheta_rad;
        // 确保悬浮角度被限制在 -PI 到 PI 之间
        if (ElecTheta_Sus_rad > PI)       ElecTheta_Sus_rad -= PI2;
        else if (ElecTheta_Sus_rad < -PI) ElecTheta_Sus_rad += PI2;

        // 5. 悬浮电流 Park 变换：Ia, Ib, Ic ---> Id, Iq
        park2.Alpha = clark2.Alpha;
        park2.Beta = clark2.Beta;
        park2.Cos = cosf(ElecTheta_Sus_rad);
        park2.Sin = sinf(ElecTheta_Sus_rad);
        park2.calc(&park2);

        // 悬浮电流环 PI：Id, Iq ---> Ud, Uq
        gSuspCurrD_PID.Ref = i_Bd_ref;
        gSuspCurrD_PID.Fdb = park2.Ds;
        gSuspCurrD_PID.calc(&gSuspCurrD_PID);

        gSuspCurrQ_PID.Ref = i_Bq_ref;
        gSuspCurrQ_PID.Fdb = park2.Qs;
        gSuspCurrQ_PID.calc(&gSuspCurrQ_PID);

        // 悬浮反 Park 变换：Ud, Uq---> Ualpha, Ubeta
        ipark2.Ds = gSuspCurrD_PID.Out;
        ipark2.Qs = gSuspCurrQ_PID.Out;
        ipark2.Cos = park2.Cos;
        ipark2.Sin = park2.Sin;
        ipark2.calc(&ipark2);

        // 6. 悬浮 SVPWM 更新
        Update_Suspension_PWM(ipark2.Alpha, ipark2.Beta);

        // --------------------------------------------------------------------
        // 3.2 悬浮就绪状态判定
        // --------------------------------------------------------------------
        if (fabsf(Pos_X_Fdb) < POS_READY_THRES && fabsf(Pos_Y_Fdb) < POS_READY_THRES)
        {
            if (Sus_Ready_Tick < SUS_READY_DELAY) Sus_Ready_Tick++;
            else Sus_Ready = 1; // 满足阈值且延时到达，标志悬浮成功！
        }
        else
        {
            Sus_Ready_Tick = 0;
            Sus_Ready = 0; // 若受到扰动偏离中心，立刻撤销就绪标志
        }

        // --------------------------------------------------------------------
        // 3.3 转矩驱动系统控制 (仅在悬浮稳定后工作)
        // --------------------------------------------------------------------
        if (Sus_Ready == 1)
        {
            if(EnDrive == 0)
            {
                EALLOW;
                // 仅当就绪瞬间，解封转矩模块 PWM (1, 2, 3)
                EPwm1Regs.TZCLR.bit.OST = 1;
                EPwm2Regs.TZCLR.bit.OST = 1;
                EPwm3Regs.TZCLR.bit.OST = 1;
                EDIS;
                // 积分器清零，防止启动瞬间电流猛冲
                gTorqCurrD_PI.Ui = 0; gTorqCurrQ_PI.Ui = 0; gSpd_PI.Ui = 0;
                EnDrive = 1;
            }

            // --- 转矩闭环计算 ---
//            // 1. 转矩电流 Clark 变换（此处1和4提到前面，以进行力-电流转换）
//            clark1.As = Iu;
//            clark1.Bs = Iv;
//            clark1.calc(&clark1);

            // 2. 计算反馈转速 (微分+滤波)
            float dTheta = ElecTheta_pu - ElecTheta_pre;
            ElecTheta_pre = ElecTheta_pu;

            // 处理角度溢出突变 (过零点处理)
            if(dTheta >  0.5f) dTheta -= 1.0f;
            if(dTheta < -0.5f) dTheta += 1.0f;

            // 计算物理转速 RPM: (dTheta * 频率 * 60) / 极对数
            Speed_Raw_rpm = (dTheta * ISR_FREQ * 60.0f) / POLE_PAIRS;
            // 一阶低通滤波 (时间常数需根据噪声调整)
            Speed_Fdb_rpm = 0.95f * Speed_Fdb_rpm + 0.05f * Speed_Raw_rpm;

            // 3. 速度环 (外环)
            gSpd_PI.Ref = Speed_Ref_rpm;
            gSpd_PI.Fdb = Speed_Fdb_rpm;
            gSpd_PI.calc(&gSpd_PI);

//            // 4. 转矩电流 Park 变换：Ia, Ib
//            park1.Alpha = clark1.Alpha;
//            park1.Beta = clark1.Beta;
//            park1.Cos = cosf(ElecTheta_rad);
//            park1.Sin = sinf(ElecTheta_rad);
//            park1.calc(&park1);


            // 5. 电流环 (内环)
            gTorqCurrD_PI.Ref = 0.0f;          // D 轴：id = 0
            gTorqCurrD_PI.Fdb = park1.Ds;
            gTorqCurrD_PI.calc(&gTorqCurrD_PI);

            gTorqCurrQ_PI.Ref = gSpd_PI.Out;   // Q 轴
            gTorqCurrQ_PI.Fdb = park1.Qs;
            gTorqCurrQ_PI.calc(&gTorqCurrQ_PI);

            // 6. 反 Park 变换
            ipark1.Ds = gTorqCurrD_PI.Out;
            ipark1.Qs = gTorqCurrQ_PI.Out;
            ipark1.Cos = park1.Cos;
            ipark1.Sin = park1.Sin;
            ipark1.calc(&ipark1);

            // 6. 转矩 SVPWM 更新
            Update_Svpwm(ipark1.Alpha, ipark1.Beta); // 发给转矩逆变器
        }
        else
        {
            EnDrive = 0;
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST = 1;
            EPwm2Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZFRC.bit.OST = 1;
            EDIS;
        }
    }
    // ========================================================================
    // 步骤 4: 停机/故障保护态处理
    // ========================================================================
    else
    {
        // 系统关闭或发生故障，封锁所有 PWM
        EALLOW;
        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm3Regs.TZFRC.bit.OST = 1;
        EPwm4Regs.TZFRC.bit.OST = 1;
        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm6Regs.TZFRC.bit.OST = 1;
        EDIS;

        // 状态标志位全清零
        EnDrive = 0;
        EnSus_Flag = 0;
        Sus_Ready = 0;
        Sus_Ready_Tick = 0;
        Speed_Fdb_rpm = 0;

        // 所有 PID 积分器清零，防止再次启动时炸机
        gTorqCurrD_PI.Ui = 0; gTorqCurrQ_PI.Ui = 0; gSpd_PI.Ui = 0;
        pid_x.Ui = 0;  pid_y.Ui = 0;
        gSuspCurrD_PID.Ui = 0; gSuspCurrQ_PID.Ui = 0;
    }

    // 5. 清除中断标志
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
*/
