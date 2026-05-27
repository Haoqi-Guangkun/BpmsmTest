/*
 * control.c
 *
 *  Created on: 2026年3月16日
 *      Author: L.YF
 */

#include "control.h"
#include <math.h>
#include "Svpwm.h"

// ===================== 滑动窗口滤波 =====================
#define FILTER_SIZE 10    // 窗口大小 10~20 效果最好

// 内部静态变量
static Uint32 Align_Counter = 0;        // 预定位时间计数
// 角度变量
static float Raw_Hall_Theta_rad = 0.0f; // 霍尔传感器的原始电角度，弧度
static float Raw_Hall_Theta_pu = 0.0f;  // 霍尔直接计算得到的原始电角度，标幺值
extern float ElecTheta_pu;              // 标幺化电角度 (0.0 ~ 1.0)
static float ElecTheta_rad = 0.0f;      // 转子机械角度，弧度
static float ElecTheta_pre = 0.0f;      // 上一次的角度 (用于计算速度)
static float temp_sus_theta_pu = 0.0f;  // 根据模式确定转矩角度和悬浮角度
static float ElecTheta_Sus_pu = 0.0f;   // 悬浮系统电角度，标幺值
static float ElecTheta_Sus_rad = 0.0f;  // 悬浮系统电角度，弧度
static float dTheta = 0.0f;             // 转速，标幺值
//static Uint16 div_cnt = 0;              // 转速环降频

// 角度滤波相关变量
static float h_alpha_filt = 0.0f;
static float h_beta_filt = 0.0f;

// 速度滤波相关变量
static float speed_buffer[FILTER_SIZE] = {0.0f};  // 环形队列缓冲区
static Uint16 filter_idx  = 0;                    // 缓冲区指针
static float  speed_sum   = 0.0f;                 // 实时累加和，避免循环求和提高效率
static Uint16 data_count  = 0;

// 运行模式状态(全局枚举)
typedef enum {
    MODE_IDLE = 0,
    MODE_ALIGN = 1,   // 预定位模式
    MODE_RUN = 2      // 闭环运行模式
} MOTOR_MODE;

volatile MOTOR_MODE MotorMode = MODE_RUN;   // 默认模式：MODE_IDLE（空闲）

// 速度变量
extern float Speed_Ref_rpm;             // 目标给定转速 (RPM)
static float Speed_Raw_rpm = 0.0f;      // 瞬时计算转速
static float Speed_Fdb_rpm = 0.0f;      // 滤波后的实际转速
// 速度斜坡变量
float Speed_Target_Final = 1000.0f;     // 最终目标转速 (RPM)
float Speed_Target_Ramp = 0.0f;         // 当前斜坡指令转速 (送入电流环的Ref)
float Speed_Accel_Step = 0.2f;          // 每次步进的转速值 (RPM/采样周期),对应加速度 200 RPM/s
static Uint16 Speed_Loop_Cnt = 0;       // 速度环分频计数器

// 位移与力变量
static float Pos_D_Filtered = 0.0f, Pos_Q_Filtered = 0.0f;   // 低通滤波后的d、q轴实际位移
static float Force_D_Ref = 0.0f, Force_Q_Ref = 0.0f;         // d、q轴力给定 (PID输出)
static float i_Bd_ref = 0.0f, i_Bq_ref = 0.0f;               // 悬浮d、q轴电流给定

// 控制器实例化
CLARK  clark1  = CLARK_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;
CLARK  clark2  = CLARK_DEFAULTS;
PARK   park2   = PARK_DEFAULTS;
IPARK  ipark2  = IPARK_DEFAULTS;
PIDREG pid_d   = PIDREG_dPOS_DEFAULTS;   // d轴位移环位
PIDREG pid_q   = PIDREG_qPOS_DEFAULTS;   // q轴位移环位
extern PARK   park_eddy;           //外部声明
extern PIDREG pid_id_sus;                     // 悬浮d轴电流环
extern PIDREG pid_iq_sus;                     // 悬浮q轴电流环
extern PIDREG pid_id;
extern PIDREG pid_iq;
extern PIDREG pid_spd;                        // 速度环

// 在 main.c 中已定义的零偏
extern float offsetA, offsetB, offsetC;
extern float offsetU, offsetV, offsetW;
extern float Hall_OffsetA, Hall_OffsetB, Hall_OffsetC;
extern float Eddy_OffsetA, Eddy_OffsetB, Eddy_OffsetC;
extern float Ia, Ib, Ic;            // 悬浮三相电流
extern float Iu, Iv, Iw;            // 转矩三相电流
extern float ha, hb, hc;            // 霍尔传感器电压值
extern float Ea, Eb, Ec;            // 电涡流传感器电压值
extern float Pos_D_Ref, Pos_Q_Ref;  // d、q轴目标位移
extern float SENSOR_POS_SCAL;
extern float Hall_Angle_Offset;
extern Uint16 AlignDone, EnSus;

float LPF_out[10] = {0};
// 低通滤波器
// 针对 10kHz 采样频率修正
float one_order_LPF(float cutoff_fre, float input, Uint16 i)
{
    // A = (2*pi*fc*Ts) / (1 + 2*pi*fc*Ts)
    // 其中 Ts = 0.0001s (10kHz)
    const float Ts = 0.0001f;
    float alpha = 2.0f * PI * cutoff_fre * Ts;
    float A = alpha / (1.0f + alpha);
    LPF_out[i] = A * input + (1.0f - A) * LPF_out[i];
    return LPF_out[i];
}

// 转速滑动窗口滤波
float Speed_Filter(float input)
{
    // 1. 减去最老数据
    speed_sum -= speed_buffer[filter_idx];
    // 2. 存入最新数据
    speed_buffer[filter_idx] = input;
    // 3. 加上最新数据
    speed_sum += speed_buffer[filter_idx];
    // 4. 上电前20次自动平滑，不跳变
    if(data_count < FILTER_SIZE)
    {
        data_count++;
    }
    // 5. 计算滤波后转速
    float output = speed_sum / data_count;
    // 6. 环形指针更新
    filter_idx++;
    if(filter_idx >= FILTER_SIZE)
    {
        filter_idx = 0;
    }
    return output;
}

// ==================== 1. 读取ADC ====================
void Read_Adc(void)
{
    // 读取结果并转换为电压值
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
}

// ==================== 2. 数据转换 ====================
void Convert_Adc_Data(void)
{
    // 转矩三相电流
    Iu = (g_AdcData.Filtered_U - offsetU) * CURRENT1_SCALE;  //CURRENT1_SCALE：电压-电流转换系数（采样电阻+运放系数）
    Iv = (g_AdcData.Filtered_V - offsetV) * CURRENT1_SCALE;
    Iw = (g_AdcData.Filtered_W - offsetW) * CURRENT1_SCALE;
    // 悬浮三相电流
    Ia = (g_AdcData.Filtered_A - offsetA) * CURRENT2_SCALE;
    Ib = (g_AdcData.Filtered_B - offsetB) * CURRENT2_SCALE;
    Ic = (g_AdcData.Filtered_C - offsetC) * CURRENT2_SCALE;
    // 霍尔传感器电压值（减去静态偏置，得到以 0 为中心的交流正弦波）
    ha = -(g_AdcData.Hall_A - Hall_OffsetA);
    hb = -(g_AdcData.Hall_B - Hall_OffsetB);
    hc = -(g_AdcData.Hall_C - Hall_OffsetC);
    // 电涡流传感器电压值
    Ea = g_AdcData.Eddy_A - Eddy_OffsetA;
    Eb = g_AdcData.Eddy_B - Eddy_OffsetB;
    Ec = g_AdcData.Eddy_C - Eddy_OffsetC;
}

// ==================== 3. 公共反馈变量计算 ====================
void Calculate_Feedback(void)
{
    // =================== 1. 角度解算 ===================
    // 霍尔信号 Clark 变换 (提取直角坐标分量)，利用三相公式可以消除共模噪声，精度更高
//    float h_alpha = 0.666666667f * (ha - 0.5f * hb - 0.5f * hc);
//    float h_beta  = 0.577350269f * (hb - hc);
    // 计算霍尔传感器的原始电角度
    float h_alpha = 0.577350269f * (hb - hc);        // 霍尔坐标系与绕组坐标系同！
    float h_beta  = 0.666666667f * (-ha + 0.5f * hb + 0.5f * hc);

    // 简单的低通滤波，减少 atan2f 之前的噪声，滤波系数可调整。
//    h_alpha_filt = 0.7f * h_alpha_filt + 0.3f * h_alpha;
//    h_beta_filt  = 0.7f * h_beta_filt  + 0.3f * h_beta;
    Raw_Hall_Theta_rad = atan2f(h_beta, h_alpha);    // 求解绝对电角度，范围: -PI ~ PI

    // 转换为标幺值 0~1.0
    Raw_Hall_Theta_pu = Raw_Hall_Theta_rad * INV_PI2;//归一化
    if(Raw_Hall_Theta_pu < 0.0f) Raw_Hall_Theta_pu += 1.0f;//始终正角度显示

    if(MotorMode == MODE_ALIGN) {
        // 定位期间，锁定角度使转矩磁场固定方向
        if(Align_Counter < 10000) ElecTheta_pu = 0.25f; // 先拉到90度
        else ElecTheta_pu = 0.0f;                       // 再对齐0度 (A轴)
        // 悬浮依然使用原始霍尔角度（此时偏移量还没算出来）
        temp_sus_theta_pu = Raw_Hall_Theta_pu;
    }
    else {
        // 正常运行：ElecTheta_pu = 原始角度 - 零偏
        ElecTheta_pu = Raw_Hall_Theta_pu - Hall_Angle_Offset;//修正角度
        if(ElecTheta_pu < 0.0f)  ElecTheta_pu += 1.0f;
        if(ElecTheta_pu >= 1.0f) ElecTheta_pu -= 1.0f;
        temp_sus_theta_pu = ElecTheta_pu;
    }
    // 修正后的转（子位置弧度）
    ElecTheta_rad = ElecTheta_pu * PI2;

    // ------悬浮电角度------
    ElecTheta_Sus_pu = temp_sus_theta_pu * 2.0f;//0~2.0
    if (ElecTheta_Sus_pu >= 1.0f) ElecTheta_Sus_pu -= 1.0f;//0-2pi  周期折叠
    if (ElecTheta_Sus_pu >= 1.0f) ElecTheta_Sus_pu -= 1.0f; // 针对2倍频，最多减两次
    ElecTheta_Sus_rad = ElecTheta_Sus_pu * PI2;

    // =================== 2. 位移解算 ===================
    float E_alpha = 0.577350269f * (Ea - Ec);      //电涡流坐标系与绕组坐标系同！
    float E_beta  = 0.666666667f * (Eb - 0.5f * Ea - 0.5f * Ec);

    park_eddy.Alpha = E_alpha * SENSOR_POS_SCALE ;
    park_eddy.Beta = E_beta * SENSOR_POS_SCALE;
    park_eddy.Cos = cosf(temp_sus_theta_pu * PI2);
    park_eddy.Sin = sinf(temp_sus_theta_pu * PI2);
    park_eddy.calc(&park_eddy);//计算得到D,Q轴数据---park_eddy.Ds
    Pos_D_Filtered = one_order_LPF(500.0f, park_eddy.Ds, 0);
    Pos_Q_Filtered = one_order_LPF(500.0f, park_eddy.Qs, 1);

//    // =================== 3. 转速解算 ===================
//    // 计算转速 (微分+滤波)
//    div_cnt++;
//    if(div_cnt >= 10)  // 每10次电流环执行一次，即1kHz
//    {
//        div_cnt = 0;
//        dTheta = ElecTheta_pu - ElecTheta_pre;
//        ElecTheta_pre = ElecTheta_pu;
//        // 处理角度溢出突变 (过零点处理)
//        if(dTheta >  0.5f) dTheta -= 1.0f;
//        if(dTheta < -0.5f) dTheta += 1.0f;
//        // 计算物理转速 RPM: (dTheta * 频率 * 60) / 极对数
//        Speed_Raw_rpm = (dTheta * ISR_FREQ * 0.1f * 60.0f) / POLE_PAIRS;
//        // 一阶低通滤波 (时间常数需根据噪声调整)
////        Speed_Fdb_rpm = 0.85f * Speed_Fdb_rpm + 0.15f * Speed_Raw_rpm;
//        Speed_Fdb_rpm = Speed_Filter(Speed_Raw_rpm);
//    }
    //========================电流============================
    // 悬浮电流与转矩电流 Clark 变换：Ia, Ib, Ic ---> Ialpha, Ibeta
    clark2.As = Ia;        // 悬浮
    clark2.Bs = Ib;
    clark2.calc(&clark2);

    clark1.As = Iu;        // 转矩
    clark1.Bs = Iv;
    clark1.calc(&clark1);

    // 悬浮电流与转矩电流 Park 变换：Ialpha, Ibeta ---> Id, Iq
    park2.Cos = cosf(ElecTheta_Sus_rad);
    park2.Sin = sinf(ElecTheta_Sus_rad);
    park2.calc(&park2);//已经算好park2.Ds
}

// ==================== 3. 悬浮算法 ====================
void Control_Suspension(void)
{
    // 解封 TZ，每周期都确保PWM不被锁死
    EALLOW;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZCLR.bit.OST = 1;
    EDIS;

    // 1. 位移环 PID：输入给定 x*, y*, 反馈 x, y，输出力给定 Fx*, Fy*)
//    pid_d.Ref = Pos_D_Ref;
    pid_d.Fdb = park_eddy.Ds;//通过位移解算的值
    pid_d.calc(&pid_d);//根据反馈值计算参考力
    Force_D_Ref = -pid_d.Out;    //得到 Fd*,加了负号
//    pid_q.Ref = Pos_Q_Ref;
    pid_q.Fdb = park_eddy.Qs;
    pid_q.calc(&pid_q);
    Force_Q_Ref = -pid_q.Out;    //得到 Fq*
    // 2. 力-电流解耦计算（公式），需根据所用模型修改！！！！
    float I1q = park1.Qs;                                // 获取转矩绕组 q 轴电流
    float u3 = MOTOR_L1Q * I1q; //测试：0                // 计算中间变量 u(3) = L1q * I1q
    float denom = MOTOR_KM * (MOTOR_PHI_F_SQ + u3 * u3); // 计算公式公共分母: Km * (Phi_f^2 + u(3)^2)
    if(denom < 1e-6f && denom > -1e-6f) denom = 1e-6f;   // 【硬件保护】防止分母意外变为 0 导致单片机除零死机复位
    float inv_denom = 1.0f / denom;                      // 求分母倒数 (由于浮点乘法指令通常比除法指令快得多，优化DSP执行效率)
    i_Bd_ref = (Force_D_Ref * MOTOR_PHI_F - Force_Q_Ref * u3) * inv_denom;
    i_Bq_ref = (Force_D_Ref * u3 + Force_Q_Ref * MOTOR_PHI_F) * inv_denom;

    // 3. 悬浮电流环 PI：Id, Iq ---> Ud, Uq
    pid_id_sus.Ref = i_Bd_ref;    // 测试电流环PI时，临时注释
    pid_id_sus.Fdb = park2.Ds;
    pid_id_sus.calc(&pid_id_sus);
    pid_iq_sus.Ref = i_Bq_ref;    // 测试电流环PI时，临时注释
    pid_iq_sus.Fdb = park2.Qs;
    pid_iq_sus.calc(&pid_iq_sus);

    // 4. 悬浮反 Park 变换：Ud, Uq---> Ualpha, Ubeta
    ipark2.Ds = pid_id_sus.Out;   //临时测试0.05f
    ipark2.Qs = pid_iq_sus.Out;
    ipark2.Cos = park2.Cos;
    ipark2.Sin = park2.Sin;
    ipark2.calc(&ipark2);

    // 5. 悬浮 SVPWM 更新
    Update_Suspension_PWM(ipark2.Alpha, ipark2.Beta);
}

// ==================== 4. 驱动算法 ====================
void Control_Torque(void)
{
    // 根据不同模式执行控制
    switch(MotorMode)
    {
       // ------------------ 【模式 0：空闲状态】 ------------------
       case MODE_IDLE:
           Stop_Torque();          // 封锁 PWM 输出，以及清除 PID 积分项，防止切换模式时的积分过冲
           Speed_Target_Ramp = 0;  // 斜坡指令清零//软启动
           Align_Counter = 0;
           break;
       // ------------------ 【模式 1：预定位状态】 ------------------
       case MODE_ALIGN:
           EALLOW;
           EPwm1Regs.TZCLR.bit.OST = 1;
           EPwm2Regs.TZCLR.bit.OST = 1;
           EPwm3Regs.TZCLR.bit.OST = 1;
           EDIS;

           pid_id.Ref = 0.1f;  // 给一定的拉入电流
           pid_id.Fdb = park1.Ds;
           pid_id.calc(&pid_id);
           pid_iq.Ref = 0.0f;
           pid_iq.Fdb = park1.Qs;
           pid_iq.calc(&pid_iq);

           ipark1.Ds = pid_id.Out;
           ipark1.Qs = pid_iq.Out;
           ipark1.Cos = park1.Cos; // 使用 Calculate_Feedback 强制的角度
           ipark1.Sin = park1.Sin;
           ipark1.calc(&ipark1);

           Update_Svpwm(ipark1.Alpha, ipark1.Beta);

           Align_Counter++;
           if(Align_Counter == 20000)  // 1秒(10000次中断)后捕获
           {
               Hall_Angle_Offset = Raw_Hall_Theta_pu; // 记录偏置
               AlignDone = 1;                         // 标记定位成功
               MotorMode = MODE_IDLE;                 // 定位结束先回 IDLE 缓冲，等待启动指令
           }
           break;
           // ------------------ 【模式 2：闭环运行状态】 ------------------
       case MODE_RUN:
//               if(AlignDone == 0) {   // 安全检查：如果没定位过，拒绝运行
//                   MotorMode = MODE_IDLE;
//                   break;
//               }
           EALLOW;
           EPwm1Regs.TZCLR.bit.OST = 1;
           EPwm2Regs.TZCLR.bit.OST = 1;
           EPwm3Regs.TZCLR.bit.OST = 1;
           EDIS;
           // 1. 速度环分频计算 & 斜坡发生器 (每 10 个电流周期执行一次速度环)-----1KHz?
           // 转速环 PI 的执行频率必须与转速计算频率对齐
           if (++Speed_Loop_Cnt >= 10)//前置自增
           {
               Speed_Loop_Cnt = 0;

               // === A. 转速解算 ===
               // 计算转速 (微分+滤波)
               dTheta = ElecTheta_pu - ElecTheta_pre;
               ElecTheta_pre = ElecTheta_pu;
               // 处理角度溢出突变 (过零点处理)
               if(dTheta >  0.5f) dTheta -= 1.0f;
               if(dTheta < -0.5f) dTheta += 1.0f;

               // 计算物理转速 RPM: (dTheta * 频率 * 60) / 极对数
               // 频率系数 0.1f 已经包含了分频逻辑
               Speed_Raw_rpm = (dTheta * ISR_FREQ * 0.1f * 60.0f) / POLE_PAIRS;

               // === B. 转速滤波 ===
               // 一阶低通滤波 (时间常数需根据噪声调整)
               Speed_Fdb_rpm = 0.95f * Speed_Fdb_rpm + 0.05f * Speed_Raw_rpm;
//               Speed_Fdb_rpm = Speed_Filter(Speed_Raw_rpm);   // 滑动滤波，可调整滑动窗口值

               // === C. 斜坡发生器 ===
               // 计算当前斜坡指令与最终目标转速的差距
               float error = Speed_Ref_rpm - Speed_Target_Ramp;
               // 逐步增加或减少 Speed_Target_Ramp
               if (error > Speed_Accel_Step)       Speed_Target_Ramp += Speed_Accel_Step;
               else if (error < -Speed_Accel_Step)  Speed_Target_Ramp -= Speed_Accel_Step;
               else                                Speed_Target_Ramp = Speed_Ref_rpm;

               // === D. 速度环 PI 计算 ===
               // 速度环 PI
//                   pid_spd.Ref = Speed_Target_Ramp;
//                   pid_spd.Ref = 300.0f;
               pid_spd.Fdb = Speed_Fdb_rpm;
               pid_spd.calc(&pid_spd);
           }

           // 2. 电流环 PI
           pid_id.Ref = 0.0f;                  // d 轴：id* = 0
           pid_id.Fdb = park1.Ds;
           pid_id.calc(&pid_id);
           pid_iq.Ref = -pid_spd.Out; //0.1f   // q 轴
           pid_iq.Fdb = park1.Qs;
           pid_iq.calc(&pid_iq);

           // 3. 转矩反 Park 变换
           ipark1.Ds = pid_id.Out;
           ipark1.Qs = pid_iq.Out;
           ipark1.Cos = park1.Cos;
           ipark1.Sin = park1.Sin;
           ipark1.calc(&ipark1);
           Update_Svpwm(ipark1.Alpha, ipark1.Beta);  //Update_Svpwm(1.0f, 0.0f);   // 测试电流方向用
           break;

       default:
           MotorMode = MODE_IDLE;
           break;
    }
}

/*
void Control_Torque(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    if (EnAlign == 1)
    {
        // ---------- 转子预定位 ----------
        pid_id.Ref = 0.2f;  // 给一定的拉入电流
        pid_id.Fdb = park1.Ds;
        pid_id.calc(&pid_id);
        pid_iq.Ref = 0.0f;
        pid_iq.Fdb = park1.Qs;
        pid_iq.calc(&pid_iq);

        ipark1.Ds = pid_id.Out;
        ipark1.Qs = pid_iq.Out;
        ipark1.Cos = park1.Cos; // 使用 Calculate_Feedback 强制的角度
        ipark1.Sin = park1.Sin;
        ipark1.calc(&ipark1);

        Update_Svpwm(ipark1.Alpha, ipark1.Beta);

        Align_Counter++;
        // 假设运行1秒(10000次中断)后捕获
        if(Align_Counter == 10000)
        {
            Hall_Angle_Offset = Raw_Hall_Theta_pu; // 记录偏置
            AlignDone = 1;                         // 标记定位成功
        }
        // 定位结束，延时一会后关闭使能，等待正常使能开启
        if(Align_Counter > 10100)
        {
            EnAlign = 0;
            Align_Counter = 0;
            Stop_Torque();
        }
    }
    else
    {
       // ---------- 正常 FOC 驱动 ----------
       // 1. 速度环
       pid_spd.Ref = Speed_Ref_rpm;
       pid_spd.Fdb = Speed_Fdb_rpm;
       pid_spd.calc(&pid_spd);

       // 2. 电流环 PI
       pid_id.Ref = 0.0f;         // d 轴：id* = 0
       pid_id.Fdb = park1.Ds;
       pid_id.calc(&pid_id);
       pid_iq.Ref = pid_spd.Out;  // q 轴
       pid_iq.Fdb = park1.Qs;
       pid_iq.calc(&pid_iq);

       // 3. 转矩反 Park 变换
       ipark1.Ds = pid_id.Out;
       ipark1.Qs = pid_iq.Out;
       ipark1.Cos = park1.Cos;
       ipark1.Sin = park1.Sin;
       ipark1.calc(&ipark1);
       Update_Svpwm(ipark1.Alpha, ipark1.Beta);
   }
}
*/
/*
void Control_Torque(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    // 1. 速度环
    pid_spd.Ref = Speed_Ref_rpm;
    pid_spd.Fdb = Speed_Fdb_rpm;
    pid_spd.calc(&pid_spd);

    // 2. 电流环 PI
    pid_id.Ref = 0.0f;         // d 轴：id* = 0
    pid_id.Fdb = park1.Ds;
    pid_id.calc(&pid_id);
    pid_iq.Ref = pid_spd.Out;  // q 轴
    pid_iq.Fdb = park1.Qs;
    pid_iq.calc(&pid_iq);

    // 3. 转矩反 Park 变换
    ipark1.Ds = pid_id.Out;
    ipark1.Qs = pid_iq.Out;
    ipark1.Cos = park1.Cos;
    ipark1.Sin = park1.Sin;
    ipark1.calc(&ipark1);

    // 4. 转矩 SVPWM 更新
    Update_Svpwm(ipark1.Alpha, ipark1.Beta);
}
*/

// ==================== 5. 停止逻辑 ====================
void Stop_Suspension(void)
{
    EALLOW;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EDIS;
    // 处于停止态时，将所有相关 PID 的积分器清零
    // 下次 EnSus_Flag 变为 1 时，积分器就是从 0 开始
    pid_d.Ui = 0;
    pid_q.Ui = 0;
    pid_id_sus.Ui = 0;
    pid_iq_sus.Ui = 0;
}

void Stop_Torque(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
    // 清零转矩和速度环积分器
    pid_id.Ui = 0;
    pid_iq.Ui = 0;
    pid_spd.Ui = 0;
}

void Stop_System(void)
{
    Stop_Suspension();
    Stop_Torque();
}

// 解锁全部的 PWM，用于初始化校准完成后
void Resume_System(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZCLR.bit.OST = 1;
    EDIS;
}

/*
void Control_Align(void)
{
    // 1. 解锁转矩 PWM (EPWM1-3)
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    // 2. 运行转矩电流环
    pid_id.Ref = 0.2f; // 您给定的拉入电流
    pid_id.Fdb = park1.Ds;
    pid_id.calc(&pid_id);
    pid_iq.Ref = 0.0f;
    pid_iq.Fdb = park1.Qs;
    pid_iq.calc(&pid_iq);

    // 3. 反 Park 变换
    ipark1.Ds = pid_id.Out;
    ipark1.Qs = pid_iq.Out;
    ipark1.Cos = park1.Cos;
    ipark1.Sin = park1.Sin;
    ipark1.calc(&ipark1);

    Update_Svpwm(ipark1.Alpha, ipark1.Beta);

    // 4. 定位计时
    Align_Counter++;
    if(Align_Counter == 10000)  // 1s时间
        // 捕获此时刻的原始电角度作为零位偏移量
        Hall_Angle_Offset = Raw_Hall_Theta_pu;
    if(Align_Counter > 100000)
    {
        Align_Counter = 10001;
//        EnAlign = 0;
//        AlignDone = 1;
//        Align_Counter = 0;
//        Stop_Torque(); // 定位完成后暂时封锁转矩 PWM，等待转速指令
    }
}
*/

