/*
 * pid_reg.h
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#ifndef PID_REG_H_
#define PID_REG_H_

#include "DSP2833x_Device.h"

// 定义 PID 调节器结构体
typedef struct PIDREG_OBJ {
    float  Ref;         // 输入：参考值 (目标值)
    float  Fdb;         // 输入：反馈值 (实际值)
    float  Err;         // 变量：当前误差
    float  ErrPre;      // 变量：上一次误差 (用于计算微分)
    float  Kp;          // 参数：比例系数 (P)
    float  Up;          // 变量：比例输出项
    float  Ui;          // 变量：积分输出项
    float  Ud;          // 变量：微分输出项
    float  OutPreSat;   // 变量：饱和前的总输出
    float  OutMax;      // 参数：输出上限 (限幅)
    float  OutMin;      // 参数：输出下限 (限幅)
    float  UiMAX;       // 参数：积分上限 (抗积分饱和)
    float  UiMIN;       // 参数：积分下限 (抗积分饱和)
    float  Out;         // 输出：最终 PID 输出
    float  Ki;          // 参数：积分系数 (I)
    float  Kd;          // 参数：微分系数 (D)

    // 新增：用于实时调试和计算的参数
    float  Ts;          // 参数：采样周期 (1/10000.0f)
    float  Nd;          // 参数：微分滤波系数 (通常 10~300)
    float  T_xing;      // 变量：滤波系数中间值 (1 - Nd*Ts)

    void   (*calc)(void *); // 函数指针：指向计算函数
} PIDREG;

typedef PIDREG *PIDREG_handle;

// 默认采样频率
#define PID_TS_DEFAULT  (0.0001f)   // 10kHz

//-----------------------------------------------------------------------------
// 默认初始化宏
// 注意：所有数字必须带 'f' 后缀，确保使用单精度浮点运算
//-----------------------------------------------------------------------------

// 转矩Q轴电流环默认参数 (PI控制)
#define PIDREG_IQ_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                             5.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                             13.16f, -13.16f, 12.0f, -12.0f, \
                             0.0f, 2500.0f, 0.0f, \
                             PID_TS_DEFAULT, 300.0f, 0.0f, \
                             (void (*)(void *))pid_reg_calc }

// 转矩D轴电流环默认参数 (PI控制)
#define PIDREG_ID_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                             5.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                             13.16f, -13.16f, 12.0f, -12.0f, \
                             0.0f, 2500.0f, 0.0f, \
                             PID_TS_DEFAULT, 300.0f, 0.0f, \
                             (void (*)(void *))pid_reg_calc }

// 速度环默认参数
#define PIDREG_SPD_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                              0.01f, 0.0f, 0.0f, 0.0f, 0.0f, \
                              4.0f, -4.0f, 3.0f, -3.0f, \
                              0.0f, 0.0f, 0.0f, \
                              PID_TS_DEFAULT, 300.0f, 0.0f, \
                              (void (*)(void *))pid_reg_calc }

// 悬浮D/Q轴电流环默认参数 (PI控制)
#define PIDREG_ISUS_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                               10.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                               13.16f, -13.16f, 12.0f, -12.0f, \
                               0.0f, 3000.0f, 0.0f, \
                               PID_TS_DEFAULT, 300.0f, 0.0f, \
                               (void (*)(void *))pid_reg_calc }

// 悬浮位移环d轴默认参数 (PID控制)
#define PIDREG_dPOS_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                              25000.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                              15.0f, -15.0f, 10.0f, -10.0f, \
                              0.0f, 500.0f, 100.0f, \
                              PID_TS_DEFAULT, 300.0f, 0.0f, \
                              (void (*)(void *))pid_reg_calc }
// 悬浮位移环q轴默认参数 (PID控制)
#define PIDREG_qPOS_DEFAULTS { 0.0f, 0.0f, 0.0f, 0.0f, \
                              23000.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                              15.0f, -15.0f, 10.0f, -10.0f, \
                              0.0f, 500.0f, 100.0f, \
                              PID_TS_DEFAULT, 300.0f, 0.0f, \
                              (void (*)(void *))pid_reg_calc }

// 函数声明
void pid_reg_calc(PIDREG *v);

#endif /* PID_REG_H_ */
