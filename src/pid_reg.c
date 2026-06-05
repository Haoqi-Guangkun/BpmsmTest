/*
 * pid_reg.c
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#include "pid_reg.h"

/*
 * PID 计算核心函数
 * 包含：位置式PID算法、积分抗饱和(Anti-windup)、输出限幅
 */

//#define Ts        (1.0f/10000.0f)   // PID控制周期 10kHz
//#define Nd        300.0f            // 微分滤波系数
//#define T_xing    (1.0f - Nd * Ts)  // 滤波计算值

//static float Ts = 1.0f/10000.0f;
//static Uint16 Nd = 300;
//static float T_xing = 1.0f - 300.0f * 1.0f/10000.0f;

//定义v结构体中Nd变量以及Ts变量   Ts=1.0f/10000.0f；Nd=300; 先定300实验   v->UiPre  v->UdPre  v->Ki_xing v->Kd_xing v->T_xing

//提前计算
//   v->Ki_xing = v->Ki * v->Ts ;
//   v->Kd_xing = v->Kd * v->Nd ;
//   v->T_xing = 1 - v->Nd * v->Ts ;

/*
 * PID 计算核心函数
 * 包含：位置式PID算法、积分抗饱和(Anti-windup)、输出限幅
 */
void pid_reg_calc(PIDREG *v)
{
    // 0. 中间系数实时更新
    v->T_xing = 1.0f - (v->Nd * v->Ts);

    // 1. 保存上一次误差，并计算当前误差
    v->ErrPre = v->Err;
    v->Err = v->Ref - v->Fdb;

    // 2. 比例项 (P)
    v->Up = v->Kp * v->Err;

    // 3. 积分项 (I) - 累加误差
    v->Ui = v->Ui + (v->Ki * v->Ts * v->Err);                       // 加入Ts

    // 积分限幅
    if(v->Ui > v->UiMAX)
    {
        v->Ui = v->UiMAX;
    }
    else if(v->Ui < v->UiMIN)
    {
        v->Ui = v->UiMIN;
    }

    // 4. 微分项 D + 低通滤波
    v->Ud = v->T_xing * v->Ud + (v->Kd *v->Nd * (v->Err - v->ErrPre));   // 修改：不完全微分！

    // 5. 计算未限幅前的总输出
    v->OutPreSat = v->Up + v->Ui + v->Ud;

    // 6. 总输出限幅处理
    if (v->OutPreSat > v->OutMax)         v->Out = v->OutMax;
    else if (v->OutPreSat < v->OutMin)    v->Out = v->OutMin;
    else                                  v->Out = v->OutPreSat;
}

/*
void pid_reg_calc(PIDREG *v)
{
    // 1. 保存上一次误差，并计算当前误差
    v->ErrPre = v->Err;
    v->Err = v->Ref - v->Fdb;

    // 2. 比例项 (P)
    v->Up = v->Kp * v->Err;

    // 3. 积分项 (I) - 累加误差
    v->Ui = v->Ui + (v->Ki * v->Err);

    // 积分抗饱和处理 (防止积分风卷)
    if(v->Ui > v->UiMAX)
    {
        v->Ui = v->UiMAX;
    }
    else if(v->Ui < v->UiMIN)
    {
        v->Ui = v->UiMIN;
    }

    // 4. 微分项 (D) - 误差的变化率
    v->Ud = v->Kd * (v->Err - v->ErrPre);

    // 5. 计算未限幅前的总输出
    v->OutPreSat = v->Up + v->Ui + v->Ud;

    // 6. 总输出限幅处理
    if (v->OutPreSat > v->OutMax)
    {
        v->Out = v->OutMax;
    }
    else if (v->OutPreSat < v->OutMin)
    {
        v->Out = v->OutMin;
    }
    else
    {
        v->Out = v->OutPreSat;
    }
}


*/
