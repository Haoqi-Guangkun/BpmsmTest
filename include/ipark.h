/*
 * ipark.h
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#ifndef IPARK_H_
#define IPARK_H_
#include "DSP2833x_Device.h"

// 定义 IPARK 结构体
typedef struct IPARK_OBJ {
    float Ds;       // 输入：旋转坐标系 D 轴分量
    float Qs;       // 输入：旋转坐标系 Q 轴分量
    float Alpha;    // 输出：静止坐标系 Alpha 轴分量
    float Beta;     // 输出：静止坐标系 Beta 轴分量
    float Cos;      // 输入：当前电角度的余弦值
    float Sin;      // 输入：当前电角度的正弦值
    void (*calc)(void *); // 计算函数指针
} IPARK;

// 定义句柄类型
typedef IPARK *IPARK_handle;

// 默认初始化宏定义
// 使用 0.0f 强制单精度浮点，并修复指针强转警告
#define IPARK_DEFAULTS { 0.0f, 0.0f, \
                         0.0f, 0.0f, \
                         0.0f, 0.0f, \
                         (void (*)(void *))ipark_calc }

// 函数声明
void ipark_calc(IPARK *p);

#endif /* IPARK_H_ */
