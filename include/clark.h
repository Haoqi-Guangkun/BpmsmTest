/*
 * clark.h
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#ifndef CLARK_H_
#define CLARK_H_
#include "DSP2833x_Device.h"

// 定义 CLARK 结构体
typedef struct CLARK_OBJ {
    float As;       // 输入：A相电流
    float Bs;       // 输入：B相电流
    float Cs;       // 输入：C相电流 (在两相电流采样中通常不参与计算)
    float Alpha;    // 输出：Alpha轴分量
    float Beta;     // 输出：Beta轴分量
    void (*calc)(void *); // 计算函数指针//解释：定义一个函数指针成员calc，它指向：返回值为void输入参数为void*的函数
} CLARK;

// 定义句柄类型
typedef CLARK * CLARK_handle;

// 默认初始化宏定义
// 将函数指针强转为 (void (*)(void *)) 更加规范，避免类型警告
#define CLARK_DEFAULTS {0.0f, \
                        0.0f, \
                        0.0f, \
                        0.0f, \
                        0.0f, \
                        (void (*)(void *))clark_calc}//这里有一个强制类型转换，就是把这个函数指针类型强行转换成void（*）类型的函数指针//函数指针类型主要由返回值的类型、参数个数、参数类型三个部分组成

// 函数声明
void clark_calc(CLARK *p);

#endif /* CLARK_H_ */
