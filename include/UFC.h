#ifndef UNBALANCE_COMPENSATION_H_
#define UNBALANCE_COMPENSATION_H_

#include "DSP2833x_Device.h"
#include "control.h"

// ============================================================================
// 1. 结构体定义（把它们挪到最上面，让编译器先认识 SPU 和 UFC 类型）
// ============================================================================
typedef struct SPU_Instance {
  float32 sum_sin;
  float32 sum_cos;
  Uint32 sample_cnt;
  Uint32 N_samples;
  float32 a;
  float32 b;
} SPU;

typedef struct Unbalance_Compensator {
  SPU spu_x;
  SPU spu_y;
  PIDREG pid_u;
  PIDREG pid_v;
  PARK park;
  IPARK ipark;
  float32 omega_r;
  float32 Ts;

  float32 Fx_out; // 不平衡力补偿输出 X
  float32 Fy_out; // 不平衡力补偿输出 Y

  // 🌟 专门为后台任务新增的缓冲区
  float32 x_raw_buf;   // 缓存 X 轴位移
  float32 y_raw_buf;   // 缓存 Y 轴位移
  float32 theta_buf;   // 缓存电角度
  Uint16 NewData_Flag; // 触发后台计算的旗帜
} UFC;

// ============================================================================
// 2. 子模块初始化宏（参数和排版完全保留）
// ============================================================================
#define UNBALANCE_PID_U_DEFAULTS                                               \
  {0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   15.0f,                                                                      \
   -15.0f,                                                                     \
   10.0f,                                                                      \
   -10.0f,                                                                     \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   PID_TS_DEFAULT,                                                             \
   400.0f,                                                                     \
   (1.0f - 400.0f * PID_TS_DEFAULT),                                           \
   (void (*)(void *))pid_reg_calc}

#define UNBALANCE_PID_V_DEFAULTS                                               \
  {0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   20000.0f,                                                                   \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   15.0f,                                                                      \
   -15.0f,                                                                     \
   10.0f,                                                                      \
   -10.0f,                                                                     \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   PID_TS_DEFAULT,                                                             \
   400.0f,                                                                     \
   (1.0f - 400.0f * PID_TS_DEFAULT),                                           \
   (void (*)(void *))pid_reg_calc}

// ============================================================================
// 3. 主模块初始化宏（成员顺序：spu_x, spu_y, pid_u, pid_v, park, ipark,
// omega_r, Ts）
// ============================================================================
#define UNBALANCE_COMP_DEFAULTS                                                \
  {{0.0f, 0.0f, 0, 0, 0.0f, 0.0f},                                             \
   {0.0f, 0.0f, 0, 0, 0.0f, 0.0f},                                             \
   UNBALANCE_PID_U_DEFAULTS, /* 抽离后的 u 轴 PID */                           \
   UNBALANCE_PID_V_DEFAULTS, /* 抽离后的 v 轴 PID */                           \
   {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, (void (*)(void *))park_calc},          \
   {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, (void (*)(void *))ipark_calc},         \
   0.0f,                                                                       \
   PID_TS_DEFAULT,                                                             \
   0.0f,                                                                       \
   0.0f, /* 初始化 Fx_out 和 Fy_out 为 0 */                                    \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0.0f,                                                                       \
   0}

void Unbalance_Comp_Run(UFC *p, float32 x_raw, float32 y_raw,
                        float32 theta_hall);

#endif /* UNBALANCE_COMPENSATION_H_ */
