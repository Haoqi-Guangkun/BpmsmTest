# BpmsmTest - 无刷永磁同步电机 (BPMSM) 控制工程

基于 TI TMS320F28335 DSP 的无刷永磁同步电机 (Brushless Permanent Magnet Synchronous Motor) 控制项目，使用 Code Composer Studio (CCS) 开发。

## 项目结构

```
BpmsmTest/
├── include/          # 头文件（DSP驱动、电机控制算法）
│   ├── DSP2833x_*.h  # TI DSP2833x 外设驱动头文件
│   ├── clark.h       # Clark 变换
│   ├── park.h        # Park 变换
│   ├── ipark.h       # 反Park 变换
│   ├── pid_reg.h     # PID 调节器
│   ├── Svpwm.h       # SVPWM 空间矢量调制
│   └── wave.h        # 波形生成
├── src/              # 源文件
│   ├── main.c        # 主程序入口
│   ├── Adc.c/h       # ADC 采样
│   ├── control.c     # 核心控制逻辑
│   ├── clark.c       # Clark 变换实现
│   ├── park.c        # Park 变换实现
│   ├── ipark.c       # 反Park 变换实现
│   ├── pid_reg.c     # PID 调节器实现
│   ├── Svpwm.c       # SVPWM 实现
│   ├── wave.c        # 波形生成实现
│   ├── yff.c         # 用户功能模块
│   ├── cut_mao.c     # 裁角控制
│   └── DSP2833x_*.c  # TI DSP 库源文件
├── targetConfigs/    # 目标板配置文件
├── 28335_RAM_lnk.cmd # RAM 链接命令文件
├── F28335.cmd        # Flash 链接命令文件
└── DSP2833x_Headers_nonBIOS.cmd
```

## 开发环境

- **IDE**: Code Composer Studio (CCS)
- **芯片**: TI TMS320F28335
- **语言**: C / 汇编
- **控制算法**: FOC (磁场定向控制)、SVPWM、Clark/Park 变换、PID 调节

## 快速开始

1. 安装 [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO)
2. 克隆本仓库
3. 在 CCS 中 `File → Import → CCS Projects` 导入项目
4. 编译并下载到 TMS320F28335 目标板

## 协作方式

本项目采用 **Fork + Pull Request** 工作流，4 位开发者各自拥有个人仓库。
