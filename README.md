# Balanced_WheelLeg_Robot

> **串联腿构型轮腿机器人**的底盘控制代码 采用VMC+LQR控制方案

---

## 📖 项目简介

本项目是 RoboMaster 比赛中**串联腿构型（Serial Leg）轮腿机器人**的全套底盘控制固件。串联腿构型通过四个独立控制的关节电机，实现腿部长度的动态调节（0.15m ~ 0.35m），使机器人具备**主动悬挂**能力——可根据地形和运动状态实时调整每条腿的长度，在保持机身平稳的同时实现灵活机动。

整套代码基于 STM32G474 平台和 FreeRTOS 实时操作系统构建，采用多任务并行架构，覆盖底盘运动解算、云台控制、发射控制、惯导姿态解算、裁判系统交互、遥控器解析、超电管理等全部功能模块。

---

## 🛠 硬件平台

| 项目 | 型号 / 规格 |
|------|------------|
| **主控芯片** | STM32G474VETx (Arm Cortex-M4F, 170MHz, 带 FPU + DSP 指令集) |
| **底盘轮电机 ×2** | DJI M3508 (FDCAN2, 标准 CAN) |
| **底盘关节电机 ×4** | DAMIAO 8009 (MIT控制, 髋关节) |
| **云台电机 ×2** | Pitch (CAN2) + Yaw (CAN3) |
| **摩擦轮电机 ×6** | DJI M2006 / 双级摩擦轮 (FDCAN3) |
| **拨弹电机 ×1** | DJI M2006 (CAN3) |
| **姿态传感器** | Bosch BMI088 (SPI4) + 外部 IMU (CAN3) |
| **遥控器** | 大疆 DR16 (USART1 SBUS) + RS485 扩展 (USART2/3) |
| **裁判系统** | 大疆裁判系统 (UART4/UART5) |
| **超级电容** | FDCAN1 (CAN ID 0x030/0x031) |
| **激光测距** | UART 激光传感器 |
| **电机总数** | 14 台电机统一管理 |

---

## 🧱 软件架构

### FreeRTOS 任务体系

| 任务 | 优先级 | 栈 (words) | 周期 | 职责 |
|------|--------|-----------|------|------|
| `Robo_Task` | `osPriorityRealtime` | 512 | 1ms | 顶层机器人控制调度，遥控器数据解析，状态机管理 |
| `Chassis_Task` | `osPriorityHigh` | 2048 | 1ms | 底盘运动学解算，轮腿协调控制，功率限制 |
| `Communicate_Task` | `osPriorityBelowNormal` | 512 | 1ms | 双板通信，超电控制帧发送 |
| `INS_Task` | `osPriorityLow` | 1024 | 1ms | 惯导姿态解算 (四元数 EKF) |
| `Gimbal_Task` | `osPriorityIdle` | 512 | 1ms | 云台 Pitch/Yaw 电机控制 |
| `Key_LED_Task` | `osPriorityIdle` | 256 | 10ms | 按键扫描 + RGB LED 驱动 |
| `Buzzer_Task` | `osPriorityIdle` | 256 | 1ms | 蜂鸣器音效 |
| `UI_Task` | `osPriorityIdle` | 512 | 1ms | 裁判系统操作界面绘制 |

> **说明**: `Shoot_Task` 和 `Aim_Task` 当前已注释，发射与自瞄逻辑在其他任务中直接调用。

### 核心数据结构

`RoboControl_StructTypeDef`（定义于 `RoboControl.h`）是机器人全局状态体，包含底盘目标速度、云台角度、关节参数、功率状态、发射状态等，作为各任务间的共享数据中枢。

### 控制流程图

```
遥控器 (SBUS) ──→ Robo_Task ──→ Chassis_Task ──→ 轮电机 (M3508) + 关节电机 (DAMIAO)
                      │
                      ├──→ Gimbal_Task ──→ 云台电机 (Pitch/Yaw)
                      │
                      ├──→ Shoot 控制 ──→ 摩擦轮 + 拨弹电机
                      │
              INS_Task (BMI088) ──→ 姿态反馈
              裁判系统 ──→ 功率限制 ──→ 超电管理
```

---

## 📂 目录结构

```
.
├── Core/                       # CubeMX 生成的 HAL 层代码
│   ├── Inc/                    # 外设头文件、FreeRTOSConfig.h、main.h
│   └── Src/                    # 外设实现、app_freertos.c (任务创建)、main.c
│
├── Drivers/                    # STM32G4xx HAL + CMSIS 官方驱动库
├── Middlewares/                # 第三方中间件 (FreeRTOS 内核 + ST USB 库)
├── USB_Device/                 # USB CDC 虚拟串口配置
│
├── Project/                    # ★ 应用层代码 (主要工作区)
│   ├── Algorithm_Drivers/      # 算法模块
│   │   ├── PID.c               #   PID 控制器 (位置式 + 前馈 + 低通滤波)
│   │   ├── kalman_filter.c     #   卡尔曼滤波器 (通用矩阵实现)
│   │   ├── QuaternionEKF.c     #   四元数扩展卡尔曼滤波 (姿态估计)
│   │   ├── controller.c        #   控制器工具 (模糊 PID 规则表、OLS 微分)
│   │   ├── Power_Limit.c       #   功率限制算法 (基于裁判系统数据)
│   │   ├── CRC8_CRC16.c        #   CRC 校验 (裁判系统协议)
│   │   ├── Vofa.c              #   VOFA 调试协议 (浮点数据可视化)
│   │   └── Function.c          #   通用函数 (限幅、死区、斜坡/斜率限制)
│   │
│   ├── BSP/                    # 板级支持包
│   │   ├── BMI088driver.c      #   BMI088 六轴 IMU 底层驱动 (SPI)
│   │   ├── BMI088Middleware.c  #   BMI088 中间层 (片选 + 读写)
│   │   ├── DWT.c               #   DWT 高精度微秒时钟
│   │   ├── PWM.c               #   PWM 输出 (TIM3 四通道)
│   │   ├── Buzzer.c            #   蜂鸣器驱动 (TIM5 PWM)
│   │   ├── Key.c                #   按键扫描 (三键 + 消抖状态机)
│   │   ├── LED.c               #   RGB LED 驱动 (TIM1 PWM + 颜色渐变)
│   │   └── Flash.c             #   片上 Flash 读写 (参数持久化)
│   │
│   ├── Hardware_Drivers/       # 硬件设备驱动
│   │   ├── Motor_Driver.c      #   通用电机数据结构 (14 台电机统一管理)
│   │   ├── Motor_DJI_Driver.c  #   大疆电机驱动 (M3508/M2006 CAN 控制)
│   │   ├── Motor_Unitree_Driver.c  # Unitree 关节电机驱动 (串口 MIT 控制)
│   │   ├── Motor_DAMIAO_Driver.c   # 达妙电机驱动 (CAN 位置/力矩控制)
│   │   ├── Remote_Control.c    #   遥控器解码 (SBUS 协议解析)
│   │   ├── Referee_Unpack.c    #   裁判系统数据解包
│   │   ├── SuperCap_Driver.c   #   超级电容 CAN 通信
│   │   └── Laser.c             #   激光测距传感器驱动
│   │
│   ├── Commnuicate_Drivers/    # 通信抽象层
│   │   ├── CAN_Driver.c        #   CAN/FDCAN 收发封装
│   │   ├── USART_Driver.c      #   串口驱动 (DMA 模式)
│   │   └── USB_Driver.c        #   USB CDC 虚拟串口
│   │
│   ├── Robot_Application/      # ★ 机器人核心逻辑
│   │   ├── RoboControl.c       #   机器人状态控制器 (核心调度)
│   │   ├── Chassis.c           #   底盘运动解算 (轮腿运动学)
│   │   ├── Gimbal.c            #   云台控制
│   │   ├── INS.c               #   惯导解算 (姿态融合)
│   │   ├── Shoot.c             #   发射控制 (单发/连发)
│   │   ├── Aim.c               #   自瞄通信与数据处理
│   │   ├── Communicate.c       #   双板间通信协议
│   │   ├── Key_LED.c           #   按键 + LED 任务封装
│   │   └── Define.h            #   全局宏定义 (CAN ID、电机参数、引脚映射)
│   │
│   ├── UI/                     # 裁判系统操作界面 (UI 绘制)
│   ├── UI-backup/              # UI 旧版备份
│   └── Vision_Old/             # 旧版视觉代码 (弹道解算)
│
├── MDK-ARM/                    # Keil 工程文件
│   ├── Kawashiro_Frame_G474.uvprojx   # ★ 工程文件 (双击打开)
│   └── startup_stm32g474xx.s          # 启动文件
│
├── Kawashiro_Frame_G474.ioc   # STM32CubeMX 配置文件
├── CLAUDE.md                   # Claude Code AI 辅助开发说明
└── README.md                   # 本文件
```

---

## 🔌 通信总线一览

| 总线 | 外设 | 用途 |
|------|------|------|
| **FDCAN1** | 标准 CAN | 底盘关节电机 (DAMIAO) + 超级电容 |
| **FDCAN2** | 标准 CAN | 底盘轮电机 (M3508) + 云台 Pitch 电机 |
| **FDCAN3** | 标准 CAN | 云台 Yaw 电机 + 摩擦轮 + 拨弹 + 外部 IMU |
| **SPI4** | SPI | BMI088 六轴姿态传感器 |
| **USART1** | UART | SBUS 遥控器信号 |
| **USART2/3** | UART | RS485 扩展通信 |
| **UART4/5** | UART | 裁判系统数据交互 |
| **USB CDC** | USB | VOFA 调试上位机 (浮点数据可视化) |

---

## 🚀 快速开始

### 环境要求

- **IDE**: Keil MDK-ARM 5.x
- **工具链**: ARM Compiler 5/6 (推荐 AC6)
- **配置工具**: STM32CubeMX (修改引脚配置时使用)
- **调试器**: J-Link / DAP-Link / ST-Link
- **上位机**: [VOFA+](https://www.vofa-plus.com/) (数据可视化调试)

### 编译

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/Kawashiro_Frame_G474.uvprojx`
2. 点击 **Build (F7)** 编译工程
3. 编译产物位于 `MDK-ARM/` 目录下

### 烧录

1. 通过调试器连接开发板
2. 在 Keil 中点击 **Download (F8)** 烧录固件

### 硬件配置修改

如需修改外设引脚配置：
1. 使用 STM32CubeMX 打开 `Kawashiro_Frame_G474.ioc`
2. 修改引脚/外设配置
3. 点击 **Generate Code** 重新生成代码
4. 位于 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间的自定义代码会被保留

---

## 🐛 调试指南

### 串口调试

- 使用 USB CDC 虚拟串口连接 PC
- 打开 VOFA+，选择对应串口，使用 `JustFloat` 协议
- 可实时查看浮动数据波形（电机速度、电流、姿态角等）

### 硬件调试

- 通过 J-Link / DAP-Link 在 Keil 中进行断点调试
- DWT 高精度时钟可用于代码段耗时测量（微秒级精度）

### 调试变量

- 在 `Vofa.c` 中可配置通过 VOFA 发送的调试变量
- 在 `Define.h` 中可调整各类控制参数宏定义

---

## 📐 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 轮半径 | 0.06 m | 底盘驱动轮 |
| 最大 Vx | 3.0 m/s | 前后平移最大速度 |
| 最大 Vy | 3.0 m/s | 左右平移最大速度 |
| 最大 Wz | 12.0 rad/s | 原地旋转最大角速度 |
| 腿长范围 | 0.15 ~ 0.35 m | 串联腿伸缩范围 |
| 电机总数 | 14 台 | 全局电机数组统一管理 |
| 系统主频 | 170 MHz | STM32G474 最大主频 |

---

## 🙏 致谢

本项目使用了以下开源代码和参考实现：

| 模块 | 来源 | 作者 |
|------|------|------|
| 底盘控制算法方案 | 平衡步兵控制系统开源 | Wang Hongxi |
| 卡尔曼滤波 / 四元数 EKF / 控制器 | 开源算法库 | Wang Hongxi |
| 功率限制算法 | 西交利物浦大学 (XJTLU) Maxwell's Demon 开源 | WUST_MURRAY |
| CRC8/CRC16 校验 | 大疆裁判系统协议 (DJI 2019) | DJI |
| BMI088 驱动 | Bosch 传感器驱动适配 | — |
| FreeRTOS | Amazon / Richard Barry | Real Time Engineers Ltd. |

---

## 📄 许可证

本项目为 RoboMaster 竞赛用途的嵌入式固件，仅供学习与竞赛参考。部分算法模块使用了第三方开源代码，请遵循其各自的许可证条款。

---

> **Kawashiro Frame** · Built with STM32G4 + FreeRTOS · RMUC 2026
