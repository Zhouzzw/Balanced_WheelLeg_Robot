# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with this repository.

## 项目定位

RM-WheelLeg — **串联腿构型平衡轮腿机器人**的底盘控制固件。

- **技术重心**：底盘运动控制（正运动学 + 变参数 LQR + VMC 虚拟模型控制 + 特殊机动动作）。
- **定位说明**：底盘运控是项目的技术价值所在，但**整体控制框架必须完整、规范**——云台 / 发射 / 惯导 / 裁判 / 通信等外围模块作为底盘运控的支撑，全部纳入统一的多任务控制架构，不可只写底盘、丢框架。

- **主控芯片**: STM32G474VETx (Arm Cortex-M4F, 170MHz, FPU+DSP)
- **实时系统**: FreeRTOS V10.3.1 (CMSIS RTOS v1 封装)
- **开发环境**: Keil MDK-ARM (uvprojx) + STM32CubeMX (.ioc)
- **通信总线**: CAN (FDCAN1/2/3), USART (SBUS/RS485), SPI (IMU), I2C, USB CDC
- **依赖库**: CMSIS-DSP (`arm_math.h`) 加速正运动学与旋转分解三角计算

## 项目架构

### 目录分层

```
Core/                    ← CubeMX 生成的代码（外设初始化、FreeRTOS 配置、中断）
  ├── Inc/               ← 外设头文件、FreeRTOSConfig.h、main.h
  └── Src/               ← 外设实现、app_freertos.c（任务创建入口）、main.c

Drivers/                 ← STM32G4xx HAL + CMSIS（官方库，建议不改动）
Middlewares/             ← FreeRTOS 内核 + ST USB 设备库
USB_Device/              ← USB CDC 虚拟串口配置

Project/                 ← 应用层代码（主要工作区）
  ├── Algorithm_Drivers/ ← 算法层：PID、卡尔曼、四元数EKF、控制器、功率限制、VOFA调试
  ├── BSP/               ← 板级支持：BMI088驱动、DWT时钟、Buzzer、Key、LED、PWM、Flash
  ├── Commnuicate_Drivers/ ← CAN/FDCAN、USART(DMA)、USB 驱动
  ├── Hardware_Drivers/  ← 电机（DJI M3508/Unitree/DAMIAO 8009）、裁判系统、遥控、超电、激光
  ├── Robot_Application/ ← ★ 机器人核心逻辑：底盘、云台、发射、惯导(INS)、自瞄
  ├── UI/                ← 裁判系统UI绘制
  ├── UI-backup/         ← UI旧版备份
  └── Vision_Old/        ← 旧版视觉代码（弹道解算）

MDK-ARM/                 ← Keil 工程文件
  ├── Kawashiro_Frame_G474.uvprojx  ← 工程文件（核心，F7 编译）
  └── startup_stm32g474xx.s         ← 启动文件
```

### FreeRTOS 任务体系（app_freertos.c 中创建）

| 任务 | 优先级 | 栈(字) | 周期 | 职责 |
|------|--------|-------|------|------|
| `Robo_Task` | osPriorityRealtime | 512 | 1ms | 顶层调度：遥控解码、底盘状态机、功能标志触发 |
| `Chassis_Task` | osPriorityHigh | 2048 | 1ms | **底盘运控**：运动学解算、LQR/VMC、特殊机动、功率控制 |
| `Communicate_Task` | osPriorityBelowNormal | 512 | 1ms | 双板通信、超电控制帧发送 |
| `INS_Task` | osPriorityLow | 1024 | 1ms | 惯导姿态解算（四元数 EKF，BMI088） |
| `Gimbal_Task` | osPriorityIdle | 512 | 1ms | 云台 Pitch/Yaw 控制 |
| `Key_LED_Task` | osPriorityIdle | 256 | 10ms | 按键扫描 + RGB LED |
| `Buzzer_Task` | osPriorityIdle | 256 | 1ms | 蜂鸣器音效 |
| `UI_Task` | osPriorityIdle | 512 | 1ms | 裁判系统 UI 刷新 |

> `Shoot_Task` / `Aim_Task` 已注释，发射与自瞄逻辑在其他任务中直接调用。

### 通信总线

| 总线 | 外设/设备 | 用途 |
|------|-----------|------|
| FDCAN1 | 关节电机(DAMIAO/LB) + 超电 | 腿长 MIT 力控 / 功率（超电 CAN ID 0x030/0x031） |
| FDCAN2 | 轮电机(M3508) + 云台 Pitch | 底盘 250Hz 力矩控制 |
| FDCAN3 | 云台 Yaw + 摩擦轮 + 拨弹 + 外部 IMU | 云台/发射/姿态 |
| SPI4 | BMI088 | 六轴 IMU 姿态 |
| USART1 | SBUS | 遥控器 |
| USART2/3 | RS485 | 扩展通信 |
| UART4/5 | 裁判系统 | 数据交互 / 功率限制 |
| USB CDC | VOFA+ | 浮点数据可视化调试 |

### 关键数据结构

- **`RoboControl_StructTypeDef`**（RoboControl.h）：机器人全局状态体——底盘目标速度 Vx/Vy/Wz、腿长/腿摆角、云台角度、功率状态、各模块状态、功能标志。各任务共享。
- **`Chassis_Control_StructTypeDef`**（Chassis.h）：底盘运控核心状态体——10 个 LQR 状态量、虚拟腿长/摆角及各级导、雅可比 `Jl/Jr/Jt`、PID 集合、位移积分、各机动状态机标志、氮气弹簧模型左右各一。

---

## 底盘运控核心（项目技术重点）

> 改动底盘 / 写文档 / 介绍项目时优先围绕以下内容。整体框架见上方任务体系与下方状态调度。

### 10 大创新点

1. **氮气弹簧动态补偿（GasSpring）** — 串联腿内置氮气弹簧，回复力随腿长非线性（0.14m→90~100N，0.38m→159~180N，左右腿各一张标定表 `spring_table_L/R[]`）。`GasSpring_GetForce()` 按当前腿长分段线性插值得弹簧推力，`safety_force_limit=200N` 限幅，作为**动态前馈** `Spring_comp` 注入腿长力控 `F_L/F_R`，抵消弹簧非线性，使腿长力控等效为纯惯性对象。是无弹簧补偿时腿长 PID 被非线性干扰难收敛问题、也是跳跃/上台阶/起立等一切腿长动作平稳的前提。
2. **变参数 LQR** — 独立腿（轮腿共生）用 LQR 状态反馈，但腿长 0.15~0.35m 大幅变化，固定增益难兼顾全工作点。`Poly_Coefficient[12][4]` 存 12 个增益的**三次多项式**拟合系数，`calucateK()/LQR_K_calc()` 每周期按 `Ave_Leg_Length` 插值出实时 `LQR_K[12]`，再 `calucateLQR_L/R()` 算轮/关节力矩。10 状态：左右腿摆角/角速度/角加速度(差分) + 位移/速度 + 机体 pitch/角速度。
3. **VMC 虚拟模型控制** — 上层产出「腿力 `F_L/F_R` + 关节摆角力矩 `Torque_Joint`」两组虚拟量，经力雅可比 `Jt_l/Jt_r` 映射到 4 个关节电机力矩 `TP[4]`（`calucateTp()`）。正运动学（哈工程 5 连杆方案）由关节电机总角解出虚拟腿长/摆角与 `Jt`。配 `Gravity_Comp=90N` 重力前馈、`Roll_Leg_PID` 横滚左右差动、`Tp_Comp_PID` 防劈叉。
4. **跳跃算法（4 态 + 3 档）** — `Jump_Level`(1/2/3)→蹬伸腿长 0.20/0.25/0.33m；`Jump_State` 状态机：**0 降腿蓄力**(收腿压弹簧，双腿<0.16m 累计30tick) → **1 斜坡蹬腿起跳**(`F=Spring_comp+K_slope·330N`，`K_slope+=25·dt` 缓加力防抖，双腿到目标长 15tick) → **2 空中收腿**(收腿PID、关节只留摆角LQR×0.25、轮置0，稳定40tick，阈值定滞空时长) → **3 复位**。空中由左右腿摆角 LQR 独立防劈叉。
5. **自动上台阶** — `Up_Step_Dection()`：进入检测即 `Leg_Length=0.35` 伸最高腿并**解除腿长限速**(`V_limit=2.3`)，便于短距加速磕台阶；双腿摆角均>14° **且**角速度均>1.7 判「磕到台阶」。`Up_Step_Func()` 态0 **后摆腿±55°+收腿<0.18m**（轮置0）→ 态2 复位。
6. **离地检测** — 不解力传感器，`Off_ground_Detection()` 用动力学估算轮端支持力 `Fn = M_wheel·g + P`（`P=(Gravity_Comp+腿长速度环输出)·cosθ + Tp·sinθ/L0`），`Fn<50N` 判该轮离地；**双腿同时离地 且 `MotionAccel_n_Z<-5`** 才确认（双重防误判）。触发：轮置0、腿长切**软着陆恒力**`Soft_landing_Comp_F=120N`、关节只保摆角LQR，平稳落地。跳跃/上台阶期屏蔽，起立前不检测。
7. **翻倒自起 + 缓起立** — `Recover_from_Ground()` 按 pitch 正负判**前栽/后仰**；态0 **平齐双腿前摆**(轮置0，摆角速度环归中) → 态2 **收腿最短0.15**(<0.17m 累计200tick) → 态3 恢复 `Chassis_STATIC`+`Allow_to_Stand=1`。`Smooth_Restand_Func()` 缓起立：倒地后先**缓慢收腿**(腿<0.17m 累计100tick)才允许进常规运控，防一上电疯车。
8. **卡尔曼速度观测器** — `xvEstimateKF`(左右各一)：2 维卡尔曼，编码器速度=测量、IMU 加速度=输入，融合出稳定车速 `V_ave`，抑制打滑/编码器跳变。
9. **功率反解限幅 + 腿长限速** — `Chassis_Power_MAX = 超电实时功率 + 裁判功率上限` 反解极限 `V/Wz` 限幅；`V_limit`(2.3→1.3 随腿长0.22→0.36m线性递减)、`Wz_limit` 随腿长降，上台阶态解除 V 限速；SPIN 态入态受 `|车速|<2.2` 门控防翻车。
10. **小陀螺平移解耦（SPIN）** — SPIN 态 Vx/Vy 按云台 yaw 旋转分解（`-45°` 相位补偿 `offset_Comp`，四象限+正反转全组合），绕敌平移跟随；转速随 `Chassis_Speed_Level` 5 档缩放（6/7.5/8.5/10/12）。

### 底盘运控数据流（1ms，Chassis_Task）

```
Get_Control_Data()  取目标/反馈：位移、车速、pitch、关节总角，装配 state_lqr[6]
Calculated_displacement()  编码器总角 → 车轮位移/速度
xvEstimateKF()            轮速+IMU加速度 → 滤波车速 V_ave
calculateValues_Q_l/r()   关节总角 → 虚拟腿长/摆角 + 力雅可比 Jt（哈工程正运动学）
State_Filter()            Z轴加速度 3点中值+低通 双级滤波
add_six_state()           装配 10 状态 x
calucateK() + calucateLQR_L/R()   变参数 LQR → 轮力矩 + 关节摆角力矩
Jump_Control() / Off_ground_Detection() / Up_Step_*() / Recover_from_Ground()  特殊机动叠加力/力矩
calucateTp()              Jt·F + Jt·Tj → 4 关节力矩 TP[4]
Output_Current()          力矩×K_gear2chain → 关节 MIT + 轮电流 → CAN
```

所有特殊机动以「力或关节力矩叠加量」形式注入同一个 `calucateTp()` 雅可比分解点，与常规 LQR 共用一条力矩合成端——机动只改写 `F_L/F_R` 与 `Torque_Joint` 取值来源，不另起控制环。

## 机器人状态调度（Robo_Task，RoboControl.c）

操控→状态调度层：把遥控/键鼠输入解码为整体目标 `Robo_Enable/Vx/Vy/Wz/Leg_Length/Leg_Angle` + 各模块状态 + 功能标志触发。

- **底盘状态机**：`Chassis_OFF / STATIC / FOLLOW / SPIN / DASH / Bench`。
- **功能标志触发**（遥控/键鼠，写入 `Chassis_Control_Struct`）：跳跃 `Jump_Flag`(鼠标中键/右键/滑轮下滑右推)、上台阶 `UpStep_Dect_Flag`(按C/滑轮下滑左推)、翻倒自起 `Recover_from_ground_Flag`(Ctrl+R/滑轮下滑左拉)、缓起立 `Smooth_restand_Flag`(失能态右键/Ctrl+E)、一键清标志防疯车(Ctrl+Z/滑轮下滑左拉)。
- **`Get_Chassis_Wz()`**：OFF→0；STATIC/FOLLOW→转向 PID 朝目标方向（输出随腿长降限）；SPIN→恒速正反转（5 档缩放）。挡位 `Chassis_Speed_Level` 1~5。
- **失能/重启**：`Robo_Stop()`（清全部功能标志、`Allow_to_Stand=0`）、`Robo_Restart()`（默认 `Chassis_STATIC`，默认进入翻倒自起流程）。

## 通信协议（Communicate.h，双板互通）

帧结构（`__packed` 位域压缩）：
- **上行（云台板→底盘）**：`Yaw_Errx100 + Gimbal_Yaw_TotalAnglex100` + 8 位域（快门/云台/摩擦轮档）+ Pitch/偏移。
- **下行（底盘→云台板）**：翻倒自起标志 + 底盘里程计 `Vx/Vy x100` + 冷却量；另有裁判上行帧 `Shoot_Qmax/Qnow + robot_id + 初速x100`。
- **遥控帧**：`Remote_Pack1`(摇杆+滑键Mode) + `Remote_Pack2`(键鼠+自订功能键位域)。

---

## 常用操作

- **构建**：Keil 打开 `MDK-ARM/Kawashiro_Frame_G474.uvprojx`，先装软件包 `Keil.STM32G4xx_DFP` + `ARM.CMSIS-DSP`（`arm_math.h` 不随仓库存），Build (F7)。产物在 `MDK-ARM/`。从零构建/真机复现全流程见 `docs/依赖编译指南.md`。
- **烧录**：J-Link / DAP-Link 连接，Download (F8)。
- **硬件配置修改**：改外设引脚经 STM32CubeMX 打开 `Kawashiro_Frame_G474.ioc` 重新生成；`/* USER CODE BEGIN/END */` 内代码保留。
- **调试**：USB CDC 虚拟串口 + VOFA+（JustFloat 浮点波形）；DWT 高精度时钟测耗时。

### 调试标定（Chassis.c 顶部）

| 参数 | 值 | 位置 |
|------|-----|------|
| `Gravity_Comp` | 90N | 腿端重力前馈 |
| `Detect_Force` | 50N | 离地支持力阈值 |
| `Soft_landing_Comp_F` | 120N | 离地软着陆恒力 |
| `spring_table_L/R` | 腿长-力标定表 | 氮气弹簧补偿 |
| `Poly_Coefficient` | LQR 三次多项式系数 | 含 3 组 Q/R 备选 |
| `angle_error_gate / angle_dot_error_gate` | 14° / 1.7 | 上台阶磕碰检测 |

## 代码风格

- 遵循全局 CLAUDE.md 的代码风格规范（Allman 大括号、下划线实词命名、`Limit_float(&x, max, min)` 等）。
- 结构体 typedef 后缀 `_StructTypeDef`、枚举 `_EnumTypedef`、控制结构体 `TypeDef`。
- 调试/实验留档的注释代码按排版规范正常保留，不受"反过度设计"约束。

## Git

- 仓库：`https://github.com/Zhouzzw/RM-WheelLeg`（分支 main）。
- 提交：`chore/feat/fix` 前缀 + 中文描述。
- 忽略：`.gitignore` 排除 .o/.d/.hex/.map/.uvguix 等编译和个人文件。
- 流程：全局 CLAUDE.md——单人两层分支（master ← feature/fix），大改动才开分支，合入必须 `--no-ff` 禁 rebase，合 master + push 需确认。