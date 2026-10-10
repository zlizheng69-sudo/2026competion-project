# STM32H750 平衡小球小车（2026 电赛 H 题 · 电控部分）

在 STM32H750VBT6 上实现一台「小车 + 摆杆」的平衡小球系统：视觉模块回传小球位置，
主控闭环控制摆杆倾角，把小球稳定在管内目标位置，同时完成底盘循迹行驶与定点停车。

> 本项目为**裸机实现**（无 RTOS）：主循环状态机 + 定时器中断节拍。

## 硬件与工具链

| 项 | 值 |
|---|---|
| MCU | STM32H750VBT6（Cortex-M7，HSE 25MHz 经 PLL1 → **SYSCLK 480MHz**，VOS0） |
| 姿态 | ICM42688（**I2C1**）+ 互补滤波 + 卡尔曼滤波 |
| 视觉 | 视觉模块串口（**USART3** 115200，4 字节定长帧回传小球 X 位置） |
| 平衡机构 | 闭环步进电机（**USART1** 115200 指令）+ 齿轮齿条 → 摆杆 |
| 底盘 | 双直流电机（**TIM1** 10kHz PWM + 方向 IO）+ 编码器（**TIM2**）+ 4 路红外循迹 |
| 显示 | SSD1306 OLED 128×64（**I2C2**，地址 0x3C） |
| 调参链路 | **UART4** 9600：命令行调参 + printf 回显（`_write` 重定向） |
| 控制节拍 | **TIM6** 50Hz（20ms）中断置标志，主循环执行平衡控制 |
| 工具链 | VS Code + STM32CubeMX + CMake + Ninja + arm-none-eabi-gcc |

## 系统架构

```
  视觉模块 ──USART3(0xAA|dx|det|cksum)──► camera.c ──► 小球位置 dx / detected
                                                            │
  ICM42688 ──I2C1──► ICM42688.c ──► IMU.c(互补滤波) ──► 姿态角 pitch/roll/yaw
                                                            │
  编码器 ──TIM2──► encoder.c ──► speed_ctrl.c(前馈+PID) ──► 底盘双电机 PWM(TIM1)
                                                            │
  红外循迹 ──GPIO──► tracking.c ──► chassis.c(循迹/刹车) ────┤
                                                            ▼
                                       main.c 状态机 (IDLE→RUN→BRAKING→DONE)
                                                            │
              balance.c(位置环 + 球速环 + 前馈) ──► motor.c ──USART1──► 闭环步进 ──► 摆杆
                                                            │
                                       oled.c 显示 + serialcmd.c 串口调参(UART4)
```

## 模块职责

| 模块 | 职责 |
|---|---|
| `main.c` | 初始化、主状态机、题目模式（2~6）调度、20ms 平衡节拍、200ms 调试输出 |
| `ICM42688.c` / `IMU.c` | 六轴读取与互补滤波姿态解算（α=0.966，yaw 死区 + 低通） |
| `kalman.c` | 像素级卡尔曼滤波（位置/速度四状态），视觉目标平滑与预测 |
| `camera.c` | 视觉帧状态机、校验和、收包/错包统计 |
| `balance.c` | 齿轮齿条几何换算、位置环 + 球速环 + 前馈、轨迹剖面控制、稳定判据 |
| `motor.c` | 闭环步进电机 UART 协议封装（配置/使能/位置/速度/停止） |
| `encoder.c` / `speed_ctrl.c` | 编码器 10ms 测速；速度环 PID + 左右轮标定前馈 |
| `chassis.c` / `tracking.c` | 底盘运动控制、停车与刹车；4 路红外循迹 |
| `gimbal.c` | 云台轴角度目标与使能控制 |
| `pid.c` | 通用 PID：积分限幅、输出限幅、微分先行 |
| `oled.c` / `serialcmd.c` | 状态显示；串口命令行实时调参 |
| `pid_core.py` / `pid_server.py` | 上位机：TCP 接收实际值、实时绘图、PID 自动整定 |
| `test_sender.py` | 模拟下位机数据源，用于上位机自测 |
| `trajectory_controller_plan.md` | 轨迹速度剖面控制器设计说明 |

## 串口调参命令（UART4，9600，以 `\n` 结尾）

支持绝对赋值 `key:value` 与相对调整 `key+value` / `key-value`，每次设置均回显：

| 命令 | 作用 |
|---|---|
| `mode:n` | 切换题目模式（2~6） |
| `target:x` | 设定小球目标位置（cm） |
| `bkp` `bki` `bkd` `bkv` `bff` | 平衡环 Kp / Ki / Kd、球速增益、前馈增益 |
| `skp` `ski` `skd` | 底盘速度环左右轮同步整定 |
| `lskp` `rskp` `lski` `rski` `lskd` `rskd` | 左右轮单独整定（补偿机械差异） |
| `pkp` | 位置环增益 |
| `bsmode:n` | 平衡速度模式开关 |
| `dir:n` | 直接给定模式 |
| `kp` `ki` `kd` `enable` | 当前云台轴 PID 与使能 |

## 关键设计

### 1. 视觉帧解析（`camera.c`）
定长 4 字节：`0xAA | dx(int8) | det('Y'/'N') | checksum`，其中
`checksum = (0xAA + dx + det) & 0xFF`。逐字节状态机解析，校验失败计入错包统计；
`fresh` 标志保证同一帧不会被重复消费。

### 2. 姿态解算（`IMU.c`）
互补滤波：`angle = α·(angle + gyro·dt) + (1-α)·accel_angle`，α = 0.966；
俯仰/横滚由加速度计长期修正、陀螺仪提供动态响应。
yaw 采用陀螺积分，加速度计无法观测 yaw，故用死区（0.5 dps）抑制零漂 + 一阶低通（β=0.15）。

### 3. 摆杆角度换算（`balance.c`）
齿轮齿条 + 摆杆机构：

```
DEG_PER_PULSE = 360 / 3200 × (19.5 / 225) ≈ 0.00975°   （每脉冲对应的摆杆倾角）
```

球加速度约 12.2 cm/s²/°（实测标定），据此把期望加速度换算为倾角与电机脉冲数，并限幅输出。

### 4. 轨迹速度剖面控制（替代纯 PID）
纯 PID 的矛盾：Kp 大 → 球速快 → 摄像头丢目标 → 失控。
解法是**先限速、再控速**：按位置给出安全速度上限
`v_safe(x) = sqrt(2·a_max·|x|)·safety`，叠加 `v_max_limit` 防止摄像头跟丢，
再用以速度误差为主的控制器输出倾角，自然形成「先加速、后减速」的梯形速度曲线。
完整推导与可调参数见 `trajectory_controller_plan.md`。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
# 产物：build/Debug/mpu6050.elf
```

## 已知问题与待改进

- 比赛时间内**未实现稳定平衡**：受小球动力学建模与串级 PID 整定经验限制，
  收敛性不足 —— 这也是本仓库保留轨迹剖面方案（`trajectory_controller_plan.md`）的原因
- 视觉帧率约 16 Hz，是控制带宽的主要瓶颈，目前靠限速策略规避
- 平衡参数依赖实车标定（球加速度 `a/°`、安全系数），更换机构需重新标定
- 未启用 RTOS：当前为裸机超级循环 + 中断，后续计划改为多任务（控制 / 通信 / 保护分离）
- 视觉通信无超时降级：丢帧时 `detected` 置 0，建议增加超时保护策略
- `main.c` 中状态机与题目模式耦合较紧，可按题目拆分为独立策略模块

## 目录结构

```
Core/
├── Inc/   ICM42688.h  IMU.h  kalman.h  balance.h  motor.h  pid.h  speed_ctrl.h
│          encoder.h  chassis.h  tracking.h  gimbal.h  camera.h  oled.h
│          serialcmd.h  main.h  （以及 CubeMX 生成的外设头文件）
└── Src/   ICM42688.c  IMU.c  kalman.c  balance.c  motor.c  pid.c  speed_ctrl.c
           encoder.c  chassis.c  tracking.c  gimbal.c  camera.c  oled.c
           serialcmd.c  main.c  （以及 gpio.c / i2c.c / tim.c / usart.c 等生成文件）
cmake/  Drivers/  build/
CMakeLists.txt  CMakePresets.json  mpu6050.ioc  startup_stm32h750xx.s
STM32H750XX_FLASH.ld  pid_core.py  pid_server.py  test_sender.py
trajectory_controller_plan.md
```
