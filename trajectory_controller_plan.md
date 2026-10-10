# 轨迹速度-剖面控制器（替代纯 PID）

## 背景

纯 PID 无法工作的根本矛盾：Kp 大 → 球速快 → 摄像头丢目标 → 失控。必须在控制层面**主动限制球速**，保证摄像头始终能跟踪。

## 核心思路

用**速度剖面控制**替代纯 PID：

1. 定义每个位置的"安全速度上限"：`v_safe(x) = sqrt(2 × a_max × |x|) × safety`
   - 离中心远 → 允许高速 → 加速阶段
   - 靠近中心 → 必须减速 → 减速阶段
   - 自然形成"先加速、后减速"的梯形速度曲线
2. 叠加最大速度限制 `v_max_limit`，防止摄像头跟丢
3. 控制倾角去追踪期望速度，而不是直接追踪位置

## 物理常数

```
a_max  = 12.2 × 5.07° = 61.8 cm/s²    (最大倾角下的球加速度)
θ/pulse = 0.00975°                     (每脉冲对应摆杆角度)
a/°    = 12.2 cm/s²/°                  (每度倾角的球加速度)
```

## 控制律

```c
v_safe    = sqrt(2 × a_max × |x|) × safety_factor
v_desired = -sign(x) × min(v_safe, v_max_limit)
v_error   = v_desired - v_filt          // v_filt = 低通滤波后的球速
a_cmd     = K_vel × v_error + K_pos × x // 速度误差为主 + 弱位置修正
θ_deg     = a_cmd / 12.2               // 所需倾角
pulses    = θ_deg / 0.00975            // 电机脉冲
clamp to  ±520
```

## 改动文件

| 文件 | 改动 |
|------|------|
| `Core/Inc/balance.h` | 新增 `TrajParams` 结构体、`Balance_TrajCtrl()`、`Balance_SetTrajParams()` |
| `Core/Src/balance.c` | 实现轨迹控制器 + 球速低通滤波 |
| `Core/Src/main.c` | 题1 加球速计算 + `g_traj_ctrl` 标志路由 |
| `Core/Src/serialcmd.c` | 新增 `btraj`、`btsf`、`btvm`、`btkv`、`btkp` 命令 |

## 可调参数（默认值）

| 参数 | 命令 | 默认 | 说明 |
|------|------|------|------|
| safety_factor | `btsf` | 0.75 | 安全系数，越高越保守 |
| v_max_limit | `btvm` | 20.0 | 最大球速 cm/s，受摄像头限制 |
| K_vel | `btkv` | 3.0 | 速度误差增益 |
| K_pos | `btkp` | 0.3 | 弱位置修正增益 |
| vel_alpha | — | 0.3 | 速度低通滤波系数 |

## 球速计算

```c
// 摄像头每帧（~16Hz）:
v_raw = (x_k - x_{k-1}) / dt_cam   // cm/s
v_filt = 0.3 × v_raw + 0.7 × v_filt // 低通平滑
```

## 使用方法

1. 编译烧录
2. 题1 串口发 `btraj:1` 启用轨迹模式
3. 放球偏离 5cm，观察是否"先加速后减速"收敛到中心
4. 串口调参：`btkv+1`、`btsf+0.05` 等
5. `btraj:0` 切回 PID 模式
