"""
PID 自动调节主程序
- TCP Server 接收实际值
- 实时绘图：目标线 + 实际值曲线 + PID 输出
- 自适应调节 Kp / Ki / Kd

通信协议 (JSON over TCP):
  客户端发送: {"value": 123.45}
  可选字段:   {"value": 123.45, "dt": 0.02}

默认监听 0.0.0.0:9999
"""

import json
import socket
import threading
import time
import sys
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pid_core import PIDController, PIDAutoTuner

# ======================== 配置 ========================
HOST = "0.0.0.0"
PORT = 9999
SETPOINT = 50.0           # 目标值（图上水平线位置）
HISTORY_SIZE = 500        # 显示的历史点数
UPDATE_INTERVAL = 50      # 绘图刷新间隔 (ms)
TUNE_INTERVAL = 50        # 每 N 个采样点评估一次 PID 参数

INITIAL_KP = 2.0
INITIAL_KI = 0.1
INITIAL_KD = 0.5
# =======================================================


class SharedData:
    """线程安全的数据共享"""
    def __init__(self, maxlen=HISTORY_SIZE):
        self.lock = threading.Lock()
        self.times = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)
        self.outputs = deque(maxlen=maxlen)
        self.kp_hist = deque(maxlen=maxlen)
        self.ki_hist = deque(maxlen=maxlen)
        self.kd_hist = deque(maxlen=maxlen)
        self.start_time = time.time()

    def append(self, value: float, output: float, kp: float, ki: float, kd: float):
        with self.lock:
            t = time.time() - self.start_time
            self.times.append(t)
            self.values.append(value)
            self.outputs.append(output)
            self.kp_hist.append(kp)
            self.ki_hist.append(ki)
            self.kd_hist.append(kd)


def tcp_server(data: SharedData, pid: PIDController, tuner: PIDAutoTuner):
    """TCP 服务器线程：接收客户端发送的实际值"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen(1)
    print(f"[TCP] 监听 {HOST}:{PORT}，等待客户端连接...")

    conn = None
    addr = None
    buf = ""

    while True:
        try:
            if conn is None:
                conn, addr = sock.accept()
                print(f"[TCP] 客户端已连接: {addr}")
                buf = ""

            raw = conn.recv(4096)
            if not raw:
                print(f"[TCP] 客户端断开: {addr}")
                conn.close()
                conn = None
                continue

            buf += raw.decode("utf-8")

            # 解析 JSON 消息（支持粘包，\n 分隔）
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                    value = float(msg["value"])
                    dt = float(msg.get("dt", 0.02))

                    # PID 计算
                    output = pid.update(value, dt=dt)
                    error = pid.setpoint - value

                    # 喂入调节器
                    tuner.feed(error, output)
                    if tuner.should_tune():
                        new_kp, new_ki, new_kd = tuner.tune()
                        pid.kp, pid.ki, pid.kd = new_kp, new_ki, new_kd

                    # 存入共享数据
                    data.append(value, output, pid.kp, pid.ki, pid.kd)

                    # 返回 PID 输出给客户端（用于闭环仿真）
                    try:
                        resp = json.dumps({
                            "output": output,
                            "kp": pid.kp,
                            "ki": pid.ki,
                            "kd": pid.kd
                        }) + "\n"
                        conn.sendall(resp.encode("utf-8"))
                    except OSError:
                        pass

                except (json.JSONDecodeError, KeyError, ValueError):
                    pass  # 忽略格式错误的消息

        except (ConnectionResetError, ConnectionAbortedError, OSError):
            if conn:
                conn.close()
                conn = None
                print(f"[TCP] 连接断开: {addr}")
        except KeyboardInterrupt:
            break

    if conn:
        conn.close()
    sock.close()


def setup_plot():
    """创建 matplotlib 图形"""
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure("PID 自动调节监控", figsize=(14, 10))
    fig.patch.set_facecolor("#1a1a2e")

    # 子图 1：实际值 vs 目标值
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.set_facecolor("#16213e")
    ax1.set_title("实际值 vs 目标值", color="#e0e0e0", fontsize=14)
    ax1.set_ylabel("数值", color="#e0e0e0")
    ax1.tick_params(colors="#b0b0b0")
    ax1.grid(True, alpha=0.3, color="#4a4a6a")
    ax1.set_ylim(SETPOINT - 30, SETPOINT + 30)

    # 子图 2：PID 输出
    ax2 = fig.add_subplot(3, 1, 2)
    ax2.set_facecolor("#16213e")
    ax2.set_title("PID 控制输出", color="#e0e0e0", fontsize=14)
    ax2.set_ylabel("输出", color="#e0e0e0")
    ax2.tick_params(colors="#b0b0b0")
    ax2.grid(True, alpha=0.3, color="#4a4a6a")

    # 子图 3：Kp / Ki / Kd 变化曲线
    ax3 = fig.add_subplot(3, 1, 3)
    ax3.set_facecolor("#16213e")
    ax3.set_title("PID 参数自适应调节", color="#e0e0e0", fontsize=14)
    ax3.set_xlabel("时间 (s)", color="#e0e0e0")
    ax3.set_ylabel("参数值", color="#e0e0e0")
    ax3.tick_params(colors="#b0b0b0")
    ax3.grid(True, alpha=0.3, color="#4a4a6a")

    fig.tight_layout(pad=3.0)
    return fig, ax1, ax2, ax3


def animate(frame, data: SharedData, fig, ax1, ax2, ax3):
    """matplotlib 动画回调"""
    with data.lock:
        if len(data.times) == 0:
            return []

        times = list(data.times)
        values = list(data.values)
        outputs = list(data.outputs)
        kps = list(data.kp_hist)
        kis = list(data.ki_hist)
        kds = list(data.kd_hist)

    # --- 子图 1 ---
    ax1.clear()
    ax1.set_facecolor("#16213e")
    ax1.set_title("实际值 vs 目标值", color="#e0e0e0", fontsize=14)
    ax1.set_ylabel("数值", color="#e0e0e0")
    ax1.tick_params(colors="#b0b0b0")
    ax1.grid(True, alpha=0.3, color="#4a4a6a")

    # 目标线（水平横线）
    ax1.axhline(y=SETPOINT, color="#00ff88", linestyle="--", linewidth=2, alpha=0.8, label=f"目标值 = {SETPOINT:.1f}")
    # 实际值曲线
    ax1.plot(times, values, color="#ff6b6b", linewidth=1.5, label="实际值")
    ax1.legend(loc="upper right", facecolor="#1a1a2e", edgecolor="#4a4a6a", labelcolor="#e0e0e0")

    # 动态 Y 轴范围
    if values:
        vmin, vmax = min(values), max(values)
        pad = max((vmax - vmin) * 0.3, 5)
        ax1.set_ylim(min(vmin - pad, SETPOINT - pad), max(vmax + pad, SETPOINT + pad))

    # --- 子图 2 ---
    ax2.clear()
    ax2.set_facecolor("#16213e")
    ax2.set_title("PID 控制输出", color="#e0e0e0", fontsize=14)
    ax2.set_ylabel("输出", color="#e0e0e0")
    ax2.tick_params(colors="#b0b0b0")
    ax2.grid(True, alpha=0.3, color="#4a4a6a")
    ax2.plot(times, outputs, color="#4ecdc4", linewidth=1.5)
    if outputs:
        omin, omax = min(outputs), max(outputs)
        opad = max((omax - omin) * 0.2, 1)
        ax2.set_ylim(omin - opad, omax + opad)

    # --- 子图 3 ---
    ax3.clear()
    ax3.set_facecolor("#16213e")
    ax3.set_title("PID 参数自适应调节", color="#e0e0e0", fontsize=14)
    ax3.set_xlabel("时间 (s)", color="#e0e0e0")
    ax3.set_ylabel("参数值", color="#e0e0e0")
    ax3.tick_params(colors="#b0b0b0")
    ax3.grid(True, alpha=0.3, color="#4a4a6a")
    ax3.plot(times, kps, color="#ffd93d", linewidth=1.5, label="Kp")
    ax3.plot(times, kis, color="#6bcb77", linewidth=1.5, label="Ki")
    ax3.plot(times, kds, color="#4d96ff", linewidth=1.5, label="Kd")
    ax3.legend(loc="upper right", facecolor="#1a1a2e", edgecolor="#4a4a6a", labelcolor="#e0e0e0")

    return []


def main():
    print("=" * 55)
    print("  PID 自动调节监控程序")
    print(f"  目标值 (Setpoint): {SETPOINT}")
    print(f"  初始参数: Kp={INITIAL_KP}, Ki={INITIAL_KI}, Kd={INITIAL_KD}")
    print(f"  TCP Server: {HOST}:{PORT}")
    print("=" * 55)
    print()
    print("  客户端连接后，按以下 JSON 格式每行发送数据：")
    print('  {"value": 48.2}')
    print('  {"value": 49.1, "dt": 0.02}')
    print()

    # 初始化 PID 和调谐器
    pid = PIDController(kp=INITIAL_KP, ki=INITIAL_KI, kd=INITIAL_KD,
                        setpoint=SETPOINT, output_min=-100, output_max=100)
    tuner = PIDAutoTuner(kp=INITIAL_KP, ki=INITIAL_KI, kd=INITIAL_KD,
                         kp_range=(0.0, 100.0), ki_range=(0.0, 20.0), kd_range=(0.0, 50.0),
                         adapt_rate=0.08)
    tuner._tune_interval = TUNE_INTERVAL

    data = SharedData()

    # 启动 TCP 服务线程
    server_thread = threading.Thread(target=tcp_server, args=(data, pid, tuner), daemon=True)
    server_thread.start()

    # 设置绘图
    fig, ax1, ax2, ax3 = setup_plot()

    # 动画循环
    ani = animation.FuncAnimation(
        fig, animate, fargs=(data, fig, ax1, ax2, ax3),
        interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
    )

    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n[退出] 程序终止")
        sys.exit(0)


if __name__ == "__main__":
    main()
