"""
测试发送脚本 — 模拟一个二阶物理系统（如电机/温控/倒立摆），
将 PID 输出作为控制信号，计算系统响应，发送回服务器形成闭环。

用法：
    python test_sender.py                # 使用默认模拟
    python test_sender.py --manual       # 手动输入数值
"""

import socket
import json
import time
import math
import random
import sys
import argparse

HOST = "127.0.0.1"
PORT = 9999
DT = 0.02  # 控制周期 20ms


class SimulatedPlant:
    """
    模拟二阶系统:  G(s) = K / (tau*s^2 + 2*zeta*tau*s + 1)
    离散化后用欧拉方法递推。
    叠加少量噪声模拟传感器。
    """

    def __init__(self, gain=1.0, tau=0.5, zeta=0.3, noise_std=0.05, initial=0.0):
        self.gain = gain
        self.tau = tau
        self.zeta = zeta
        self.noise_std = noise_std

        # 状态变量
        self.y = initial       # 输出
        self.dy = 0.0          # 一阶导数
        self.ddy = 0.0         # 二阶导数

    def step(self, u: float, dt: float) -> float:
        """输入控制信号 u，返回测量值 y"""
        # 二阶系统: tau^2 * y'' + 2*zeta*tau * y' + y = K * u
        # => y'' = (K*u - y - 2*zeta*tau*y') / tau^2
        self.ddy = (self.gain * u - self.y - 2 * self.zeta * self.tau * self.dy) / (self.tau ** 2 + 1e-6)
        self.dy += self.ddy * dt
        self.y += self.dy * dt

        # 加传感器噪声
        noise = random.gauss(0, self.noise_std)
        return self.y + noise

    def disturb(self, amount=5.0):
        """施加扰动"""
        self.y += amount


def run_simulation():
    """自动模拟模式"""
    plant = SimulatedPlant(gain=1.0, tau=0.5, zeta=0.3, noise_std=0.1, initial=0.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"[模拟] 连接服务器 {HOST}:{PORT} ...")
    sock.connect((HOST, PORT))
    print("[模拟] 已连接，开始闭环控制仿真。")
    print(f"[模拟] 目标值见服务器设定，当前初始值 = {plant.y:.2f}")
    print()

    buf = ""
    step_count = 0

    try:
        while True:
            # 发送当前测量值
            msg = json.dumps({"value": round(plant.y, 4), "dt": DT}) + "\n"
            sock.sendall(msg.encode("utf-8"))

            # 接收 PID 输出
            try:
                sock.settimeout(0.5)
                raw = sock.recv(4096)
                buf += raw.decode("utf-8")
            except socket.timeout:
                pass

            # 解析响应
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    resp = json.loads(line)
                    pid_out = resp["output"]
                    # 用 PID 输出驱动模拟系统
                    plant.step(pid_out, DT)
                except (json.JSONDecodeError, KeyError):
                    pass

            step_count += 1
            if step_count % 50 == 0:
                print(f"[模拟] t={step_count*DT:.1f}s  y={plant.y:.3f}  "
                      f"Kp={resp.get('kp','?'):.3f} Ki={resp.get('ki','?'):.3f} Kd={resp.get('kd','?'):.3f}")

            # 每隔 5 秒施加一次随机扰动
            if step_count % 250 == 0:
                disturbance = random.uniform(-10, 10)
                plant.disturb(disturbance)
                print(f"[模拟] >>> 施加扰动: {disturbance:+.2f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[模拟] 仿真结束。")
    finally:
        sock.close()


def run_manual():
    """手动输入模式"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print(f"[手动] 已连接 {HOST}:{PORT}")
    print("[手动] 输入数值后回车发送，Ctrl+C 退出。")
    print()

    try:
        while True:
            line = input("实际值 > ").strip()
            if not line:
                continue
            try:
                value = float(line)
            except ValueError:
                print("请输入有效数字")
                continue

            msg = json.dumps({"value": value, "dt": DT}) + "\n"
            sock.sendall(msg.encode("utf-8"))
    except KeyboardInterrupt:
        print("\n[手动] 结束。")
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description="PID 测试客户端")
    parser.add_argument("--manual", "-m", action="store_true", help="手动输入模式")
    parser.add_argument("--host", default=HOST, help=f"服务器地址 (默认: {HOST})")
    parser.add_argument("--port", type=int, default=PORT, help=f"服务器端口 (默认: {PORT})")
    args = parser.parse_args()

    global HOST, PORT
    HOST = args.host
    PORT = args.port

    if args.manual:
        run_manual()
    else:
        run_simulation()


if __name__ == "__main__":
    main()
