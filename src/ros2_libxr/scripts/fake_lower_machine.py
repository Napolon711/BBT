#!/usr/bin/env python3
"""假下位机：在没有 STM32 的情况下把 ros2_libxr 桥跑起来。

两种用法：

1) 默认用伪终端（pty），自己打印从设备路径，另开终端把桥指过去：
       python3 fake_lower_machine.py
       输出 /dev/pts/N → ros2 run ros2_libxr ros2_libxr_node --ros-args -p port:=/dev/pts/N

2) 直接用 pty 起桥 + 自己（一条命令看全链路）：
       python3 fake_lower_machine.py --spawn-bridge

3) 灌真实串口（需要 root 或用 socat 造一对）：
       python3 fake_lower_machine.py --port /dev/ttyUSB0

CRC 与线格式严格照 LibXR 实现，和 C++ 侧做过逐字节对比。
"""

import argparse
import math
import os
import pty
import random
import struct
import subprocess
import sys
import threading
import time
import tty

# ---------------------------------------------------------------- CRC

def _mk_table(poly, width):
    tbl = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ poly if c & 1 else c >> 1
        tbl.append(c & ((1 << width) - 1))
    return tbl


_T8 = _mk_table(0x8C, 8)
_T16 = _mk_table(0x8408, 16)
_T32 = _mk_table(0xEDB88320, 32)


def crc8(data: bytes) -> int:
    c = 0xFF
    for b in data:
        c = _T8[(c ^ b) & 0xFF] ^ (c >> 8)
    return c & 0xFF


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c = _T16[(c ^ b) & 0xFF] ^ (c >> 8)
    return c & 0xFFFF


def crc32(data: bytes) -> int:
    c = 0xFFFFFFFF
    for b in data:
        c = _T32[(c ^ b) & 0xFF] ^ (c >> 8)
    return c & 0xFFFFFFFF  # 注意：没有最终异或，和 zlib.crc32 不同


assert crc8(b"123456789") == 0x0B
assert crc16(b"123456789") == 0x6F91
assert crc32(b"ahrs_quaternion") == 0xA6351AEA

# ---------------------------------------------------------------- SharedTopic

TOPIC_CRC = {
    "ahrs_quaternion": crc32(b"ahrs_quaternion"),
    "yawmotor_angle": crc32(b"yawmotor_angle"),
    "chassis_state": crc32(b"chassis_state"),
    "ref_frame": crc32(b"ref_frame"),
    "chassis_data": crc32(b"chassis_data"),
    "chassis_mode": crc32(b"chassis_mode"),
    "ref_tx": crc32(b"ref_tx"),
    "fold_cmd": crc32(b"fold_cmd"),
}
BY_CRC = {v: k for k, v in TOPIC_CRC.items()}

PREFIX = 0x5A


def pack(topic: str, payload: bytes, ts_us: int) -> bytes:
    n = len(payload)
    head = struct.pack("<B", PREFIX) + n.to_bytes(3, "big")
    head += struct.pack("<I", TOPIC_CRC[topic])
    head += ts_us.to_bytes(8, "big")
    head += bytes([crc8(head)])
    body = head + payload
    return body + bytes([crc8(body)])


class Parser:
    """增量解包，坏字节自恢复。"""

    def __init__(self):
        self.buf = bytearray()
        self.stats = {"ok": 0, "bad": 0, "drop": 0}

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(bytes([PREFIX]))
            if i < 0:
                self.stats["drop"] += len(self.buf)
                self.buf.clear()
                return out
            if i > 0:
                self.stats["drop"] += i
                del self.buf[:i]
            if len(self.buf) < 18:
                return out
            n = int.from_bytes(self.buf[1:4], "big")
            if n > 4096:
                self.stats["bad"] += 1
                del self.buf[:1]
                continue
            total = 18 + n
            if len(self.buf) < total:
                return out
            frame = bytes(self.buf[:total])
            if crc8(frame[:16]) != frame[16] or crc8(frame[:total - 1]) != frame[total - 1]:
                self.stats["bad"] += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            self.stats["ok"] += 1
            name = BY_CRC.get(struct.unpack("<I", frame[4:8])[0], "?%08X" % struct.unpack("<I", frame[4:8])[0])
            out.append((name, frame[17:total - 1],
                        int.from_bytes(frame[8:16], "big")))

# ---------------------------------------------------------------- 裁判帧

CMD_GAME_STATUS, CMD_ROBOT_HP, CMD_FIELD_EVENT = 0x0001, 0x0003, 0x0101
CMD_ROBOT_STATUS, CMD_POWER_HEAT, CMD_ROBOT_POS = 0x0201, 0x0202, 0x0203
CMD_ALLOWANCE, CMD_RFID, CMD_GROUND_POS = 0x0208, 0x0209, 0x020B
CMD_SENTRY_INFO, CMD_CUSTOM = 0x020D, 0x0311



def ref_frame(cmd_id: int, data: bytes, seq: int) -> bytes:
    n = len(data)
    h = bytes([0xA5]) + n.to_bytes(2, "little") + bytes([seq & 0xFF])
    h += bytes([crc8(h)])
    body = h + cmd_id.to_bytes(2, "little") + data
    return body + crc16(body).to_bytes(2, "little")


# 每个命令的真实发送频率（协议表 1-5），别按 50 Hz 灌，会把伪终端灌爆
CMD_PERIOD = {
    CMD_GAME_STATUS: 1.0,
    CMD_ROBOT_HP: 1.0 / 3,
    CMD_FIELD_EVENT: 1.0,
    CMD_ROBOT_STATUS: 0.1,
    CMD_POWER_HEAT: 0.1,
    CMD_ROBOT_POS: 1.0,
    CMD_ALLOWANCE: 0.1,
    CMD_RFID: 1.0 / 3,
    CMD_GROUND_POS: 1.0,
    CMD_SENTRY_INFO: 1.0,
    # 0x0311 解波分帧：每 3 s 一批，一批 8 帧连续发，这里用 1/3 s 触发
    CMD_CUSTOM: 3.0,
}


# 哪些命令走图传链路（USART6, bus=1），其余走常规链路（USART1, bus=0）
CMD_BUS = {CMD_CUSTOM: 1}


def build_round(t: float):
    """造一组自洽的 2027 下行帧。"""
    hp = 200 + int(200 * (0.5 + 0.5 * math.sin(t / 5)))
    frames = [
        (CMD_GAME_STATUS, struct.pack("<BHQ", (4 << 4) | 1, int(420 - t) & 0xFFFF, 0x1122334455667788)),
        # 0x0003：己方/对方视角，24 字节
        (CMD_ROBOT_HP, struct.pack("<HHHHhHHHHHHH",
                                   hp, 0, 180, 180, -35, hp, 300, 5000, 300, 5000, 120, 96)),
        # 0x0101：8 字节，R 前哨阵地控制状态 = 1（己方临时控制），B = 2（未控制）
        (CMD_FIELD_EVENT, struct.pack("<II", (1 << 28) | (2 << 30) | 1, 0)),
        # 0x0201：17 字节，含 float 初速度上限
        (CMD_ROBOT_STATUS, struct.pack("<BBHHHHHfB", 7, 3, hp, 400, 40, 200, 80, 30.0, 0b101)),
        # 0x0202：14 字节，前 8 字节保留
        (CMD_POWER_HEAT, struct.pack("<HHfHHH", 0, 0, 0.0, 60, 20, 0)),
        (CMD_ROBOT_POS, struct.pack("<fff", 8.0, 6.0, 90.0)),
        (CMD_ALLOWANCE, struct.pack("<HHHH", 250, 400, 120, 0)),
        # 0x0209：6 字节。bit22-24 = 己方隧道下段增益点
        (CMD_RFID, struct.pack("<IH", 1 << 0 | 1 << 22 | 1 << 2, 0)),
        (CMD_GROUND_POS, struct.pack("<ffIIffffII", 6.0, 7.0, 0, 0, 5.0, 6.0, 7.0, 5.0, 0, 0)),
        # 0x020D：14 字节。脱战(bit0)=0 战斗中，姿态=1 进攻
        (CMD_SENTRY_INFO, struct.pack("<IHQ", 0, (1 << 12) | (0 << 0), 30)),
    ]
    # 0x0311：分帧的解波数据。模拟"解波成功"：一批 5 帧连续发出。
    return frames


# ---------------------------------------------------------------- 主循环

class FakeStm32:
    def __init__(self, fd, spawn_bridge=False, pts=None, verbose=False):
        self.fd = fd
        self.parser = Parser()
        self.ts = 0
        self.seq = 0
        self.tx_drop = 0
        self.last_move = None
        self.rx_log = {}
        self.bridge = None
        if spawn_bridge:
            self.bridge = subprocess.Popen(
                ["ros2", "run", "ros2_libxr", "ros2_libxr_node", "--ros-args",
                 "-p", "port:=" + pts, "-p", "publish_raw:=true",
                 "-p", "radar_origin_x_m:=0.0", "-p", "radar_origin_y_m:=0.0"],
                stdout=None if verbose else subprocess.DEVNULL,
                stderr=None if verbose else subprocess.DEVNULL)

    def send(self, topic, payload):
        self.ts += 1000
        try:
            os.write(self.fd, pack(topic, payload, self.ts))
        except BlockingIOError:
            # 没人读（桥还没起 / 缓冲区满）：真实串口这里也是丢包，计数即可
            self.tx_drop += 1

    def pump_rx(self):
        try:
            data = os.read(self.fd, 65536)
        except (BlockingIOError, OSError):
            return
        if not data:
            return
        for name, payload, _ts in self.parser.feed(data):
            first = name not in self.rx_log
            self.rx_log[name] = self.rx_log.get(name, 0) + 1
            # 只打印每个话题的第一次，避免刷屏；ref_tx 每次都要看
            verbose = first or name == "ref_tx"
            if verbose and name == "ref_tx" and len(payload) >= 8:
                cmd, n, bus = struct.unpack_from("<HHB", payload, 0)
                body = payload[8:8 + n]
                print("[rx] ref_tx cmd=0x%04X len=%d bus=%d data=%s"
                      % (cmd, n, bus, body.hex()), flush=True)
                if cmd == CMD_CUSTOM:
                    act = (int.from_bytes(body[6:10], "little") >> 21) & 0x7 if len(body) >= 10 else 0
                    print("     -> 0x0120 姿态=%d 发弹累计=%d"
                          % (act, (int.from_bytes(body[6:10], "little") >> 2) & 0x7FF), flush=True)
            elif verbose and name == "fold_cmd":
                print("[rx] fold_cmd = %d" % payload[0], flush=True)
            elif verbose and name == "chassis_mode":
                print("[rx] chassis_mode = %d" % payload[0], flush=True)
            elif name == "chassis_data":
                vx, vy, wz = struct.unpack("<fff", payload[:12])
                self.last_move = (vx, vy, wz)

    def run(self, seconds):
        import atexit
        atexit.register(self.shutdown)
        t0 = time.time()
        nxt = {cmd: 0.0 for cmd in CMD_PERIOD}
        nxt_ahrs = nxt_yaw = nxt_cs = nxt_report = 0.0
        while True:
            t = time.time() - t0
            if t >= seconds:
                break
            for cmd, data in build_round(t):
                if cmd == CMD_CUSTOM:
                    continue
                if t >= nxt[cmd]:
                    nxt[cmd] = t + CMD_PERIOD[cmd]
                    self.seq += 1
                    raw = ref_frame(cmd, data, self.seq)
                    # RefFrameWire: bus, seq, len, tick_ms, raw[128]
                    wire = struct.pack("<BBHI", CMD_BUS.get(cmd, 0), self.seq & 0xFF,
                                       len(raw), int(t * 1000)) + raw
                    self.send("ref_frame", wire)
            if t >= nxt_ahrs:
                nxt_ahrs = t + 0.01
                self.send("ahrs_quaternion",
                          struct.pack("<ffff", 0.0, 0.0, math.sin(t), 1.0))
            if t >= nxt_yaw:
                nxt_yaw = t + 0.01
                self.send("yawmotor_angle", struct.pack("<f", random.uniform(-0.3, 0.3)))
            if t >= nxt_cs:
                nxt_cs = t + 0.05
                # ChassisStateWire: vx, vy, wz, fold_state, posture, buffer_energy_j, power_w, tick_ms
                self.send("chassis_state", struct.pack("<fffBBHHI", 0.5, 0.0, 0.1, 2, 1,
                                                       60, 45, int(t * 1000)))
            self.pump_rx()
            if t >= nxt_report:
                nxt_report = t + 2.0
                if self.last_move is not None:
                    print("[rx] chassis_data 最新 vx=%.2f vy=%.2f wz=%.2f"
                          % self.last_move, flush=True)
            time.sleep(0.005)
        print("\n[stats] 发出 ref_frame=%d  写丢=%d  收到 %s  解析 %s"
              % (self.seq, self.tx_drop, self.rx_log or "（无）", self.parser.stats),
              flush=True)
        self.shutdown()

    def shutdown(self):
        if self.bridge is not None and self.bridge.poll() is None:
            self.bridge.terminate()
            try:
                self.bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bridge.kill()
            self.bridge = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="写到这个真实串口（默认自建 pty）")
    ap.add_argument("--spawn-bridge", action="store_true",
                    help="自动起 ros2_libxr_node 并在结束时关掉")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--verbose", action="store_true",
                    help="把桥的日志直接��到当前终端")
    args = ap.parse_args()

    if args.port:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        pts = args.port
    else:
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        fd = master
        pts = os.ttyname(slave)
        print("假下位机在 %s" % pts, flush=True)
        if not args.spawn_bridge:
            print("另开终端：ros2 run ros2_libxr ros2_libxr_node --ros-args -p port:=%s" % pts,
                  flush=True)
        time.sleep(1.0)

    FakeStm32(fd, args.spawn_bridge, pts, args.verbose).run(args.seconds)


if __name__ == "__main__":
    main()
