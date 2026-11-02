#!/usr/bin/env python3
"""Isolated production bridge / PTY / ROS test; never opens a physical device."""
import importlib.util
import os
import pty
import resource
import struct
import subprocess
import sys
import time
import tty
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float64MultiArray, Float32, Int32, UInt8
from geometry_msgs.msg import Twist
from rm2027_interfaces.msg import GameStatus, RobotStatus, SentryCommand

root = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'fake', root/'src/ros2_libxr/scripts/fake_lower_machine.py')
fake = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fake)
exe = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    root/'install/ros2_libxr/lib/ros2_libxr/ros2_libxr_node')

# Choosing the side is mandatory; an old automatic selection cannot silently run.
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
for own_team in ('', 'auto', 'green'):
    args = [str(exe), '--ros-args']
    if own_team:
        args += ['-p', 'own_team:=' + own_team]
    invalid = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             timeout=3, text=True)
    assert invalid.returncode != 0 and 'own_team must be explicitly set' in invalid.stdout

rclpy.init()
node = rclpy.create_node('bridge_integration_test')
games, summaries, statuses, operators = [], [], [], []
node.create_subscription(GameStatus, '/referee/game_status', games.append, 10)
node.create_subscription(Float64MultiArray, '/referee/summary', summaries.append, 10)
node.create_subscription(RobotStatus, '/referee/robot_status', statuses.append, 10)
node.create_subscription(Float64MultiArray, '/referee/operator_cmd', operators.append, 10)
foldpub = node.create_publisher(UInt8, '/gimbal_fold_cmd', 10)
velpub = node.create_publisher(Twist, '/cmd_vel', 10)
modepub = node.create_publisher(Int32, '/move_mode', 10)
spinpub = node.create_publisher(Float32, '/cmd_spin', 10)
cmdpub = node.create_publisher(SentryCommand, '/referee/sentry_command', 10)
parameter_client = AsyncParameterClient(node, '/ros2_libxr_bridge')


def check_team(own_team):
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    os.set_blocking(master, False)
    parser = fake.Parser()
    received = []
    proc = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                data = os.read(master, 65536)
                if data:
                    received.extend(parser.feed(data))
            except BlockingIOError:
                pass

    def send(cmd, payload):
        raw = fake.ref_frame(cmd, payload, 1)
        wire = struct.pack('<BBHI', 0, 1, len(raw), 10) + raw.ljust(128, b'\0')
        os.write(master, fake.pack('ref_frame', wire, 10000))

    def sentry_frames():
        bodies = []
        for name, payload, _ in received:
            if name != 'ref_tx':
                continue
            cmd, length, bus = struct.unpack_from('<HHB', payload)
            assert (cmd, length, bus) == (0x0301, 10, 0), 'official RefTx wire header'
            body = payload[8:8 + length]
            subcmd, sender, receiver, bits = struct.unpack('<HHHI', body)
            assert subcmd == 0x0120 and receiver == 0x8080, 'official interaction body'
            assert sender == (7 if own_team == 'red' else 107), 'sender follows operator selection'
            assert (bits >> 2) & 0x7ff == 0, 'remote request must not duplicate a local purchase'
            bodies.append(bits)
        return bodies

    try:
        proc = subprocess.Popen([
            str(exe), '--ros-args', '-p', f'port:={os.ttyname(slave)}',
            '-p', f'own_team:={own_team}', '-p', 'referee_timeout_ms:=400',
            '-p', 'sentry_cmd_rate_hz:=10.0', '-p', 'sentry_cmd_hold_s:=0.2'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        spin(0.65)
        games.clear()
        summaries.clear()
        statuses.clear()
        operators.clear()
        assert proc.poll() is None
        assert parameter_client.wait_for_services(timeout_sec=1.0)
        change = parameter_client.set_parameters([
            Parameter('own_team', value='blue' if own_team == 'red' else 'red')])
        spin(0.1)
        assert change.done() and not change.result().results[0].successful, 'team is fixed for the run'
        assert not summaries, 'no summary before receiving required referee sources'
        frames = dict(fake.build_round(120))
        frames[fake.CMD_GAME_STATUS] = struct.pack('<BHQ', 0x41, 300, 0x1122334455667788)
        hp = bytearray(frames[fake.CMD_ROBOT_HP])
        struct.pack_into('<H', hp, 12, 151)  # red outpost
        struct.pack_into('<H', hp, 16, 263)  # blue outpost
        frames[fake.CMD_ROBOT_HP] = bytes(hp)
        robot_status = frames[fake.CMD_ROBOT_STATUS]

        # No referee identity has arrived: the selected sender is already known.
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=1))
        spin(0.16)
        assert not statuses and not summaries
        assert any((bits >> 13) & 15 == 1 for bits in sentry_frames()), 'missing referee ID must not veto purchase'

        # Even the first status with an unknown ID uses the operator's perspective.
        frames[fake.CMD_ROBOT_STATUS] = b'\x00' + robot_status[1:]
        for cmd, data in frames.items():
            send(cmd, data)
        spin(0.12)
        assert games and (games[-1].game_type, games[-1].game_progress,
                          games[-1].stage_remain_time) == (1, 4, 300)
        assert games[-1].sync_time_stamp == 0x1122334455667788, 'unaligned game timestamp'
        expected = [151, 1, 1, 263] if own_team == 'red' else [263, 2, 1, 151]
        assert summaries and len(summaries[-1].data) == 14
        assert [summaries[-1].data[i] for i in (5, 6, 7, 13)] == expected
        assert summaries[-1].data[10] == 5000, 'protocol ally-base field retains ally perspective'
        first_age = summaries[-1].data[12]
        spin(0.12)
        assert summaries[-1].data[12] > first_age, 'source age advances without new source frames'

        # An opposite, unknown, or matching referee ID never changes our side.
        wrong_id = 107 if own_team == 'red' else 7
        for reported_id in (wrong_id, 0, 7 if own_team == 'red' else 107):
            for cmd in (fake.CMD_GAME_STATUS, fake.CMD_ALLOWANCE,
                        fake.CMD_ROBOT_HP, fake.CMD_FIELD_EVENT):
                send(cmd, frames[cmd])
            send(fake.CMD_ROBOT_STATUS, bytes([reported_id]) + robot_status[1:])
            spin(0.06)
            assert statuses[-1].robot_id == reported_id, 'raw referee ID remains observable'
            assert [summaries[-1].data[i] for i in (5, 6, 7, 13)] == expected
        send(fake.CMD_ROBOT_STATUS, bytes([wrong_id]) + robot_status[1:])
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=2))
        spin(0.16)
        assert any((bits >> 13) & 15 == 2 for bits in sentry_frames()), 'wrong referee ID must not veto purchase'

        spin(0.45)
        count = len(summaries)
        spin(0.1)
        assert len(summaries) == count, 'stale referee must stop producing live summaries'

        # Robot status alone goes stale while the other required sources stay live.
        for cmd, data in frames.items():
            send(cmd, data)
        spin(0.08)
        for _ in range(6):
            for cmd, data in frames.items():
                if cmd != fake.CMD_ROBOT_STATUS:
                    send(cmd, data)
            spin(0.1)
        count = len(summaries)
        for cmd in (fake.CMD_GAME_STATUS, fake.CMD_ALLOWANCE):
            send(cmd, frames[cmd])
        spin(0.1)
        assert len(summaries) == count, 'stale robot status cannot be masked by other fresh sources'

        # Purchases only require their own fresh game/counter sources, not robot ID.
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=3))
        spin(0.16)
        assert any((bits >> 13) & 15 == 3 for bits in sentry_frames()), 'stale referee ID must not veto configured sender'

        # Identity independence does not bypass freshness or counter progression.
        spin(0.45)
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=4))
        spin(0.23)
        assert not sentry_frames(), 'stale sentry counters still reject purchases'
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=5))
        spin(0.23)
        assert not sentry_frames(), 'remote request counter may only advance by one'
        spin(0.45)
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=4))
        spin(0.23)
        assert not sentry_frames(), 'stale game status still rejects purchases'
        send(fake.CMD_GAME_STATUS, frames[fake.CMD_GAME_STATUS])
        send(fake.CMD_SENTRY_INFO, frames[fake.CMD_SENTRY_INFO])
        received.clear()
        cmdpub.publish(SentryCommand(posture=1, remote_allowance_times=1))
        spin(0.16)
        assert sentry_frames() and all((bits >> 13) & 15 == 3 for bits in sentry_frames()), 'reordered counters must not decrease transmitted cumulative values'

        # Preserve the BBT transport and actuator regressions for both selected sides.
        modepub.publish(Int32(data=1))
        spin(0.04)
        vel = Twist()
        vel.linear.x = 0.2
        vel.linear.y = -0.1
        vel.angular.z = 0.3
        velpub.publish(vel)
        spin(0.08)
        moves = [struct.unpack('<fff', p) for name, p, _ in received if name == 'chassis_data']
        assert any(abs(x - 0.2) < 1e-6 and abs(y + 0.1) < 1e-6 and abs(w - 0.3) < 1e-6
                   for x, y, w in moves), 'SI axes must not rotate'
        foldpub.publish(UInt8(data=1))
        spin(0.05)
        assert any(name == 'fold_cmd' and payload == b'\x01' for name, payload, _ in received)
        for cmd, data in frames.items():
            send(cmd, data)
        click = struct.pack('<ffBBH', 2.61, 1.52, 0, 0, 0x8080)
        send(0x0303, click[:11])
        spin(0.05)
        assert not operators, 'malformed click must not be published'
        send(0x0303, click)
        spin(0.06)
        send(0x0303, click)
        spin(0.04)
        assert len(operators) == 1 and list(operators[0].data[:2]) == [4.0, -1.0]
        assert abs(operators[0].data[3] - 2.61) < 1e-5
        modepub.publish(Int32(data=2))
        spinpub.publish(Float32(data=3.0))
        vel = Twist()
        vel.linear.x = 0.2
        velpub.publish(vel)
        spin(0.1)
        spin(0.3)
        moves = [struct.unpack('<fff', p) for name, p, _ in received if name == 'chassis_data']
        assert moves and moves[-1] == (0.0, 0.0, 0.0), 'watchdog stops rotation as well as translation'
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        os.close(master)
        os.close(slave)


try:
    for own_team in ('red', 'blue'):
        check_team(own_team)
    print('PASS: manual red/blue, missing/unknown/wrong/stale referee IDs, official wire, '
          'source aging, purchase gates, SI axes, fold_cmd, operator deduplication and watchdog')
finally:
    node.destroy_node()
    rclpy.shutdown()
