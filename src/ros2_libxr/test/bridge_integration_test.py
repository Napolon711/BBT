#!/usr/bin/env python3
"""Isolated PTY/ROS integration test; never opens a physical device."""
import importlib.util
import os
import pty
import struct
import subprocess
import sys
import time
import tty
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from std_msgs.msg import Float64MultiArray, Float32, Int32, UInt8
from geometry_msgs.msg import Twist
from rm2027_interfaces.msg import GameStatus, SentryCommand

root = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('fake', root/'src/ros2_libxr/scripts/fake_lower_machine.py')
fake = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fake)
master, slave = pty.openpty()
tty.setraw(master)
tty.setraw(slave)
os.set_blocking(master, False)
proc = None
rclpy.init()
node = rclpy.create_node('bridge_integration_test')
games, summaries, received, operators = [], [], [], []
node.create_subscription(GameStatus, '/referee/game_status', games.append, 10)
node.create_subscription(Float64MultiArray, '/referee/summary', summaries.append, 10)
node.create_subscription(Float64MultiArray, '/referee/operator_cmd', operators.append, 10)
foldpub = node.create_publisher(UInt8, '/gimbal_fold_cmd', 10)
velpub = node.create_publisher(Twist, '/cmd_vel', 10)
modepub = node.create_publisher(Int32, '/move_mode', 10)
spinpub = node.create_publisher(Float32, '/cmd_spin', 10)
cmdpub = node.create_publisher(SentryCommand, '/referee/sentry_command', 10)
parser = fake.Parser()
def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=0.01)
        try:
            data=os.read(master,65536)
            if data: received.extend(parser.feed(data))
        except BlockingIOError:
            pass

def send(cmd, payload):
    raw=fake.ref_frame(cmd,payload,1)
    wire=struct.pack('<BBHI',0,1,len(raw),10)+raw.ljust(128,b'\0')
    os.write(master,fake.pack('ref_frame',wire,10000))

try:
    exe = Path(sys.argv[1]) if len(sys.argv)>1 else root/'install/ros2_libxr/lib/ros2_libxr/ros2_libxr_node'
    proc=subprocess.Popen([str(exe),'--ros-args','-p',f'port:={os.ttyname(slave)}',
        '-p','referee_timeout_ms:=400'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    spin(0.8)
    assert not summaries, 'no summary before receiving required referee sources'
    send(1,struct.pack('<BHQ',0x41,300,0x1122334455667788))
    for cmd,data in fake.build_round(120):
        if cmd!=1: send(cmd,data)
    spin(0.12)
    assert games and (games[-1].game_type,games[-1].game_progress,games[-1].stage_remain_time)==(1,4,300)
    assert games[-1].sync_time_stamp==0x1122334455667788, 'unaligned game timestamp'
    assert summaries and len(summaries[-1].data)==14 and summaries[-1].data[13]==300
    first_age=summaries[-1].data[12]
    spin(0.12)
    assert summaries[-1].data[12]>first_age, 'source age must advance without frames'
    spin(0.45)
    count=len(summaries);spin(0.15)
    assert len(summaries)==count, 'stale referee must stop producing live summaries'
    # Identity zero must not be interpreted as the opposite side, even with live sources.
    for cmd,data in fake.build_round(120):
        send(cmd,(bytes([0])+data[1:]) if cmd==0x0201 else data)
    spin(0.12)
    assert summaries[-1].data[5]==-1 and summaries[-1].data[6]==-1 and summaries[-1].data[13]==-1
    for cmd,data in fake.build_round(120): send(cmd,data)
    spin(.12)
    # Keep game/ammo live while robot status alone goes stale: summaries must stop.
    for _ in range(6):
        for cmd,data in fake.build_round(120):
            if cmd!=0x0201:send(cmd,data)
        spin(.1)
    count=len(summaries);spin(.1)
    assert len(summaries)==count
    # BBT uses the same raw frame transport, SI velocity axes and command packing.
    modepub.publish(Int32(data=1));spin(0.04)
    vel=Twist();vel.linear.x=0.2;vel.linear.y=-0.1;vel.angular.z=0.3
    velpub.publish(vel);spin(0.08)
    moves=[struct.unpack('<fff',p) for name,p,_ in received if name=='chassis_data']
    assert any(abs(x-0.2)<1e-6 and abs(y+0.1)<1e-6 and abs(w-0.3)<1e-6 for x,y,w in moves), 'SI axes must not rotate'
    foldpub.publish(UInt8(data=1));spin(0.05)
    assert any(name=='fold_cmd' and p==b'\x01' for name,p,_ in received)
    for cmd,data in fake.build_round(120): send(cmd,data)
    click=struct.pack('<ffBBH',2.61,1.52,0,0,0x8080)
    send(0x0303,click[:11]);spin(.05);assert not operators
    send(0x0303,click);spin(0.06);send(0x0303,click);spin(0.04)
    assert len(operators)==1 and list(operators[0].data[:2])==[4.0,-1.0]
    assert abs(operators[0].data[3]-2.61)<1e-5
    command=SentryCommand(remote_allowance_times=1)
    cmdpub.publish(command);spin(0.25)
    transmissions=[p for name,p,_ in received if name=='ref_tx']
    assert transmissions, 'purchase must reach the lower machine'
    wire=transmissions[-1]
    assert struct.unpack_from('<HHB',wire)==(0x0301,10,0)
    assert struct.unpack_from('<HHH',wire,8)==(0x0120,7,0x8080)
    assert (struct.unpack_from('<I',wire,14)[0]>>13)&15==1
    assert (struct.unpack_from('<I',wire,14)[0]>>2)&0x7ff==0, 'remote request must not duplicate a local purchase'
    modepub.publish(Int32(data=2));spinpub.publish(Float32(data=3.0))
    vel=Twist();vel.linear.x=0.2;velpub.publish(vel);spin(0.1)
    spin(0.3)
    moves=[struct.unpack('<fff',p) for name,p,_ in received if name=='chassis_data']
    assert moves and moves[-1]==(0.0,0.0,0.0), 'watchdog stops rotation as well as translation'
    print('PASS: official referee wire, source aging, SI velocity axes, fold_cmd, operator deduplication, 0x0120 purchase and watchdog')
finally:
    if proc is not None:
        proc.terminate()
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
    node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
