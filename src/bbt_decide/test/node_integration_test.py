#!/usr/bin/env python3
"""Whole BBT node, isolated DDS; no physical serial port or robot."""
import os
os.environ['ROS_DOMAIN_ID']='196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int32, String
from rm2027_interfaces.msg import RobotStatus

rclpy.init()
node=rclpy.create_node('bbt_node_test')
goals=[]
node.create_subscription(PoseStamped,'/goal_pose',goals.append,10)
refpub=node.create_publisher(Float64MultiArray,'/referee/summary',10)
idpub=node.create_publisher(RobotStatus,'/referee/robot_status',10)
odompub=node.create_publisher(Odometry,'/odom',10)
foldpub=node.create_publisher(Int32,'/gimbal_fold_state',10)
sourcepub=node.create_publisher(String,'/sentry/data_source',10)
root=Path(__file__).resolve().parents[1]
ref=Float64MultiArray(data=[4.,50.,400.,300.,0.,1000.,0.,1.,300.,100.,10000.,0.,0.,1000.])
odom=Odometry();odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=25.;odom.pose.pose.position.y=13.
status=RobotStatus(robot_id=0)
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        refpub.publish(ref);idpub.publish(status);odompub.publish(odom);foldpub.publish(Int32(data=4))
        rclpy.spin_once(node,timeout_sec=.03)
def stop(proc):
    proc.terminate()
    try:proc.wait(timeout=3)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
proc=None
with tempfile.TemporaryDirectory(prefix='bbt-node-test-') as tmp:
    config=Path(tmp)/'decision.yaml'
    source=yaml.safe_load((root/'config/decision.yaml').read_text())
    source['field_to_map']['enabled']=False
    log=open(Path(tmp)/'node.log','w+')
    def start():
        config.write_text(yaml.safe_dump(source))
        return subprocess.Popen([sys.argv[1],'--ros-args','-p',f'config_file:={config}',
            '-p',f'tree_file:={root}/trees/main.xml'],stdout=log,stderr=log)
    try:
        proc=start();pump(1.)
        assert not goals, 'uncalibrated real configuration must not issue navigation'
        stop(proc);proc=None
        source['field_to_map']['enabled']=True
        proc=start();pump(1.3)
        if proc.poll() is not None:
            log.seek(0);raise AssertionError(log.read())
        assert goals and abs(goals[-1].pose.position.x-25.39)<1e-6 and abs(goals[-1].pose.position.y-13.48)<1e-6
        # Protocol team changes do not rewrite manual own/enemy geometry or cancel its goal.
        count=len(goals);status.robot_id=7;pump(.3);status.robot_id=107;pump(.3)
        assert len(goals)==count, 'robot ID must not switch tactical coordinates'
        sourcepub.publish(String(data='sim'));pump(.2)
        count=len(goals);pump(.4)
        assert len(goals)==count, 'later summaries must not clear the source guard'
        print('PASS: uncalibrated startup, own/enemy manual map independent of robot ID, sticky source guard')
    finally:
        if proc:stop(proc)
        log.close()
node.destroy_node();rclpy.shutdown()
