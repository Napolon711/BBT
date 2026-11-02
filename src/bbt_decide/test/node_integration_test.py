#!/usr/bin/env python3
"""Whole BBT node, isolated DDS; no physical serial port or robot."""
import json
import os
os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import resource
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from std_msgs.msg import Float64MultiArray, Int32, String
from rm2027_interfaces.msg import RobotStatus

resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
rclpy.init()
node = rclpy.create_node('bbt_node_test')
goals, traces = [], []
node.create_subscription(PoseStamped, '/goal_pose', goals.append, 10)
node.create_subscription(String, '/sentry/decision_trace', lambda m: traces.append(json.loads(m.data)), 10)
refpub = node.create_publisher(Float64MultiArray, '/referee/summary', 10)
idpub = node.create_publisher(RobotStatus, '/referee/robot_status', 10)
odompub = node.create_publisher(Odometry, '/odom', 10)
foldpub = node.create_publisher(Int32, '/gimbal_fold_state', 10)
sourcepub = node.create_publisher(String, '/sentry/data_source', 10)
parameter_client = AsyncParameterClient(node, '/bbt_decide')
root = Path(__file__).resolve().parents[1]
ref = Float64MultiArray(data=[4., 50., 400., 300., 0., 1000., 0., 1., 300., 100., 10000., 0., 0., 1000.])
odom = Odometry()
odom.pose.pose.orientation.w = 1.
reported_id = None


def compute_path(goal_handle):
    """Nav2 boundary substitute; production BBT must inspect this returned path."""
    result = ComputePathToPose.Result()
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.pose = odom.pose.pose
    result.path = NavPath(poses=[start, goal_handle.request.goal])
    result.path.header.frame_id = 'map'
    goal_handle.succeed()
    return result


def follow_path(goal_handle):
    assert goal_handle.request.path.header.frame_id == 'map'
    assert len(goal_handle.request.path.poses) >= 2
    goal_handle.succeed()
    return FollowPath.Result()


compute_server = ActionServer(node, ComputePathToPose, '/compute_path_to_pose', compute_path)
follow_server = ActionServer(node, FollowPath, '/follow_path', follow_path)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        refpub.publish(ref)
        if reported_id is not None:
            idpub.publish(RobotStatus(robot_id=reported_id))
        odompub.publish(odom)
        foldpub.publish(Int32(data=4))
        rclpy.spin_once(node, timeout_sec=.03)


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


proc = None
with tempfile.TemporaryDirectory(prefix='bbt-node-test-') as tmp:
    config = Path(tmp) / 'decision.yaml'
    source = yaml.safe_load((root / 'config/decision.yaml').read_text())
    source['geometry_reference'] = 'red'
    source['field_to_map'] = dict(enabled=True, dx=0., dy=0., dyaw_deg=0.)
    config.write_text(yaml.safe_dump(source))
    base_args = [sys.argv[1], '--ros-args', '-p', f'config_file:={config}',
                 '-p', f'tree_file:={root}/trees/main.xml']
    for invalid in ('', 'auto', 'green'):
        args = base_args + (['-p', 'own_team:=' + invalid] if invalid else [])
        result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=3, text=True)
        assert result.returncode != 0 and 'own_team' in result.stdout, result.stdout
    home = next(p for p in source['points'] if p['zone'] == 'home')
    for own_team in ('red', 'blue'):
        expected = (home['x'], home['y']) if own_team == 'red' else (28. - home['x'], 15. - home['y'])
        odom.pose.pose.position.x, odom.pose.pose.position.y = expected[0] + .7, expected[1]
        reported_id = None
        goals.clear()
        traces.clear()
        with open(Path(tmp) / f'{own_team}.log', 'w+') as log:
            def start():
                config.write_text(yaml.safe_dump(source))
                return subprocess.Popen(base_args + ['-p', 'own_team:=' + own_team], stdout=log, stderr=log)
            try:
                source['field_to_map']['enabled'] = False
                proc = start()
                pump(.7)
                assert proc.poll() is None
                assert not goals, 'uncalibrated real configuration must not issue navigation'
                stop(proc)
                proc = None
                source['field_to_map']['enabled'] = True
                proc = start()
                # DDS discovery can outlive the quick restart of the prior node.
                for _ in range(30):
                    pump(.1)
                    # Goal and trace use independent DDS streams. A goal can
                    # arrive before the running trace from the same tick.
                    ready = goals and traces and traces[-1].get('own_team') == own_team and traces[-1].get('match_running')
                    if ready or proc.poll() is not None:
                        break
                if proc.poll() is not None:
                    log.seek(0)
                    raise AssertionError(log.read())
                if not goals:
                    log.seek(0)
                    raise AssertionError('manually configured team must navigate without referee identity\n' + log.read())
                assert abs(goals[-1].pose.position.x - expected[0]) < 1e-6
                assert abs(goals[-1].pose.position.y - expected[1]) < 1e-6
                assert traces and traces[-1]['source'] == 'fallback' and traces[-1]['own_team'] == own_team
                assert traces[-1]['match_running']
                assert parameter_client.wait_for_services(timeout_sec=1.)
                change = parameter_client.set_parameters([
                    Parameter('own_team', value='blue' if own_team == 'red' else 'red')])
                pump(.1)
                assert change.done() and not change.result().results[0].successful, 'own_team is fixed until restart'
                count = len(goals)
                for reported_id in ((107 if own_team == 'red' else 7), 0, (7 if own_team == 'red' else 107)):
                    pump(.2)
                    assert len(goals) == count, 'referee ID must not override or veto the manual team'
                reported_id = None
                pump(.2)
                assert len(goals) == count
                sourcepub.publish(String(data='sim'))
                pump(.2)
                count = len(goals)
                pump(.3)
                assert len(goals) == count, 'later summaries must not clear the source guard'
            finally:
                if proc:
                    stop(proc)
                    proc = None
node.destroy_node()
rclpy.shutdown()
print('PASS: required immutable red/blue, rotated home, missing/wrong referee IDs, decision trace, calibration gate and source guard')
