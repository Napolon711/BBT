#!/usr/bin/env python3
"""Run production BBT + ros2_libxr against Arena-1 in an isolated DDS domain.

No simulator writes the decision's internal state or synthesizes successful navigation
or folding. Physics state produces referee bytes; serialized commands drive physics.
The --acceptance scenario changes ONLY a generated decision config, never engine rules.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arena-root', type=Path, default=Path(os.environ.get('ARENA_ROOT', Path.home()/'.zcode/workspace/default/Arena-1')))
    ap.add_argument('--config', type=Path, default=ROOT/'src/bbt_decide/config/decision.yaml')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--domain', type=int, default=186)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--team', choices=['blue', 'red'], default='blue', help='Arena slot owning the unchanged own/enemy point table')
    ap.add_argument('--output', type=Path, default=ROOT/'runs/arena_latest')
    ap.add_argument('--acceptance', action='store_true', help='Disable uncalibrated outpost points; raise ammo trigger to 350 to test a real remote purchase')
    ap.add_argument('--fault-chassis', default='', help='Suppress chassis feedback during elapsed seconds START:END')
    ap.add_argument('--no-replay', action='store_true')
    ap.add_argument('--six-tunnels', action='store_true', help='Generate a temporary unique-region chain that traverses all six tunnels once')
    args = ap.parse_args()
    if not 0 <= args.domain <= 232 or not math.isfinite(args.seconds) or args.seconds <= 0:
        ap.error('domain must be 0..232; seconds must be positive')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'result.json').exists():
        ap.error('output already contains a run; choose a new --output directory')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['DECISION_ROOT'] = str(ROOT)
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    sys.path.insert(0, str(args.arena_root.resolve()))
    from rmuc_arena.engine import Match, DT, STATIC_FRAME_KEYS
    from rmuc_arena import sdk
    from arena import support
    from arena.serial_emulator import ArenaSerial
    support.sdk = sdk
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32, UInt8, Empty, Float64MultiArray, Int32MultiArray, String
    from rm2027_interfaces.msg import SentryInfo, SentryCommand

    cfg = yaml.safe_load(args.config.read_text())
    transform = cfg.get('field_to_map', {})
    if transform.get('enabled') and any(abs(float(transform.get(k, 0))) > 1e-9 for k in ('dx','dy','dyaw_deg')):
        ap.error('Arena adapter publishes fixed field/map coordinates; use a separate identity simulation config')
    cfg['field_to_map'] = dict(enabled=True, dx=0., dy=0., dyaw_deg=0.)
    six_targets=[]
    if args.six_tunnels:
        # Demo-only chain. Each edge gets a unique pair of regions so parallel
        # real tunnels cannot make BFS choose the same edge twice.
        regions=[f'demo_{i}' for i in range(7)]
        cfg['regions']=regions; cfg['start_region']=regions[0]
        cfg['outpost']['enable']=False; cfg['auto_buy']['enable']=False
        chain={
          'own_road_tunnel':(0,1), 'enemy_near_tunnel':(1,2),
          'enemy_road_tunnel':(2,3), 'enemy_far_tunnel':(3,4),
          'own_far_tunnel':(4,5), 'own_near_tunnel':(5,6)}
        for tunnel in cfg['tunnels']:
            a,b=chain[tunnel['name']]
            tunnel['align'][0]['region']=regions[a]
            tunnel['align'][1]['region']=regions[b]
        cfg['points']=[
          {'id':3000,'x':22.69,'y':9.42,'zone':'home','region':regions[0]},
          {'id':3001,'x':13.27,'y':11.39,'zone':'patrol','region':regions[1]},
          {'id':3002,'x':17.24,'y':3.55,'zone':'patrol','region':regions[2]},
          {'id':3003,'x':10.20,'y':0.75,'zone':'patrol','region':regions[3]},
          {'id':3004,'x':5.37,'y':12.78,'zone':'patrol','region':regions[4]},
          {'id':3005,'x':6.24,'y':11.78,'zone':'patrol','region':regions[5]},
          {'id':3006,'x':22.69,'y':9.42,'zone':'patrol','region':regions[6]},
        ]
        cfg['safety']['ammo_home_min']=-1
        six_targets=[3001,3002,3003,3004,3005,3006]
    if args.acceptance:
        cfg['outpost']['enable'] = False
        cfg['auto_buy']['ammo_min_shots'] = 350
        cfg['auto_buy']['ammo_min_coins'] = 150
    config_path = out/'decision.sim.yaml'
    config_path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    topics = cfg.get('topics', {}).get('sim', {})
    topic = lambda key, default: topics.get(key) or default
    fault = tuple(map(float, args.fault_chassis.split(':'))) if args.fault_chassis else None
    if fault and (len(fault) != 2 or not 0 <= fault[0] < fault[1]):
        ap.error('--fault-chassis must be START:END')
    rclpy.init()
    node = rclpy.create_node('bbt_arena_adapter')
    stream = (out/'interfaces.jsonl').open('w')
    started = time.monotonic()
    counts = {}
    state = {'goal': None, 'attack': [0,0,0,0], 'attack_time': -1e9, 'straight_goal': None, 'pending_goal': None, 'revision': 0}
    summaries, purchases, fold_states, goals, fold_commands = [], [], [], [], []
    def record(kind, data):
        counts[kind] = counts.get(kind, 0)+1
        stream.write(json.dumps({'wall_s': time.monotonic()-started, 'kind': kind, 'data': data})+'\n')
    def goal_cb(msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        state['pending_goal'] = goal
        state['revision'] += 1
        goals.append(goal); record('goal', goal)
    def cancel_cb(_):
        state['goal'] = None; state['straight_goal'] = None; state['revision'] += 1; record('cancel', None)
    def attack_cb(msg):
        if len(msg.data) >= 4:
            state['attack'] = list(msg.data[:4]);state['attack_time'] = time.monotonic()
    def summary_cb(msg):
        summaries.append(list(msg.data)); record('referee', list(msg.data))
    def purchase_cb(msg):
        purchases.append([msg.remote_allowance_times, msg.remote_hp_times, msg.exchange_allowance_total])
        record('purchase', purchases[-1])
    def fold_cb(msg):
        if not fold_states or fold_states[-1] != msg.data:
            fold_states.append(msg.data);record('fold_state', msg.data)
    def command_cb(msg):
        fold_commands.append(msg.data);record('fold_cmd', msg.data)
    node.create_subscription(PoseStamped, topic('nav_goal','/goal_pose'), goal_cb, 10)
    node.create_subscription(Empty, topic('nav_cancel','/nav_cancel'), cancel_cb, 10)
    node.create_subscription(Int32MultiArray, topic('attack_cmd','/sentry/attack_cmd'), attack_cb, 10)
    node.create_subscription(Float64MultiArray, topic('referee','/referee/summary'), summary_cb, 10)
    node.create_subscription(SentryCommand, '/referee/sentry_command', purchase_cb, 10)
    node.create_subscription(Int32, topic('fold_state','/gimbal_fold_state'), fold_cb, 10)
    node.create_subscription(UInt8, topic('fold_cmd','/gimbal_fold_cmd'), command_cb, 10)
    odom_pub = node.create_publisher(Odometry, topic('odom','/odom'), 10)
    nav_pub = node.create_publisher(Int32, topic('nav_status','/nav_status'), 10)
    vel_pub = node.create_publisher(Twist, topic('cmd_vel','/cmd_vel'), 10)
    source_pub = node.create_publisher(String, topic('data_source','/sentry/data_source'), 10)
    op_pub = node.create_publisher(Int32MultiArray, topic('op_cmd','/sentry/operator_point'), 10)
    serial = match = decision = recorder = None
    log = (out/'decision.log').open('w')
    result = {}
    six_index = 0
    six_seq = 1
    try:
        serial = ArenaSerial(args.team, str(out/'serial.log'))
        match = Match(seed=args.seed, duration=420, map_name='rmuc2027')
        rules = json.loads((args.arena_root/'config/rules.json').read_text())
        serial.purchase_cap = int(rules['official_2027']['economy']['ammo']['17mm']['team_purchase_cap'])
        fmap = support.FrameMap(args.team)
        nav = support.NavController(fmap, arrive_m=.20)
        fold = support.EngineFoldReport()
        monitor = support.TunnelMonitor(cfg.get('tunnels', []))
        alignments = [tuple(a['p']) for t in cfg.get('tunnels', []) for a in t['align']]
        frame = match.snapshot()
        first = next(r for r in frame['robots'] if r['team']==args.team and int(r['slot'])==7)
        initial_pose = fmap.to_field_xy(first['x'], first['y'])
        initial_ammo = first['ammo']
        travelled = 0.;previous = initial_pose
        events = []; nav_successes=[]; physics_samples=[]
        if not args.no_replay:
            recorder = support.ReplayRecorder(out/'replay.json',
                {'seed': args.seed, 'map': 'rmuc2027', 'own_team': args.team,
                 'teams': {args.team:{'name':'BBT fallback','version':'BBT','mode':'team'},
                           fmap.enemy_team:{'name':'Arena idle','version':'idle','mode':'team'}},
                 'acceptance':args.acceptance}, {k:frame[k] for k in STATIC_FRAME_KEYS if k in frame}, hz=5, step_s=DT)
        decision = subprocess.Popen([str(ROOT/'install/bbt_decide/lib/bbt_decide/bbt_decide_node'),
            '--ros-args','-p',f'config_file:={config_path}','-p','data_source:=sim',
            '-p',"referee_protocol:='2027'",'-p',f'tree_file:={ROOT}/src/bbt_decide/trees/main.xml'], stdout=log, stderr=log)
        started = time.monotonic(); next_source=0.; held={args.team:{},fmap.enemy_team:{}}
        while frame['time'] < args.seconds and match.status != 'finished':
            if decision.poll() is not None: raise RuntimeError('BBT exited; see decision.log')
            rclpy.spin_once(node, timeout_sec=0)
            serial.pump()
            if match.tick % 2 == 0:
                obs = match.observation(args.team,[7])
                robot = next(r for r in obs['robots'] if int(r['slot'])==7)
                pos = fmap.to_field_xy(robot['x'], robot['y'])
                travelled += math.dist(pos,previous);previous=pos
                monitor.update(*pos)
                serial.suppress_chassis = bool(fault and fault[0] <= frame['time'] < fault[1])
                serial.referee(obs, robot, frame, fmap)
                serial.chassis(robot, fmap, fold.update(robot))
                msg=Odometry();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='map'
                msg.child_frame_id='base_link';msg.pose.pose.position.x,msg.pose.pose.position.y=pos
                msg.pose.pose.orientation.w=1.;odom_pub.publish(msg)
                if frame['time'] >= next_source:
                    source_pub.publish(String(data='sim'));next_source=frame['time']+1
                # Empty cancel and PoseStamped goal are independent DDS streams.
                # Apply all queued cancellations first, then the latest new goal.
                # This adapter treats a replacement pair in one control cycle atomically.
                if state['pending_goal'] is not None:
                    state['goal']=state['pending_goal'];state['pending_goal']=None
                    state['straight_goal']=None
                goal=state['goal']
                if goal and monitor.crossing(*pos,*goal): state['straight_goal']=goal
                vx,vy=nav.update(goal,pos,robot,match.arena,straight=goal is not None and state['straight_goal']==goal,
                                 precise=goal in alignments,revision=state['revision'])
                nav_pub.publish(Int32(data=nav.state))
                if nav.state==2 and (not nav_successes or nav_successes[-1] != list(goal)):
                    nav_successes.append(list(goal));record('nav_success',list(goal))
                    if args.six_tunnels and six_index < len(six_targets) and goal and math.dist(goal, next((p['x'],p['y']) for p in cfg['points'] if p['id']==six_targets[six_index]),) < .01:
                        six_index += 1
                        if six_index < len(six_targets):
                            cmd=Int32MultiArray();cmd.data=[4,six_targets[six_index],six_seq];six_seq+=1;op_pub.publish(cmd);record('six_target',cmd.data)
                if goal:
                    vel=Twist();vel.linear.x,vel.linear.y=fmap.to_field_vec(vx,vy);vel_pub.publish(vel)
                cvx,cvy,w=serial.velocity();ax,ay=fmap.to_arena_vec(cvx,cvy)
                kind,tid,fire,free=state['attack'] if time.monotonic()-state['attack_time']<.3 else [0,0,0,0]
                target=None
                if kind==2 and tid in (0,1):
                    candidate=obs['referee']['outposts'][args.team if tid==0 else fmap.enemy_team]
                    if candidate['hp']>0 and candidate['owner']==fmap.enemy_team:target=candidate
                elif free:
                    enemies=[r for r in obs['enemies'] if r['alive'] and int(r['slot']) in (1,3,4,7)]
                    if enemies:target=min(enemies,key=lambda r:math.hypot(r['x']-robot['x'],r['y']-robot['y']))
                action={'move':[ax/4.,ay/4.], 'yaw_rate':-w, 'fire':bool(fire and target),
                        'aim':math.atan2(target['y']-robot['y'],target['x']-robot['x']) if target else robot['aim'],
                        'pitch':.04,'crouch':serial.fold_cmd==1}
                action.update(serial.purchase_action(robot))
                held={args.team:{'7':action},fmap.enemy_team:{}}
                physics_samples.append({'time':frame['time'],'pos':pos,'fold':fold.update(robot),
                                        'goal':goal,'nav':nav.state,'ammo':robot['ammo'],'hp':robot['hp'],
                                        'speed':math.hypot(robot['vx'],robot['vy'])})
                if match.tick % 100 == 0:
                    print(f"t={frame['time']:.1f} own={pos} goal={goal} nav={nav.state} fold={fold_states[-1:]}",flush=True)
            frame=match.step(held)
            for action in held[args.team].values():
                action.pop('buy_ammo',None);action.pop('remote_heal',None)
            events.extend(frame.get('events',[]))
            if recorder:recorder.maybe_add(frame)
            deadline=started+match.tick*DT
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
        rclpy.spin_once(node,timeout_sec=.02)
        last=next(r for r in frame['robots'] if r['team']==args.team and int(r['slot'])==7)
        result={'time':frame['time'],'initial_pose':initial_pose,'final_pose':fmap.to_field_xy(last['x'],last['y']),
                'distance_m':travelled,'initial_ammo':initial_ammo,'final_ammo':last['ammo'],
                'serial_rx':serial.rx_count,'serial_tx':serial.tx_count,'bad_commands':serial.bad_commands,
                'purchases':serial.counts,'goal_count':len(goals),'nav_successes':nav_successes,
                'fold_commands':fold_commands,'fold_states':fold_states,'tunnel_entries':monitor.counts,
                'ros_counts':counts,'acceptance':args.acceptance,'chassis_fault':fault,
                'events':events,'six_targets_completed':six_index}
        (out/'physics.json').write_text(json.dumps(physics_samples))
        (out/'result.json').write_text(json.dumps(result,indent=2))
        if not summaries or not goals or serial.bad_commands or travelled<.5 or (args.six_tunnels and six_index != len(six_targets)):
            raise RuntimeError('Arena smoke checks failed; inspect result.json')
        print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))
    finally:
        if decision and decision.poll() is None:
            decision.terminate()
            try:decision.wait(timeout=3)
            except subprocess.TimeoutExpired:decision.kill();decision.wait()
        if recorder:recorder.close()
        if match:match.close()
        if serial:serial.close()
        log.close();stream.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    main()
