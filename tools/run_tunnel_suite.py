#!/usr/bin/env python3
"""Run all six 2773 tunnel cases through production BBT and serial interfaces.

The unmodified default spawn is physically navigated to each test staging point.
The BBT process then owns the approach, folding, centre/exit goals, unfolding and
continuation. No engine state, geometry, navigation result or region is injected.
Only task zones/start_region change in generated configurations; coordinates and
IDs are preserved. Each replay contains preparation and actual acceptance motion.
"""
import argparse
import copy
import difflib
import importlib.util
import re
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import yaml
from check_tunnel_suite import roof_for,validate
from arena.navigation_constraints import bumps_clearance
from build_acceptance_receipt import validate_receipt, digest

ROOT=Path(__file__).resolve().parents[1]
REGIONS={'own_low':0,'own_high':1,'mid':2,'enemy_low':3,'enemy_high':4}


def install_root():
    return Path(os.environ.get('BBT_INSTALL_ROOT', ROOT/'install')).resolve()



def runtime_snapshot(install):
    """Record actual interface/module and ELF resolution, not just search paths."""
    prefixes = [Path(p).resolve() for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    for package in ('rm2027_interfaces', 'ros2_libxr', 'bbt_decide'):
        marker = Path('share/ament_index/resource_index/packages')/package
        found = next((prefix for prefix in prefixes if (prefix/marker).is_file()), None)
        if found != (install/package).resolve():
            raise RuntimeError(f'Overlay resolves {package} to {found}, expected {install/package}')
    spec = importlib.util.find_spec('rm2027_interfaces.msg')
    if not spec or not spec.origin or not Path(spec.origin).resolve().is_relative_to((install/'rm2027_interfaces').resolve()):
        raise RuntimeError(f'Python interface resolves outside build receipt: {spec}')
    libraries = {}
    for executable in ('bbt_decide/lib/bbt_decide/bbt_decide_node', 'ros2_libxr/lib/ros2_libxr/ros2_libxr_node'):
        output = subprocess.check_output(['ldd', str(install/executable)], text=True)
        if 'not found' in output:
            raise RuntimeError(f'Unresolved dynamic dependency of {executable}: {output}')
        resolved = {}
        for line in output.splitlines():
            match = re.search(r'(?:=>\s+)?(/[^\s]+)\s+\(0x[0-9a-fA-F]+\)', line)
            if match:
                path = Path(match.group(1)).resolve(strict=True)
                if path.name.startswith('librm2027_interfaces') and not path.is_relative_to((install/'rm2027_interfaces').resolve()):
                    raise RuntimeError(f'Interface library resolves outside build receipt: {path}')
                resolved[str(path)] = digest(path)
        libraries[executable] = resolved
    return {'python_interface': str(Path(spec.origin).resolve()), 'libraries': libraries}


def source_manifest(arena):
    paths=list((ROOT/'tools/arena').glob('*.py'))+[ROOT/'tools/run_tunnel_suite.py',ROOT/'tools/check_tunnel_suite.py',ROOT/'tools/build_acceptance_receipt.py',ROOT/'src/ros2_libxr/scripts/fake_lower_machine.py']
    paths+=list((ROOT/'src/bbt_decide/src').glob('*.cpp'))
    paths+=list((ROOT/'src/bbt_decide/include/bbt_decide').glob('*.hpp'))
    paths += [ROOT/p for p in ('src/bbt_decide/config/decision.yaml','src/bbt_decide/trees/main.xml',
              )]
    paths += [install_root()/p for p in ('bbt_decide/lib/bbt_decide/bbt_decide_node','ros2_libxr/lib/ros2_libxr/ros2_libxr_node')]
    paths += [arena/p for p in ('config/rules.json','rmuc_arena/engine.py','rmuc_arena/sdk.py',
              'simulation/target/release/rmuc-sim','simulation/src/physics.rs')]
    return dict(arena_root=str(arena),files={str(p):digest(p) for p in sorted(set(paths))})


def rotate_geometry(config,team):
    geometry=copy.deepcopy(config['tunnels'])
    if team!=config.get('geometry_reference','red'):
        for t in geometry:
            if t.get('center') is not None:t['center']=[28-t['center'][0],15-t['center'][1]]
            t['corners']=[[28-x,15-y] for x,y in t['corners']]
            for a in t['align']:a['p']=[28-a['p'][0],15-a['p'][1]]
    return geometry


def case_config(original,index,direction,team):
    cfg=copy.deepcopy(original);t=cfg['tunnels'][index]
    a,b=t['align'][::1 if direction=='forward' else -1]
    def distance_to_gate(point,tunnel):
        x,y=point['x'],point['y'];p,q=(k['p'] for k in tunnel['align'])
        dx,dy=q[0]-p[0],q[1]-p[1];length2=dx*dx+dy*dy
        u=max(0.,min(1.,((x-p[0])*dx+(y-p[1])*dy)/length2)) if length2 else 0.
        return math.hypot(x-p[0]-u*dx,y-p[1]-u*dy)
    candidates=[p for p in cfg['points'] if p['region']==b['region'] and p['zone']!='home'
                and all(distance_to_gate(p,t)>=cfg['safety']['gate_clear_m'] for t in cfg['tunnels'])]
    if not candidates:raise ValueError(f'no usable existing target in {b["region"]}')
    target=min(candidates,key=lambda p:math.dist([p['x'],p['y']],b['p']))
    for p in cfg['points']:
        if p['zone']!='home':p['zone']='patrol' if p['id']==target['id'] else 'retake'
    cfg['patrol_point_ids']=[target['id']]
    cfg['start_region']=a['region'];cfg['outpost']['enable']=False;cfg['auto_buy']['enable']=False
    cfg['field_to_map']=dict(enabled=True,dx=0.,dy=0.,dyaw_deg=0.)
    xy=lambda p:list(p) if team==cfg.get('geometry_reference','red') else [28-p[0],15-p[1]]
    case=dict(tunnel_index=index,tunnel_name=t['name'],direction=direction,team=team,
        entry=xy(a['p']),exit=xy(b['p']),center=xy(t['center']),
        entry_region=a['region'],exit_region=b['region'],exit_region_code=REGIONS[b['region']],
        target_id=target['id'],target_xy=xy([target['x'],target['y']]),
        vehicle_width_m=.592,feedback_window_s=cfg['safety']['fold_stale_ms']/1000,
        note='Original 39-point table, IDs, tunnel geometry and physical spawn preserved; zones select a fixed fallback task.')
    axis=[(case['exit'][i]-case['entry'][i])/math.dist(case['entry'],case['exit']) for i in range(2)]
    case['staging']=[case['entry'][i]-.55*axis[i] for i in range(2)]
    return cfg,case


def preparation_route(tunnels,destination,team,reference='red'):
    xy=lambda p:list(p) if team==reference else [28-p[0],15-p[1]]
    queue=deque([('own_low',[])]);seen={'own_low'}
    while queue:
        region,path=queue.popleft()
        if region==destination:
            result=[]
            for t,a,b in path:
                result.append(dict(tunnel=t['name'],side='entry',goal=xy(a['p'])))
                exit_goal=dict(tunnel=t['name'],side='exit',goal=xy(b['p']))
                if t.get('center') is not None:exit_goal['center']=xy(t['center'])
                result.append(exit_goal)
                length=math.dist(a['p'],b['p'])
                clear=[b['p'][i]+.55*(b['p'][i]-a['p'][i])/length for i in range(2)]
                result.append(dict(tunnel=t['name'],side='exit_clear',goal=xy(clear)))
            return result
        for t in tunnels:
            for a,b in (t['align'],t['align'][::-1]):
                if a['region']==region and b['region'] not in seen:
                    seen.add(b['region']);queue.append((b['region'],path+[(t,a,b)]))
    raise ValueError(f'no tunnel route from spawn to {destination}')


class PreparationNavigator:
    """Use production navigation boundaries during physical staging.

    Ordinary legs publish only their final goal and wait for navigation status.
    A tunnel leg uses NavigateThroughPoses with a pass-through centre. The Arena
    servers are simulation substitutes; they do not establish real Nav2 wiring.
    """
    def __init__(self,node,record=None,topics=None):
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Empty,Int32
        from nav2_msgs.action import NavigateThroughPoses
        from rclpy.action import ActionClient
        self.node=node
        self.route_type=NavigateThroughPoses
        self.route=ActionClient(node,NavigateThroughPoses,'/navigate_through_poses')
        self.lock=threading.Lock();self.token=0;self.complete=False;self.error=None;self.handle=None
        self.record=record or (lambda *args:None)
        self.waypoint=None;self.retry_count=0;self.retry_waiting=False;self.stopped_since=None
        self.ordinary_active=False;self.ordinary_running=False
        self.topics=dict(nav_goal='/goal_pose',nav_status='/nav_status',nav_cancel='/nav_cancel')
        self.topics.update({k:v for k,v in (topics or {}).items() if k in self.topics and v})
        self.goal_pub=node.create_publisher(PoseStamped,self.topics['nav_goal'],10)
        self.cancel_pub=node.create_publisher(Empty,self.topics['nav_cancel'],10)
        self.status_sub=node.create_subscription(Int32,self.topics['nav_status'],self.on_status,10)

    def pose(self,xy):
        from geometry_msgs.msg import PoseStamped
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.node.get_clock().now().to_msg()
        pose.pose.orientation.w=1.;pose.pose.position.x,pose.pose.position.y=xy
        return pose

    def ready(self):
        deadline=time.monotonic()+10.
        if not self.route.wait_for_server(timeout_sec=10.):return False
        while time.monotonic()<deadline:
            if (self.goal_pub.get_subscription_count()>0 and self.cancel_pub.get_subscription_count()>0
                    and self.status_sub.get_publisher_count()>0):return True
            time.sleep(.05)
        return False

    def status(self):
        with self.lock:return self.complete,self.error

    def on_status(self,msg):
        with self.lock:
            if not self.ordinary_active:return
            if msg.data==1:self.ordinary_running=True
            # The legacy status topic has no goal ID. Wait for the new goal's
            # running report before consuming a terminal result. This filters
            # queued terminal-only feedback, but cannot identify all stale
            # statuses after a running report; the interface has no goal ID.
            elif self.ordinary_running and msg.data==2:
                self.complete=True;self.ordinary_active=False
            elif self.ordinary_running and msg.data==3:
                self.error='preparation ordinary navigation failed: /nav_status=3'
                self.ordinary_active=False

    def _callback(self,token,callback):
        def guarded(future):
            with self.lock:
                if token!=self.token:return
            try:callback(future)
            except Exception as exc:
                with self.lock:
                    if token==self.token:self.error=str(exc)
        return guarded

    def _send(self,client,goal,token,on_result):
        def accepted(future):
            try:
                handle=future.result()
                with self.lock:
                    stale=token!=self.token
                    if not stale and handle.accepted:self.handle=handle
                # A canceled request can still be accepted by the server. Its
                # UUID must be canceled even though result callbacks are stale.
                if stale:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:raise RuntimeError('preparation navigation action rejected')
                handle.get_result_async().add_done_callback(self._callback(token,on_result))
            except Exception as exc:
                with self.lock:
                    if token==self.token:self.error=str(exc)
        client.send_goal_async(goal).add_done_callback(accepted)

    def start(self,waypoint,retry=False):
        with self.lock:
            self.token+=1;token=self.token;self.complete=False;self.error=None;self.handle=None
            self.waypoint=copy.deepcopy(waypoint)
            if not retry:self.retry_count=0
            self.retry_waiting=False;self.stopped_since=None
            self.ordinary_active=waypoint['side']!='exit';self.ordinary_running=False
        self.record('preparation_request',dict(token=token,retry=self.retry_count,waypoint=waypoint))
        def finished(future):
            from action_msgs.msg import GoalStatus
            result=future.result()
            if result.status!=GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'preparation navigation failed: status={result.status}, '
                    f'error={getattr(result.result,"error_msg","")}')
            with self.lock:self.complete=True
        if waypoint['side']=='exit':
            points=([waypoint['center']] if waypoint.get('center') is not None else [])+[waypoint['goal']]
            goal=self.route_type.Goal();goal.poses=[self.pose(point) for point in points]
            self._send(self.route,goal,token,finished)
        else:
            self.goal_pub.publish(self.pose(waypoint['goal']))

    def cancel(self):
        from std_msgs.msg import Empty
        with self.lock:
            self.token+=1;handle=self.handle;self.handle=None
            ordinary=self.waypoint is not None and self.waypoint['side']!='exit'
            self.ordinary_active=False;self.complete=False
        if handle is not None:handle.cancel_goal_async()
        if ordinary:self.cancel_pub.publish(Empty())

    def retry_after_stop(self,speed,time_s):
        """Retry an ordinary failed goal only after an explicit physical stop.

        A retry republishes the final target through the ordinary topic boundary;
        navigation owns its new obstacle route. Tunnel actions fail closed.
        """
        with self.lock:
            error=self.error;waypoint=copy.deepcopy(self.waypoint)
            waiting=self.retry_waiting;count=self.retry_count
        if not error:return False
        if waypoint is None or waypoint['side']=='exit':raise RuntimeError(error)
        if not waiting:
            if count>=2:raise RuntimeError(f'preparation retry limit: {error}')
            self.cancel()
            with self.lock:
                self.retry_waiting=True;self.retry_count+=1
            self.record('preparation_failure',dict(time_s=time_s,error=error,
                waypoint=waypoint,retry=self.retry_count,action='cancel_then_zero_then_wait_for_physical_stop'))
        if speed>.12:
            self.stopped_since=None
            return False
        if self.stopped_since is None:self.stopped_since=time_s
        if time_s-self.stopped_since<.5-1e-9:return False
        self.record('preparation_retry',dict(time_s=time_s,waypoint=waypoint,
            retry=self.retry_count,measured_speed_mps=speed,action='publish_final_goal_pose'))
        self.start(waypoint,retry=True)
        return True


def run_one(args,index,direction):
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',DECISION_ROOT=str(ROOT))
    sys.path.insert(0,str(args.arena_root))
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import UInt8,Int32
    from rmuc_arena.engine import Match,DT,POLICY_INTERVAL,STATIC_FRAME_KEYS
    from rmuc_arena import sdk
    from arena import support
    from arena.bridge import InterfaceBridge,serial_chassis_action,replay_decision
    from arena.serial_emulator import ArenaSerial
    support.sdk=sdk
    original_text=args.config.read_text();config,case=case_config(yaml.safe_load(original_text),index,direction,args.team)
    case['preparation_route']=preparation_route(config['tunnels'],case['entry_region'],args.team,config.get('geometry_reference','red'))
    case['preparation_route'].append(dict(tunnel=None,side='staging',goal=case['staging']))
    out=args.output/f'{index}_{case["tunnel_name"]}_{direction}_{args.team}';out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    config_text=yaml.safe_dump(config,sort_keys=False,allow_unicode=True)
    (out/'decision.sim.yaml').write_text(config_text)
    (out/'config.diff').write_text(''.join(difflib.unified_diff(original_text.splitlines(True),config_text.splitlines(True),fromfile='production decision.yaml',tofile='fixed-target acceptance.yaml')))
    rclpy.init();node=rclpy.create_node('bbt_tunnel_acceptance')
    serial=ArenaSerial(args.team,out/'serial.log');fmap=support.FrameMap(args.team)
    interfaces=(out/'interfaces.jsonl').open('w')
    def record(kind,data):interfaces.write(json.dumps(dict(kind=kind,data=data))+'\n')
    bridge=InterfaceBridge(node,config,fmap,serial,record)
    nav=support.NavController(fmap,arrive_m=.08);fold=support.EngineFoldReport()
    monitor=support.TunnelMonitor(rotate_geometry(config,args.team))
    fold_pub=node.create_publisher(UInt8,bridge.t['fold_cmd'],10)
    stop_pub=node.create_publisher(Twist,bridge.t['cmd_vel'],10)
    mode_pub=node.create_publisher(Int32,bridge.t['chassis_mode'],10)
    preparation=PreparationNavigator(node,record,bridge.t)
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    match=Match(seed=args.seed,duration=420,map_name='rmuc2027',lineups={team:[dict(slot=k,model=v) for k,v in ((1,2711),(3,2731),(4,2732),(6,2761),(7,2773))] for team in ('red','blue')})
    rules=json.loads((args.arena_root/'config/rules.json').read_text())
    book=rules.get('rulebook_2027',rules.get('official_2027'))
    serial.purchase_cap=int(book['economy']['ammo']['17mm']['team_purchase_cap'])
    frame=match.snapshot();sentry=next(r for r in frame['robots'] if r['id']==args.team+'-7')
    nav.prepare(match.arena,sentry)
    case['roof']=roof_for(frame['arena'],case['entry'],case['exit'])
    case.update(initial_pose=list(fmap.to_field_xy(sentry['x'],sentry['y'])),rules_hash=frame.get('rules_hash'),engine_hash=frame.get('engine_hash'),
        preparation='Physical navigation from default spawn through ROS and production PTY; no teleport.',routing_constraint='sentry-no-bumps-v1',
        navigation_clearance_radius_m=.47,navigation_interface='nav2-through-poses-cmd-vel-v2',continuation_timeout_s=args.continuation_timeout)
    metadata=dict(api_version=3,seed=args.seed,duration=420,dt=DT,policy_interval=POLICY_INTERVAL,season='2027',map='rmuc2027',match_mode='standard',
        rules_hash=case['rules_hash'],engine_hash=case['engine_hash'],teams={args.team:dict(name='BBT保底六洞验收',version='BBT-fallback',mode='team'),fmap.enemy_team:dict(name='静止对照',version='idle',mode='team')},tunnel_acceptance=case)
    rec=support.ReplayRecorder(out/'replay.json',metadata,{k:frame[k] for k in STATIC_FRAME_KEYS if k in frame},hz=5,step_s=DT)
    trajectory=(out/'trajectory.jsonl').open('w');rows=[];decision=None;decision_log=(out/'decision.log').open('w')
    started=time.monotonic();next_announce=0.;last_command=-1.;phase='prepare_fold';held={};settled=0;complete_at=None;error=None;prep_goal=0
    last_xy=None;last_radius=0.;last_progress=-10.
    try:
        if not preparation.ready():raise RuntimeError('preparation goal/status/cancel topics or tunnel action server unavailable')
        started=time.monotonic()
        while frame['time']<min(410,args.prep_timeout+args.test_timeout+5):
            bridge.time_s=frame['time'];sentry=next(r for r in frame['robots'] if r['id']==args.team+'-7')
            position=fmap.to_field_xy(sentry['x'],sentry['y']);speed=math.hypot(sentry['vx'],sentry['vy']);now=time.monotonic()
            if now>=next_announce:bridge.announce_source();next_announce=now+1.
            if phase!='test' and now-last_command>.3:
                mode_pub.publish(Int32(data=1));fold_pub.publish(UInt8(data=2 if phase=='prepare_unfold' else 1));last_command=now
            if phase=='prepare_fold' and sentry['transform_progress']>=.999:
                preparation.start(case['preparation_route'][prep_goal]);phase='prepare_navigate'
            if phase=='prepare_navigate':
                waypoint=case['preparation_route'][prep_goal];tolerance=.04 if waypoint['side'] in ('entry','exit') else .10
                reached,error=preparation.status()
                if error:
                    stop_pub.publish(Twist())
                    preparation.retry_after_stop(speed,frame['time'])
                    reached=False;settled=0
                    error=None
                settled=settled+1 if reached and math.dist(position,waypoint['goal'])<tolerance and speed<=.12 else 0
                if settled>=10:
                    waypoint.update(reached_s=frame['time'],actual_pose=list(position));prep_goal+=1;settled=0
                    if prep_goal<len(case['preparation_route']):preparation.start(case['preparation_route'][prep_goal])
                    else:preparation.cancel();stop_pub.publish(Twist());phase='prepare_unfold';last_command=-1.
            if phase=='prepare_unfold' and sentry['transform_progress']<=.001 and speed<=.12:
                case.update(test_start_s=frame['time'],serial_at_test_start=dict(serial.rx_count))
                command=[str(install_root()/'bbt_decide/lib/bbt_decide/bbt_decide_node'),'--ros-args','-p',f'config_file:={out}/decision.sim.yaml',
                    '-p','data_source:=sim','-p',f'own_team:={args.team}','-p',"referee_protocol:='2027'",'-p',f'tree_file:={ROOT}/src/bbt_decide/trees/main.xml']
                decision=subprocess.Popen(command,stdout=decision_log,stderr=decision_log);phase='test'
            if phase!='test' and frame['time']>args.prep_timeout:raise RuntimeError(f'preparation timeout in {phase}, physical pose {position}')
            if phase=='test':
                if decision.poll() is not None:raise RuntimeError('production BBT process exited')
                if frame['time']-case['test_start_s']>args.test_timeout:raise RuntimeError('production BBT crossing/continuation timeout')
            if match.tick%POLICY_INTERVAL==0:
                obs=match.observation(args.team,[7]);robot=next(r for r in obs['robots'] if int(r['slot'])==7)
                bridge.publish_observation(robot,obs,frame,fold);monitor.update(*position)
                goal,trace=bridge.navigation(nav,monitor,robot,match.arena)
                serial.pump();held={args.team:{'7':serial_chassis_action(serial,robot,fmap)}}
            frame=match.step(held)
            for action in held.get(args.team,{}).values():action.pop('buy_ammo',None);action.pop('remote_heal',None)
            sentry=next(r for r in frame['robots'] if r['id']==args.team+'-7');physical_xy=(sentry['x'],sentry['y'])
            swept=bumps_clearance(match.arena,last_xy or physical_xy,physical_xy,max(last_radius,sentry['radius']))
            navigation_swept=bumps_clearance(match.arena,last_xy or physical_xy,physical_xy,max(last_radius,sentry['radius'],.47))
            last_xy=physical_xy;last_radius=sentry['radius'];goal,_,trace=bridge.orders()
            if trace is not None:frame['sentry_decision']=replay_decision(trace,args.team,goal,bridge.attack)
            row=dict(time=frame['time'],phase=phase,model=sentry['model_slot'],position=list(fmap.to_field_xy(*physical_xy)),z=sentry['z'],
                bumps_clearance_m=bumps_clearance(match.arena,physical_xy,radius=sentry['radius']),swept_bumps_clearance_m=swept,navigation_swept_bumps_clearance_m=navigation_swept,
                speed=math.hypot(sentry['vx'],sentry['vy']),upright=sentry['upright'],tipped=sentry['tipped'],transform_progress=sentry['transform_progress'],
                trace=trace,goal=list(goal) if goal else None,nav_state=nav.state,nav_failure=getattr(nav,'last_failure',None),serial_velocity=list(serial.velocity()))
            rows.append(row);trajectory.write(json.dumps(row,allow_nan=False)+'\n');rec.maybe_add(frame)
            if navigation_swept<=0:raise RuntimeError(f'forbidden bumps clearance violation: {navigation_swept:.6f}m')
            if phase=='test' and sentry['transform_progress']<=.001 and trace is not None and trace.get('tunnel')==-1 and trace.get('current_region')==case['exit_region_code'] and math.dist(row['position'],case['exit'])<.12:
                complete_at=complete_at or frame['time']
            if complete_at is not None and math.dist(row['position'],case['target_xy'])<.08 and row['speed']<=.12 and frame['time']>complete_at+.3:break
            if complete_at is not None and frame['time']-complete_at>args.continuation_timeout:break
            if frame['time']-last_progress>=10:
                print(f'{index} {args.team} t={frame["time"]:.1f} {phase} pose={row["position"]} goal={goal} nav={nav.state}',flush=True);last_progress=frame['time']
            deadline=started+match.tick*DT
            if deadline>time.monotonic():time.sleep(deadline-time.monotonic())
    except Exception as exc:error=''.join(traceback.format_exception(exc)).strip()
    finally:
        preparation.cancel()
        if decision is not None and decision.poll() is None:
            decision.terminate()
            try:decision.wait(timeout=4)
            except subprocess.TimeoutExpired:decision.kill();decision.wait()
        with bridge.lock:
            bridge.goal=bridge.pending_goal=None
            future=bridge.route_future
            if future and not future.done():future.set_result(False)
        stop_pub.publish(Twist());executor.wake();executor.shutdown();spin.join(timeout=3)
        case.update(serial_rx=dict(serial.rx_count),serial_tx=dict(serial.tx_count),ros_rx=dict(bridge.rx_count),bad_commands=serial.bad_commands,
            final_time_s=frame['time'],error=error,goal_events=bridge.goal_events,route_events=bridge.route_events,trace_count=len(bridge.trace_events))
        rec.close();trajectory.close();interfaces.close();decision_log.close();match.close();serial.close();node.destroy_node();rclpy.shutdown()
        (out/'case.json').write_text(json.dumps(case,indent=2,ensure_ascii=False))
    try:
        if error:raise AssertionError(error)
        result=validate(rows,case)
    except (AssertionError,KeyError,ValueError) as exc:result=dict(passed=False,tunnel=case['tunnel_name'],direction=direction,error=str(exc))
    (out/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(folder=str(out),**result)),flush=True)
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--team','--own-team',dest='team',choices=['red','blue'],default='red')
    ap.add_argument('--arena-root',type=Path,default=Path(os.environ.get('ARENA_ROOT',Path.home()/'rl_workspace/arena1_sim')))
    ap.add_argument('--config',type=Path,default=ROOT/'src/bbt_decide/config/decision.yaml')
    ap.add_argument('--tunnels',nargs='+',type=int,default=list(range(6)))
    ap.add_argument('--direction',choices=['forward','reverse','both'],default='forward')
    ap.add_argument('--domain',type=int,default=189);ap.add_argument('--seed',type=int,default=1)
    ap.add_argument('--prep-timeout',type=float,default=90.);ap.add_argument('--test-timeout',type=float,default=50.)
    ap.add_argument('--continuation-timeout',type=float,default=15.)
    ap.add_argument('--jobs',type=int,default=1,help='Parallel independent DDS domains (1..6).')
    ap.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--build-receipt',type=Path,help='Verify build inputs and installed artifacts before and after the suite')
    args=ap.parse_args()
    if any(i not in range(6) for i in args.tunnels):ap.error('tunnel IDs must be 0..5')
    if not 1<=args.jobs<=6 or not 0<=args.domain<=232-args.jobs:ap.error('invalid jobs/domain range')
    args.output=args.output.resolve();args.arena_root=args.arena_root.resolve()
    if args.worker:
        return 0 if run_one(args,args.tunnels[0],args.direction)['passed'] else 1
    args.output.mkdir(parents=True,exist_ok=True)
    if any(args.output.iterdir()):
        raise RuntimeError(f'Acceptance output directory must be empty: {args.output}')
    if args.build_receipt is None:
        ap.error('--build-receipt is required for tunnel acceptance')
    receipt = None
    if args.build_receipt:
        receipt_path = args.build_receipt.resolve()
        receipt = json.loads(receipt_path.read_text())
        verified_install = validate_receipt(receipt)
        prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if not all(str(verified_install/package) in prefixes for package in ('rm2027_interfaces', 'ros2_libxr', 'bbt_decide')):
            raise RuntimeError(f'Source {verified_install}/setup.bash before acceptance')
        os.environ['BBT_INSTALL_ROOT'] = str(verified_install)
        (args.output/'build_receipt.json').write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
    sources=source_manifest(args.arena_root)
    sources['build_receipt'] = ({'status':'verified_at_start', 'sha256':digest(receipt_path)} if receipt else {'status':'not_provided'})
    sources['invocation'] = sys.argv
    if receipt:
        sources['runtime_resolution'] = runtime_snapshot(verified_install)
    sources['config'] = {'path':str(args.config.resolve()), 'sha256':digest(args.config)}
    (args.output/'source_manifest.json').write_text(json.dumps(sources,indent=2)+'\n')
    cases=[(i,d) for i in args.tunnels for d in (('forward','reverse') if args.direction=='both' else (args.direction,))]
    def child(item):
        n,(i,d)=item
        command=[sys.executable,str(Path(__file__).resolve()),'--worker','--team',args.team,'--arena-root',str(args.arena_root),'--config',str(args.config),
            '--tunnels',str(i),'--direction',d,'--domain',str(args.domain+n%args.jobs),'--seed',str(args.seed),'--prep-timeout',str(args.prep_timeout),
            '--test-timeout',str(args.test_timeout),'--continuation-timeout',str(args.continuation_timeout),'--output',str(args.output)]
        return subprocess.run(command,check=False).returncode
    # Groups retain exclusive DDS domains even if one run finishes early.
    exit_codes=[]
    for offset in range(0,len(cases),args.jobs):
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:exit_codes.extend(pool.map(child,enumerate(cases[offset:offset+args.jobs])))
    results=[]
    for (i,d),exit_code in zip(cases,exit_codes):
        name=yaml.safe_load(args.config.read_text())['tunnels'][i]['name']
        path=args.output/f'{i}_{name}_{d}_{args.team}'/'validation.json'
        result = json.loads(path.read_text()) if path.exists() else dict(passed=False,tunnel=name,direction=d,error='worker produced no validation')
        if exit_code != 0 and result.get('passed'):
            result = dict(passed=False,tunnel=name,direction=d,error=f'worker exited {exit_code} despite passing validation')
        results.append(result)
    (args.output/'suite.json').write_text(json.dumps(results,indent=2))
    if source_manifest(args.arena_root)!={k:v for k,v in sources.items() if k not in ('build_receipt','invocation','config','runtime_resolution')}:
        raise RuntimeError('Acceptance sources changed during suite; rerun the final version')
    if digest(args.config) != sources['config']['sha256']:
        raise RuntimeError('Acceptance config changed during suite')
    if receipt:
        if digest(receipt_path) != sources['build_receipt']['sha256']:
            raise RuntimeError('Build receipt changed during suite')
        validate_receipt(receipt)
        if runtime_snapshot(verified_install) != sources['runtime_resolution']:
            raise RuntimeError('Resolved runtime dependencies changed during suite')
        sources['build_receipt']['status'] = 'verified_at_start_and_end'
    (args.output/'source_manifest.json').write_text(json.dumps(sources,indent=2)+'\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
