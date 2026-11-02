#!/usr/bin/env python3
"""Unranked feature demo: red BBT sentry, nine parked robots, recorded stimuli.

Uses the unchanged production decision/config/ROS+PTY chain and original Arena
library. A separate linked driver supplies explicitly labelled demonstration
referee/HP/ammo events. Navigation/folding and paid delivery use real physics.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import yaml

ROOT=Path(__file__).resolve().parents[1]

def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def build_driver(arena,output):
    deps=arena/'simulation/target/release/deps';source=ROOT/'tools/arena/demo_driver.rs'
    library=next(deps.glob('librmuc_sim-*.rlib'));serde=next(deps.glob('libserde_json-*.rlib'))
    binary=output/'demo-driver'
    subprocess.run(['rustc','--edition=2021','-O',str(source),'-L',f'dependency={deps}',
        '--extern',f'rmuc_sim={library}','--extern',f'serde_json={serde}','-o',str(binary)],check=True)
    return binary,dict(driver_source_sha256=digest(source),arena_library_sha256=digest(library),driver_sha256=digest(binary))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arena-root',type=Path,default=Path.home()/'rl_workspace/arena1_sim')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seconds',type=float,default=300.)
    ap.add_argument('--domain',type=int,default=214)
    ap.add_argument('--seed',type=int,default=2027)
    args=ap.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    if not 240<=args.seconds<=420:ap.error('feature demonstration needs 240..420 seconds')
    os.environ.update(DECISION_ROOT=str(ROOT),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    sys.path.insert(0,str(args.arena_root))
    from rmuc_arena.engine import Match,DT,STATIC_FRAME_KEYS
    from rmuc_arena import sdk
    from run_bbt_match import TeamController,MatchRecorder,verify_recording
    from arena import support
    from arena.bridge import InterfaceBridge,serial_chassis_action,replay_decision
    from arena.serial_emulator import ArenaSerial
    support.sdk=sdk;api=dict(bridge=InterfaceBridge,serial=ArenaSerial,action=serial_chassis_action)
    cfg_path=ROOT/'src/bbt_decide/config/decision.yaml';cfg=yaml.safe_load(cfg_path.read_text())
    reference=next(point for point in cfg['points'] if point['id']==34)
    operator_command=dict(time_s=260.,xy=[reference['x']+.12,reference['y']-.08],
        arrival_tolerance_m=.08,description='任意坐标指点，不吸附配置点；准确到达后按驻点规则交还自主权')
    (out/'decision.sim.yaml').write_bytes(cfg_path.read_bytes())
    binary,driver_manifest=build_driver(args.arena_root,out)
    # These are test stimuli, not autonomous achievements or match results.
    schedule=[dict(time_s=0.,kind='resources',amount=5000,description='演示测试余额，实际购买仍扣款'),
        dict(time_s=60.,kind='all_outposts_red',description='用户指定：一分钟我方完全占领双方前哨'),
        dict(time_s=85.,kind='hp_ratio',ratio=.35,description='受伤至35%，触发远程买血'),
        dict(time_s=135.,kind='ammo',amount=20,description='低弹20发，触发买弹与回家补给'),
        dict(time_s=190.,kind='hp_ratio',ratio=.2,description='受伤至20%，触发低血回家与远程补血'),
        dict(time_s=235.,kind='destroy',description='演示阵亡，验证真实付费复活链路')]
    # Keep original physical spawn positions. Other robots receive stop/brake;
    # their passive collision response is still governed by the original engine.
    training=None
    save(out/'scenario.json',dict(kind='feature-demo',not_competitive_match=True,only_active_robot='red-7',
        stimuli=schedule,operator_command=operator_command,initial_training=training,
        unchanged_production_config_sha256=digest(cfg_path)))
    (out/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    (out/'driver_source.rs').write_bytes((ROOT/'tools/arena/demo_driver.rs').read_bytes())
    lineups={team:[dict(slot=s,model=m) for s,m in ((1,2711),(3,2731),(4,2732),(6,2761),(7,2773 if team=='red' else 2772))] for team in ('red','blue')}
    class DemoMatch(Match):
        def _request(self,request,raw=False):
            if request.get('op')=='init':request=dict(request,demo_events=schedule)
            return super()._request(request,raw=raw)
    old_binary=os.environ.get('RMUC_SIM_BINARY');os.environ['RMUC_SIM_BINARY']=str(binary)
    match=None;controller=None;recorder=None;error=None;events=[];stats=CounterStats()
    try:
        match=DemoMatch(seed=args.seed,duration=args.seconds,map_name='rmuc2027',lineups=lineups,training_scenario=training)
        frame=match.snapshot();initial=copy.deepcopy(frame)
        metadata=dict(api_version=3,engine='rust',season='2027',duration=args.seconds,dt=DT,policy_interval=2,
            seed=args.seed,map='rmuc2027',lineups=lineups,own_team='red',test_match=True,feature_demo=True,
            demo_events=schedule,training_scenario=training,
            engine_hash=frame['engine_hash'],rules_hash=frame['rules_hash'],
            teams=dict(red=dict(name='BBT 功能演示 · 红哨兵',version='BBT-demo',mode='team'),
                blue=dict(name='静止靶标 · 无其他策略',version='idle',mode='team')))
        sentry=next(r for r in frame['robots'] if r['id']=='red-7')
        controller=TeamController('red',args.domain,cfg,out,match.arena,sentry,support,api)
        rules=json.loads((args.arena_root/'config/rules.json').read_text())
        controller.serial.purchase_cap=int(rules['rulebook_2027']['economy']['ammo']['17mm']['team_purchase_cap'])
        save(out/'source_manifest.json',dict(config_sha256=digest(cfg_path),runner_sha256=digest(Path(__file__)),
            decision_sha256=digest(ROOT/'install/bbt_decide/lib/bbt_decide/bbt_decide_node'),
            serial_sha256=digest(ROOT/'install/ros2_libxr/lib/ros2_libxr/ros2_libxr_node'),**driver_manifest))
        controller.start();started=time.monotonic();next_source=0.;next_progress=0.;held={};snapshots=[]
        from std_msgs.msg import Float64MultiArray
        operator_topic=cfg.get('topics',{}).get('sim',{}).get('op_cmd_xy') or '/referee/operator_cmd'
        operator=controller.node.create_publisher(Float64MultiArray,operator_topic,10)
        operator_sent=False
        recorder=MatchRecorder(support,out/'replay.json',metadata,{k:frame[k] for k in STATIC_FRAME_KEYS},DT)
        while match.status!='finished':
            if frame['time']>=next_source:controller.bridge.announce_source();next_source=frame['time']+.5
            if match.tick%2==0:
                obs=match.observation('red',[7]);robot=next(r for r in obs['robots'] if r['slot']==7)
                held={team:{str(item['slot']):dict(move=[0.,0.],brake=True,fire=False,crouch=False) for item in lineups[team]} for team in ('red','blue')}
                held['red']['7']=controller.update(robot,obs,frame,api,sdk)
                if frame['time']>=operator_command['time_s'] and not operator_sent:
                    operator.publish(Float64MultiArray(data=[4.,-1.,1.,*operator_command['xy']]))
                    operator_sent=True
                    controller.record('demo_operator',operator_command)
            commands=copy.deepcopy(held);frame=match.step(commands)
            for command in held.get('red',{}).values():
                for key in ('buy_ammo','remote_heal','revive'):command.pop(key,None)
            stats.add(frame,initial,commands);events.extend(frame['events'])
            goal,_,trace=controller.bridge.orders()
            if trace:
                decision=replay_decision(trace,'red',goal,controller.bridge.attack)
                # HUD reflects actual pending engine deliveries, never a
                # synthetic successful purchase or decision-state override.
                decision['safe']['buy_hp']=robot['pending_heal'] is not None
                decision['safe']['buy_ammo']=bool(robot['pending_ammo'])
                frame['sentry_decision']=decision;frame['sentry_decisions']={'red':decision}
            recorder.add(frame,commands)
            if match.tick%2==0:
                r=next(r for r in frame['robots'] if r['id']=='red-7')
                snapshots.append(dict(time_s=frame['time'],hp=r['hp'],ammo=r['ammo'],alive=r['alive'],
                    fold_progress=r['transform_progress'],position=[r['x'],15-r['y']],goal=goal,trace=trace,
                    speed_mps=math.hypot(r['vx'],r['vy']),nav_state=controller.nav.state,
                    red_coins=frame['teams']['red']['resources'],outposts=frame['positions']))
            if frame['time']>=next_progress:
                print(json.dumps(dict(time_s=frame['time'],wall_s=round(time.monotonic()-started,1),
                    phase=trace and trace.get('phase'),point=trace and trace['target_point'],
                    hp=snapshots[-1]['hp'] if snapshots else None,ammo=snapshots[-1]['ammo'] if snapshots else None,
                    purchases=controller.serial.counts)),flush=True);next_progress+=15.
            deadline=started+match.tick*DT
            if deadline>time.monotonic():time.sleep(deadline-time.monotonic())
        result=dict(winner=match.winner,reason=match.reason,status=match.status,time=frame['time'],teams=frame['teams'],
            digest=match.digest(),error=None,feature_demo=True,controller=controller.summary(),
            nav2_routes=controller.bridge.route_events,operator_command=operator_command,
            event_counts=stats.events,shots=stats.shots,other_commanded_move_ticks=stats.other_commands,
            other_max_displacement_m=stats.other_displacement)
        save(out/'result.json',result);save(out/'timeline.json',snapshots);save(out/'events.json',events)
        recorder.close(result,[],frame);recorder=None
        close_controller(controller);controller=None;match.close();match=None
        print('演示已结束，回放已保存；正在验证所有实际物理输入与场景刺激。',flush=True)
        # Factory supplies the exact same explicit scenario, not a ranked match.
        class VerificationMatch(DemoMatch):
            def __init__(self,*pos,**kw):super().__init__(*pos,training_scenario=training,**kw);self.checked=0
            def step(self,commands):
                frame=super().step(commands);self.checked+=1
                if self.checked%1000==0:print(f'回放校验 {self.checked}/{int(args.seconds/DT)}',flush=True)
                return frame
        verification=verify_recording(out/'replay.json',VerificationMatch);save(out/'replay_verification.json',verification)
        checks=demo_checks(snapshots,events,result,cfg,verification)
        manifest=json.loads((out/'source_manifest.json').read_text())
        checks['production_config_unchanged']=digest(cfg_path)==manifest['config_sha256']
        checks['production_binary_unchanged']=digest(ROOT/'install/bbt_decide/lib/bbt_decide/bbt_decide_node')==manifest['decision_sha256']
        validation=dict(passed=all(checks.values()),feature_demo=True,diagnostic=True,checks=checks,
            final_time_s=result['time'],winner=result['winner'],reason=result['reason'],replay=str(out/'replay.json'),
            scripted_state_edits=schedule,initial_training=training,
            physical_pose_edits_during_run=False,production_decision_changed=False)
        save(out/'validation.json',validation);print(json.dumps(validation,ensure_ascii=False,indent=2),flush=True)
        return 0 if validation['passed'] else 1
    except BaseException as exc:
        error=''.join(traceback.format_exception(exc));(out/'failure.txt').write_text(error);raise
    finally:
        if recorder:recorder.frames.closed=True
        if controller:close_controller(controller)
        if match:match.close()
        if old_binary is None:os.environ.pop('RMUC_SIM_BINARY',None)
        else:os.environ['RMUC_SIM_BINARY']=old_binary

class CounterStats:
    def __init__(self):self.events={};self.shots={};self.other_commands=0;self.other_displacement={}
    def add(self,frame,initial,commands):
        for team,robots in commands.items():
            for slot,action in robots.items():
                if team+'-'+slot!='red-7' and (math.hypot(*action.get('move',[0,0]))>.001 or action.get('fire')):self.other_commands+=1
        initial_robots={r['id']:r for r in initial['robots']}
        for r in frame['robots']:
            if r['id']=='red-7':continue
            old=initial_robots[r['id']]
            distance=math.hypot(r['x']-old['x'],r['y']-old['y'])
            self.other_displacement[r['id']]=max(distance,self.other_displacement.get(r['id'],0))
        for event in frame['events']:
            kind=event['type'];self.events[kind]=self.events.get(kind,0)+1
            if kind=='shot':self.shots[event['robot']]=self.shots.get(event['robot'],0)+1

def demo_checks(rows,events,result,cfg,verification):
    after=[r for r in rows if r['time_s']>=60.2]
    points={p['id']:p for p in cfg['points']}
    patrol={r['trace']['target_point'] for r in rows if r['time_s']>=60.2 and r.get('trace') and points.get(r['trace']['target_point'],{}).get('zone')=='patrol'}
    commands=result['controller']['fold_commands'];states=result['controller']['fold_states']
    operator=result['operator_command']
    requested=[row for row in rows if row['time_s']>=operator['time_s'] and
        (row.get('trace') or {}).get('operator_target') and
        row['trace'].get('target_xy') is not None and
        math.dist(row['trace']['target_xy'],operator['xy'])<1e-6]
    return dict(only_sentry_commanded=result['other_commanded_move_ticks']==0,
        only_sentry_shoots=set(result['shots'])<={'red-7'},
        all_outposts_red_at_one_minute=bool(after) and all(r['outposts'][side+'_outpost']['controller']=='red' and r['outposts'][side+'_outpost']['control']=='full' for r in after for side in ('red','blue')),
        tunnel_fold_and_unfold=1 in commands and 2 in commands and 2 in states and states.count(4)>=2,
        patrol_multiple_points=len(patrol)>=2,
        remote_hp_paid_and_delivered=result['controller']['purchases']['hp']>0 and any(e['type']=='remote_heal' and e.get('robot')=='red-7' for e in events),
        remote_ammo_paid_and_delivered=result['controller']['purchases']['ammo']>0 and any(e['type']=='ammo_delivered' and e.get('robot')=='red-7' and not e.get('reason') for e in events),
        low_resource_home_requested=any(r.get('trace') and r['trace']['target_point']==100 and r['time_s']>=135 for r in rows),
        paid_revive=any(e['type']=='respawn' and e.get('robot')=='red-7' and e.get('instant') for e in events),
        operator_coordinate_requested=bool(requested),
        operator_coordinate_arrived=any(row.get('nav_state')==2 and
            math.dist(row['position'],operator['xy'])<=operator['arrival_tolerance_m'] and
            row['speed_mps']<=.12 for row in requested),
        actual_shots=result['shots'].get('red-7',0)>0,
        deterministic_replay=verification['passed'],no_invalid_actions=result['event_counts'].get('invalid_action',0)==0)

def close_controller(controller):
    with controller.bridge.lock:
        future=controller.bridge.route_future
        if future and not future.done():future.set_result(False)
    controller.executor.wake()
    controller.close()

if __name__=='__main__':raise SystemExit(main())
