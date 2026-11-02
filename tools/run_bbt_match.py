#!/usr/bin/env python3
"""A complete Arena 2027 match: one BBT sentry and nine random existing policies.

The manually selected own sentry runs installed production BBT and serial bridge
in an isolated DDS domain. Only ROS goals, velocity, fold, purchase and fire intent
drive its action. Every other robot has an independent, randomly selected existing
strategy that matches its exact model. The candidate pool and seed are recorded.
No scenario,
spawn, physics, referee rule, health or pose is changed by this runner.

source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 tools/run_bbt_match.py --seconds 420 --domain 208 --output runs/bbt_match
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import json
import math
import os
import random
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import traceback

import yaml

ROOT = Path(__file__).resolve().parents[1]
TEAMS = ('red', 'blue')
OTHER_SLOTS = [1, 3, 4, 6]
LINEUP = [(1, 2711), (3, 2731), (4, 2732), (6, 2761), (7, 2772)]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def random_assignments(policy_root, own_team, seed, load_submission):
    """Choose immutable, exact-model strategies from the existing local cache."""
    rng = random.Random(seed)
    lineups = {team: [dict(slot=slot, model=2773 if slot == 7 and team == own_team else model)
                     for slot, model in LINEUP] for team in TEAMS}
    pool = {}
    for manifest_path in sorted(policy_root.rglob('manifest.json')):
        try:
            info, entries = load_submission(manifest_path.parent)
            if info.get('mode') != 'adaptive' or '2027' not in info.get('rules_profile', ''):continue
            model = int(info['robot'])
            entry = entries[str(model)]
            pool.setdefault(model, []).append(dict(name=manifest_path.parent.name,
                path=str(entry.resolve()), sha256=digest(entry), model=model, version=info['version']))
        except (OSError, ValueError, KeyError, TypeError):continue
    assignments = {};chosen_for_slot = {}
    for team in TEAMS:
        for item in lineups[team]:
            slot, model = item['slot'], item['model']
            if slot == 7 and team == own_team:continue
            compatible = pool.get(model, [])
            if not compatible:raise ValueError(f'no exact-model 2027 policy for model{model}')
            used = chosen_for_slot.get(slot)
            choices = [p for p in compatible if p['name'] != used] or compatible
            assignment = dict(rng.choice(choices), team=team, slot=slot)
            assignments[(team, slot)] = assignment;chosen_for_slot[slot] = assignment['name']
    return lineups, assignments, {str(model): candidates for model,candidates in pool.items() if any(p['model']==model for p in assignments.values())}


def auto_aim(robot, observation, intent, sdk):
    """A public-observation autoaim substitute; BBT retains target/fire authority."""
    kind, target_id, fire, free = intent
    enemy = 'blue' if observation['team'] == 'red' else 'red'
    candidates = []
    if kind == 2 and target_id in (0, 1):
        side = observation['team'] if target_id == 0 else enemy
        target = observation['referee']['outposts'][side]
        if target['hp'] > 0 and target.get('owner', side) == enemy:
            candidates.append(dict(target, kind='outpost', id=side + '-outpost'))
    elif kind == 3:
        target = observation['bases'][enemy]
        if target['hp'] > 0:
            candidates.append(dict(target, kind='base', id=enemy + '-base'))
    elif kind == 1:
        candidates = [r for r in observation['enemies'] if int(r['slot']) == target_id]
    if free and not candidates:
        candidates = [r for r in observation['enemies'] if r['alive'] and
                      r.get('targetable', True) and not r.get('invulnerable')]
    candidates.sort(key=lambda r: math.hypot(r['x'] - robot['x'], r['y'] - robot['y']))
    target = next((r for r in candidates if sdk.ballistic_reachable(robot, r)), None)
    if target is None:
        return dict(aim=robot['aim'], pitch=robot.get('pitch', .04), fire=False), None
    aim, pitch = sdk.aim_at(robot, target), sdk.aim_pitch(robot, target)
    yaw_error = abs((aim - robot['aim'] + math.pi) % math.tau - math.pi)
    aligned = yaw_error < .07 and abs(pitch - robot.get('pitch', 0)) < .06
    return dict(aim=aim, pitch=pitch, fire=bool(fire and aligned and sdk.can_fire(robot))), target['id']


class MatchRecorder:
    """Original v4 replay frames plus every actual engine action and final result."""
    def __init__(self, support, path, metadata, static, step_s):
        self.frames = support.ReplayRecorder(path, metadata, static, hz=5, step_s=step_s)
        self.action_path = path.parent / 'actions.jsonl'
        self.actions = self.action_path.open('wb')
        self.pending_events = []

    def add(self, frame, commands):
        self.pending_events.extend(frame['events'])
        sampled = self.frames._count % self.frames.stride == 0
        self.frames.maybe_add(dict(frame, events=list(self.pending_events)))
        if sampled:self.pending_events.clear()
        self.actions.write(json.dumps(commands, separators=(',', ':'), allow_nan=False).encode() + b'\n')

    def close(self, result, diagnostics, final_frame):
        self.actions.close()
        rec = self.frames
        if not rec._buf or json.loads(rec._buf[-1])['tick'] != final_frame['tick']:
            rec._count = 0
            rec.maybe_add(dict(final_frame, events=list(self.pending_events)))
            self.pending_events.clear()
        with rec.path.open('wb') as out:
            for prefix, value in ((b'{"format":', rec.FORMAT), (b',"metadata":', rec.metadata),
                                  (b',"static":', rec.static)):
                out.write(prefix);out.write(json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode())
            out.write(b',"frames":[')
            for i, frame in enumerate(rec._buf):
                if i:out.write(b',')
                out.write(frame)
            out.write(b'],"actions":[')
            with self.action_path.open('rb') as actions:
                for i, line in enumerate(actions):
                    if i:out.write(b',')
                    out.write(line.rstrip())
            out.write(b'],"diagnostics":');out.write(json.dumps(diagnostics, separators=(',', ':')).encode())
            out.write(b',"result":');out.write(json.dumps(result, separators=(',', ':'), allow_nan=False).encode())
            out.write(b'}')
        rec.closed = True;rec._buf = []


def verify_recording(path, match_type):
    """Re-simulate every recorded input; compare sampled physical frames and digest."""
    replay = json.loads(path.read_text())
    metadata = replay['metadata']
    samples = {frame['tick']: frame for frame in replay['frames']}
    verifier = match_type(seed=metadata['seed'], duration=metadata['duration'],
        map_name=metadata['map'], lineups=metadata['lineups'])
    try:
        checked = 0;pending_events = []
        for commands in replay['actions']:
            frame = verifier.step(commands)
            pending_events.extend(frame['events'])
            sample = samples.get(frame['tick'])
            if sample is not None:
                physical = {key: value for key, value in sample.items() if key not in ('sentry_decision', 'sentry_decisions')}
                actual = {key: value for key, value in frame.items() if key not in replay['static']}
                actual['events'] = list(pending_events);pending_events.clear()
                if physical != actual:raise AssertionError(f'physical replay differs at tick {frame["tick"]}')
                checked += 1
        if verifier.digest() != replay['result']['digest']:raise AssertionError('physical replay final digest differs')
        return dict(passed=True, verified_ticks=len(replay['actions']), verified_frames=checked,
            digest=verifier.digest(), final_time_s=verifier.time)
    finally:verifier.close()


class TeamController:
    def __init__(self, team, domain, config, output, arena, sentry, support, api):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float64MultiArray, Int32, UInt8
        from rm2027_interfaces.msg import SentryCommand
        self.team, self.domain, self.output = team, domain, output / team
        self.output.mkdir()
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node('bbt_match_adapter_' + team, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.fmap = support.FrameMap(team)
        self.nav = support.NavController(self.fmap, arrive_m=.08)
        self.nav.prepare(arena, sentry)
        self.fold = support.EngineFoldReport()
        from run_tunnel_suite import rotate_geometry
        self.monitor = support.TunnelMonitor(rotate_geometry(config, team))
        self.interfaces = (self.output / 'interfaces.jsonl').open('w')
        self.counts, self.fold_commands, self.fold_states, self.purchases = {}, [], [], []
        self.time_s = 0.
        def record(kind, data):
            self.counts[kind] = self.counts.get(kind, 0) + 1
            self.interfaces.write(json.dumps(dict(time_s=self.time_s, kind=kind, data=data), allow_nan=False) + '\n')
        self.record = record
        previous_domain, previous_log = os.environ.get('ROS_DOMAIN_ID'), os.environ.get('ROS_LOG_DIR')
        try:
            os.environ['ROS_DOMAIN_ID'] = str(domain)
            os.environ['ROS_LOG_DIR'] = str(self.output / 'ros_logs')
            self.serial = api['serial'](team, self.output / 'serial.log')
        finally:
            for key, value in (('ROS_DOMAIN_ID', previous_domain), ('ROS_LOG_DIR', previous_log)):
                if value is None:os.environ.pop(key, None)
                else:os.environ[key] = value
        self.bridge = api['bridge'](self.node, config, self.fmap, self.serial, record)
        topics = config.get('topics', {}).get('sim', {})
        topic = lambda key, default: topics.get(key) or default
        def referee(msg):record('referee', list(msg.data))
        def fold_cmd(msg):self.fold_commands.append(msg.data);record('fold_cmd', msg.data)
        def fold_state(msg):
            if not self.fold_states or self.fold_states[-1] != msg.data:
                self.fold_states.append(msg.data);record('fold_state', msg.data)
        def purchase(msg):
            data = [msg.remote_allowance_times, msg.remote_hp_times, msg.exchange_allowance_total]
            self.purchases.append(data);record('purchase', data)
        self.node.create_subscription(Float64MultiArray, topic('referee', '/referee/summary'), referee, 10)
        self.node.create_subscription(UInt8, topic('fold_cmd', '/gimbal_fold_cmd'), fold_cmd, 10)
        self.node.create_subscription(Int32, topic('fold_state', '/gimbal_fold_state'), fold_state, 10)
        self.node.create_subscription(SentryCommand, '/referee/sentry_command', purchase, 10)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.log = (self.output / 'decision.log').open('w')
        self.process = None
        self.match_output = output
        self.distance = 0.;self.previous = self.fmap.to_field_xy(sentry['x'], sentry['y'])
        self.fire_intent_ticks = self.fire_action_ticks = 0

    def start(self):
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOG_DIR=str(self.output / 'ros_logs'))
        self.process = subprocess.Popen([str(ROOT / 'install/bbt_decide/lib/bbt_decide/bbt_decide_node'),
            '--ros-args', '-p', f'config_file:={self.match_output}/decision.sim.yaml', '-p', 'data_source:=sim',
            '-p', f'own_team:={self.team}', '-p', "referee_protocol:='2027'",
            '-p', f'tree_file:={ROOT}/src/bbt_decide/trees/main.xml'], stdout=self.log, stderr=self.log, env=env)

    def update(self, robot, observation, frame, api, sdk):
        if self.process.poll() is not None:
            raise RuntimeError(f'{self.team} BBT exited; see {self.output}/decision.log')
        self.time_s = self.bridge.time_s = frame['time']
        position = self.bridge.publish_observation(robot, observation, frame, self.fold)
        self.distance += math.dist(position, self.previous);self.previous = position
        self.monitor.update(*position)
        self.bridge.navigation(self.nav, self.monitor, robot, frame['arena'])
        self.serial.pump()
        action = api['action'](self.serial, robot, self.fmap)
        with self.bridge.lock:
            intent = list(self.bridge.attack) if time.monotonic() - self.bridge.attack_time < .3 else [0, 0, 0, 0]
        weapon, target = auto_aim(robot, observation, intent, sdk)
        action.update(weapon)
        self.fire_intent_ticks += bool(intent[2]);self.fire_action_ticks += bool(action['fire'])
        self.record('control', dict(position=position, nav=self.nav.state, nav_failure=self.nav.last_failure,
            goal=self.bridge.goal, fold=self.fold.update(robot), attack_intent=intent,
            target=target, action=action, ammo=robot['ammo'], hp=robot['hp']))
        return action

    def summary(self):
        return dict(domain=self.domain, own_team=self.team, distance_m=self.distance,
            ros_rx=self.bridge.rx_count, stream_counts=self.counts, serial_rx=self.serial.rx_count,
            serial_tx=self.serial.tx_count, bad_commands=self.serial.bad_commands,
            fold_commands=self.fold_commands, fold_states=self.fold_states,
            purchases=self.serial.counts, purchase_messages=self.purchases,
            goal_count=len(self.bridge.goal_events), goals=self.bridge.goal_events,
            trace_count=len(self.bridge.trace_events), tunnel_entries=self.monitor.counts,
            fire_intent_ticks=self.fire_intent_ticks, fire_action_ticks=self.fire_action_ticks)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        from geometry_msgs.msg import Twist
        with self.bridge.lock:
            self.bridge.goal=self.bridge.pending_goal=None
            future=self.bridge.route_future
            if future and not future.done():future.set_result(False)
        self.bridge.pub_velocity.publish(Twist());self.executor.wake()
        self.serial.close();self.executor.shutdown();self.thread.join(timeout=3)
        self.node.destroy_node();self.context.shutdown();self.interfaces.close();self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arena-root', type=Path, default=Path(os.environ.get('ARENA_ROOT', Path.home() / 'rl_workspace/arena1_sim')))
    parser.add_argument('--config', type=Path, default=ROOT / 'src/bbt_decide/config/decision.yaml')
    parser.add_argument('--own-team', choices=TEAMS, default='red', help='Only this team sentry runs production BBT')
    parser.add_argument('--policy-root', type=Path, help='Existing 2027 policy cache; files are read without changing them')
    parser.add_argument('--seconds', type=float, default=420)
    parser.add_argument('--domain', type=int, default=208, help='DDS domain for the selected BBT sentry')
    parser.add_argument('--seed', type=int, default=2027)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.domain <= 231 or not math.isfinite(args.seconds) or not 0 < args.seconds <= 420:
        parser.error('domain must be0..231 and seconds must be in(0,420]')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(DECISION_ROOT=str(ROOT), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    sys.path.insert(0, str(args.arena_root.resolve()))
    from rmuc_arena.engine import Match, DT, POLICY_INTERVAL, STATIC_FRAME_KEYS
    from rmuc_arena.runner import Policy
    from rmuc_arena.submissions import load_submission
    from rmuc_arena.communication import TeamCommunication, message_byte_limit
    from rmuc_arena import sdk
    from arena import support
    from arena.bridge import InterfaceBridge, serial_chassis_action, replay_decision
    from arena.serial_emulator import ArenaSerial
    support.sdk = sdk
    api = dict(bridge=InterfaceBridge, serial=ArenaSerial, action=serial_chassis_action)
    config = yaml.safe_load(args.config.read_text())
    transform = config.get('field_to_map', {})
    if not transform.get('enabled') or any(abs(float(transform.get(k, 0))) > 1e-9 for k in ('dx', 'dy', 'dyaw_deg')):
        parser.error('use an identity field/map simulation config')
    shutil.copy2(args.config, output / 'decision.sim.yaml')
    policy_root = args.policy_root or args.arena_root / '.arena_data/strategy-tests/strategies/RMUC2027'
    lineups, assignments, candidate_pool = random_assignments(policy_root, args.own_team, args.seed, load_submission)
    assignment_record = dict(seed=args.seed, own_team=args.own_team, bbt_model=2773,
        candidate_pool=candidate_pool, assignments=list(assignments.values()),
        unique_policy_count=len({p['name'] for p in assignments.values()}))
    (output / 'policy_assignments.json').write_text(json.dumps(assignment_record, indent=2, ensure_ascii=False))
    manifest = dict(config=dict(path=str(args.config.resolve()), sha256=digest(args.config)),
        policies=list(assignments.values()),
        production=dict(decision_sha256=digest(ROOT / 'install/bbt_decide/lib/bbt_decide/bbt_decide_node'),
            serial_sha256=digest(ROOT / 'install/ros2_libxr/lib/ros2_libxr/ros2_libxr_node')),
        adapters={name: digest(ROOT / 'tools' / name) for name in ('run_bbt_match.py', 'arena/bridge.py', 'arena/support.py', 'arena/serial_emulator.py')})
    (output / 'source_manifest.json').write_text(json.dumps(manifest, indent=2))
    match = Match(seed=args.seed, duration=420, map_name='rmuc2027',
        lineups=lineups)
    frame = match.snapshot();controllers = {};policies = {};recorder = None;diagnostics = [];error = None
    communication = TeamCommunication({team:[item['slot'] for item in lineup] for team,lineup in lineups.items()})
    event_counts = {}
    ownership = {team: {str(item['slot']): ('BBT production ROS+PTY' if item['slot'] == 7 and team == args.own_team
        else assignments[(team,item['slot'])]['name']) for item in lineups[team]} for team in TEAMS}
    metadata = dict(api_version=3, engine='rust', season='2027', duration=420, dt=DT,
        policy_interval=POLICY_INTERVAL, seed=args.seed, map='rmuc2027', test_match=args.seconds != 420,
        rules_hash=frame['rules_hash'], engine_hash=frame['engine_hash'],
        teams={team: dict(name='BBT哨兵 + 随机策略队友' if team == args.own_team else '随机策略对手',
            version='BBT-randommix' if team == args.own_team else 'Arena2027-randommix', mode='team') for team in TEAMS},
        lineups=lineups, own_team=args.own_team, random_policy_selection=assignment_record,
        control_ownership=ownership, manual_own_team=True, production_timebase='wall-monotonic-realtime',
        policy_observation_scope='native per-robot camera visibility with Arena TeamCommunication',
        autoaim='SDK public observation; targets/fire permission exclusively from BBT attack_cmd', source_manifest=manifest)
    statistics = {r['id']: dict(distance_m=0., drone_distance_m=0., drone_strikes=0, movement_action_ticks=0, fire_action_ticks=0,
        shots_observed=0, hp_lost=0., initial_hp=r['hp'], initial_ammo=r['ammo'], model=r['model_slot']) for r in frame['robots']}
    previous = {r['id']: r for r in frame['robots']}
    event_file = (output / 'events.jsonl').open('w')
    physics_file = (output / 'physics.jsonl').open('w')
    try:
        rules = json.loads((args.arena_root / 'config/rules.json').read_text())
        purchase_cap = int(rules.get('rulebook_2027', rules.get('official_2027'))['economy']['ammo']['17mm']['team_purchase_cap'])
        team = args.own_team
        sentry = next(r for r in frame['robots'] if r['id'] == team + '-7')
        controllers[team] = TeamController(team, args.domain, config, output, match.arena, sentry, support, api)
        controllers[team].serial.purchase_cap = purchase_cap
        for index,(key,assignment) in enumerate(assignments.items()):
            policies[key] = Policy(Path(assignment['path']), args.seed + index, sandbox='trusted-local', timeout=.5)
        for controller in controllers.values():controller.start()
        recorder = MatchRecorder(support, output / 'replay.json', metadata,
            {k: frame[k] for k in STATIC_FRAME_KEYS if k in frame}, DT)
        started = time.monotonic();next_source = 0.;next_progress = 0.;held = {team: {} for team in TEAMS}
        with ThreadPoolExecutor(max_workers=len(policies)) as policy_pool:
            while frame['time'] < args.seconds - 1e-7 and match.status != 'finished':
                if frame['time'] >= next_source:
                    for controller in controllers.values():controller.bridge.announce_source()
                    next_source = frame['time'] + .5
                if match.tick % POLICY_INTERVAL == 0:
                    # Visibility is determined by the engine's observer slots;
                    # independent policies receive their own camera and messages.
                    observations = {team: match.observation(team, [7]) for team in controllers}
                    policy_observations = {key: match.observation(key[0], [key[1]]) for key in policies}
                    futures = {key: policy_pool.submit(policy.decide, dict(policy_observations[key],
                        messages=communication.inboxes[key[0]][str(key[1])], message_inboxes={str(key[1]):communication.inboxes[key[0]][str(key[1])] }),
                        message_byte_limit(frame['bases'][key[0]], frame.get('rulebook_rules')))
                               for key,policy in policies.items()}
                    held = {team: {} for team in TEAMS}
                    outgoing = []
                    for key, future in futures.items():
                        team,slot = key
                        commands, policy_error = future.result()
                        if policy_error:
                            detail = dict(time=frame['time'], team=team, slot=slot, error=policy_error, logs=list(policies[key].logs))
                            diagnostics.append(detail)
                            raise RuntimeError('baseline policy failed: ' + json.dumps(detail))
                        if set(commands) - {str(slot)}:raise RuntimeError('policy attempted to own another robot')
                        for sender, command in commands.items():
                            if command.get('messages'):outgoing.append((team,sender,command['messages']))
                        held[team].update({slot: {k: v for k, v in command.items() if k != 'messages'} for slot, command in commands.items()})
                    communication.route(outgoing, match.tick, match.tick+POLICY_INTERVAL)
                    for team,controller in controllers.items():
                        robot = next(r for r in observations[team]['robots'] if int(r['slot']) == 7)
                        if '7' in held[team]:raise RuntimeError('other strategy attempted to own BBT sentry')
                        held[team]['7'] = controller.update(robot, observations[team], frame, api, sdk)
                    for team, commands in held.items():
                        for slot, command in commands.items():
                            stats = statistics[team + '-' + slot]
                            stats['movement_action_ticks'] += math.hypot(*command.get('move', [0, 0])[:2]) > .01
                            stats['fire_action_ticks'] += bool(command.get('fire'))
                # Record exact commands before consuming edge-triggered purchases.
                commands = copy.deepcopy(held)
                frame = match.step(commands)
                for team_commands in held.values():
                    for command in team_commands.values():
                        command.pop('buy_ammo', None);command.pop('remote_heal', None);command.pop('revive', None)
                for robot in frame['robots']:
                    old = previous[robot['id']];stats = statistics[robot['id']]
                    stats['distance_m'] += math.hypot(robot['x'] - old['x'], robot['y'] - old['y'])
                    old_drones = {d['id']: d for d in old.get('drones') or []}
                    for drone in robot.get('drones') or []:
                        old_drone = old_drones[drone['id']]
                        stats['drone_distance_m'] += math.dist([drone[k] for k in ('x','y','z')], [old_drone[k] for k in ('x','y','z')])
                        stats['drone_strikes'] += max(0, drone['strikes'] - old_drone['strikes'])
                    stats['shots_observed'] += max(0, old['ammo'] - robot['ammo'])
                    stats['hp_lost'] += max(0, old['hp'] - robot['hp'])
                    stats.update(final_hp=robot['hp'], final_ammo=robot['ammo'], alive=robot['alive'])
                    previous[robot['id']] = robot
                decisions = {}
                for team, controller in controllers.items():
                    goal, _, trace = controller.bridge.orders()
                    if trace is not None:decisions[team] = replay_decision(trace, team, goal, controller.bridge.attack)
                if decisions:
                    frame['sentry_decision'] = decisions.get('red', next(iter(decisions.values())))
                    frame['sentry_decisions'] = decisions
                for event in frame['events']:
                    event_file.write(json.dumps(event, allow_nan=False) + '\n')
                    event_counts[event['type']] = event_counts.get(event['type'],0)+1
                if match.tick % POLICY_INTERVAL == 0:
                    physics_file.write(json.dumps(dict(time=frame['time'], robots=[{k: r[k] for k in ('id','x','y','z','hp','ammo','alive','transform_progress','tipped','upright')} for r in frame['robots']]), allow_nan=False) + '\n')
                recorder.add(frame, commands)
                if frame['time'] >= next_progress:
                    print(json.dumps(dict(time_s=round(frame['time'], 2), wall_s=round(time.monotonic()-started, 2),
                        sentries={team: dict(distance_m=round(c.distance,2), goals=len(c.bridge.goal_events),
                            trace_count=len(c.bridge.trace_events), fire_ticks=c.fire_action_ticks) for team,c in controllers.items()},
                        control={team:frame['teams'][team]['control'] for team in TEAMS})), flush=True)
                    next_progress += 30.
                deadline = started + match.tick * DT
                if deadline > time.monotonic():time.sleep(deadline - time.monotonic())
    except BaseException as exc:
        error = ''.join(traceback.format_exception(exc))
        print(error, file=sys.stderr, flush=True)
    finally:
        result = dict(winner=match.winner, reason=match.reason if match.status == 'finished' else 'stopped',
            status=match.status, time=frame['time'], teams=frame['teams'], digest=match.digest(),
            policy_errors=len(diagnostics), error=error, statistics=statistics,
            event_counts=event_counts, policy_calls={team+'-'+str(slot):policy.calls for (team,slot),policy in policies.items()},
            controllers={team: c.summary() for team,c in controllers.items()}, control_ownership=ownership)
        (output / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
        if recorder:recorder.close(result, diagnostics, frame)
        for controller in controllers.values():controller.close()
        for policy in policies.values():policy.close()
        event_file.close();physics_file.close();match.close()
    verification = dict(passed=False, error='capture failed')
    if error is None:
        try:verification = verify_recording(output / 'replay.json', Match)
        except Exception as exc:verification = dict(passed=False, error=str(exc))
    (output / 'replay_verification.json').write_text(json.dumps(verification, indent=2))
    required = ['chassis_data', 'chassis_mode']
    complete = result['status'] == 'finished' and (result['time'] >= 419.9 or result['reason'] not in ('stopped', ''))
    checks = dict(no_error=error is None, real_match_complete=complete,
        manual_team_isolation=list(controllers) == [args.own_team] and ownership[args.own_team]['7']=='BBT production ROS+PTY',
        production_node_active=all(c.bridge.rx_count.get('decision_trace', 0) > 10 and c.bridge.rx_count.get('nav_goal', 0) > 0 for c in controllers.values()) and len(controllers)==1,
        production_serial_path_active=all(c.serial.bad_commands == 0 and all(c.serial.rx_count.get(k,0)>0 for k in required) for c in controllers.values()) and len(controllers)==1,
        nine_independent_existing_policies=len(policies)==9 and all(p.calls>10 for p in policies.values()),
        varied_random_strategy_choices=assignment_record['unique_policy_count'] >= 5,
        other_robots_moved=all(s['distance_m'] + s['drone_distance_m'] > .5 for robot_id,s in statistics.items() if robot_id!=args.own_team+'-7'),
        teammate_combat=any(s['shots_observed'] > 0 for robot_id,s in statistics.items() if not robot_id.endswith('-7')),
        no_policy_errors=not diagnostics,
        no_invalid_engine_actions=event_counts.get('invalid_action',0)==0,
        deterministic_replay=verification['passed'],
        source_config_unchanged=digest(args.config) == manifest['config']['sha256'])
    diagnostic = args.seconds < 420
    passed = all(value for key,value in checks.items() if not (diagnostic and key == 'real_match_complete'))
    validation = dict(passed=passed, diagnostic=diagnostic, checks=checks, final_time_s=result['time'],
        winner=result['winner'], reason=result['reason'], engine_hash=metadata['engine_hash'], rules_hash=metadata['rules_hash'],
        replay=str(output / 'replay.json'), physical_state_edits=False, control_ownership=ownership)
    (output / 'validation.json').write_text(json.dumps(validation, indent=2, ensure_ascii=False))
    print(json.dumps(validation, ensure_ascii=False, indent=2), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
