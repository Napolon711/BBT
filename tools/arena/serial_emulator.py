"""Arena STM32/referee emulator. Only bytes cross the production ros2_libxr boundary.

The emulator observes engine state to acknowledge purchases; reception alone is not
an acknowledgment. No direct ROS referee/chassis publishers exist here.
"""
import atexit
import importlib.util
import math
import os
from pathlib import Path
import pty
import struct
import subprocess
import time
import tty

DECISION_ROOT = Path(os.environ.get('DECISION_ROOT', str(Path(__file__).resolve().parents[2])))
spec = importlib.util.spec_from_file_location('wire', DECISION_ROOT/'src/ros2_libxr/scripts/fake_lower_machine.py')
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)


def u16(value):
    return max(0, min(65535, int(value)))


class ArenaSerial:
    def __init__(self, team, log_path):
        self.closed = False
        self.team = team
        self.enemy = 'blue' if team == 'red' else 'red'
        self.robot_id = 7 if team == 'red' else 107
        self.master, self.slave = pty.openpty()
        tty.setraw(self.master); tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.parser = wire.Parser()
        self.pending_bytes = bytearray()
        self.seq = 0
        self.move = (0., 0., 0.)
        self.move_rx = -1e9
        self.fold_cmd = 0
        self.mode = 0
        self.counts = {'ammo': 0, 'hp': 0}
        self.revive_requested = False
        self.order = None
        self.inflight = None
        self.periods = {}
        self.rx_count = {}
        self.tx_count = {}
        self.suppress_referee = False
        self.suppress_chassis = False
        self.bad_commands = 0
        self.log = open(log_path, 'w')
        exe = DECISION_ROOT/'install/ros2_libxr/lib/ros2_libxr/ros2_libxr_node'
        self.proc = subprocess.Popen([str(exe), '--ros-args', '-p', f'port:={os.ttyname(self.slave)}'],
                                     stdout=self.log, stderr=self.log)
        atexit.register(self.close)

    def send(self, name, payload):
        self.pending_bytes.extend(wire.pack(name, payload, int(time.monotonic()*1e6)))
        self.tx_count[name] = self.tx_count.get(name, 0)+1
        self.flush()

    def flush(self):
        if self.pending_bytes:
            try:
                n = os.write(self.master, self.pending_bytes)
                del self.pending_bytes[:n]
            except BlockingIOError:
                pass
        if len(self.pending_bytes) > 65536:
            raise RuntimeError('PTY consumer stalled; refusing to silently drop telemetry')

    def pump(self):
        if self.proc.poll() is not None:
            raise RuntimeError('production ros2_libxr exited; inspect serial log')
        self.flush()
        while True:
            try:
                data = os.read(self.master, 65536)
            except BlockingIOError:
                break
            if not data:
                break
            for name, payload, _ in self.parser.feed(data):
                self.rx_count[name] = self.rx_count.get(name, 0)+1
                if name == 'chassis_data' and len(payload) == 12:
                    move = struct.unpack('<fff', payload)
                    if all(math.isfinite(x) for x in move):
                        self.move = move; self.move_rx = time.monotonic()
                elif name == 'chassis_mode' and payload:
                    self.mode = payload[0]
                elif name == 'fold_cmd' and len(payload) == 1 and payload[0] in (0, 1, 2):
                    self.fold_cmd = payload[0]
                elif name == 'ref_tx':
                    self.receive_purchase(payload)

    def receive_purchase(self, payload):
        if len(payload) != 136:
            self.bad_commands += 1; return
        cmd, n, bus = struct.unpack_from('<HHB', payload)
        if cmd != 0x0301 or n != 10 or bus != 0:
            self.bad_commands += 1; return
        sub, sender, receiver, bits = struct.unpack_from('<HHHI', payload, 8)
        if sub != 0x0120 or sender != self.robot_id or receiver != 0x8080:
            self.bad_commands += 1; return
        # Reserved or unsupported actions are not silently interpreted as purchases.
        if (bits >> 2) & 0x7ff or bits >> 24:
            self.bad_commands += 1; return
        self.revive_requested = bool(bits & 2)
        requested = {'ammo': (bits >> 13) & 15, 'hp': (bits >> 17) & 15}
        increments = [k for k in requested if requested[k] == self.counts[k]+1]
        if all(requested[k] == self.counts[k] for k in requested):
            return  # Retransmission of an acknowledged command.
        if len(increments) != 1 or any(requested[k] != self.counts[k] for k in requested if k not in increments):
            self.bad_commands += 1; return
        if self.order is None and self.inflight is None:
            self.order = increments[0]

    def purchase_action(self, sentry):
        if sentry['alive']:
            self.revive_requested = False
        elif self.revive_requested:
            return {'revive': True}
        if self.order is None or self.inflight is not None:
            return {}
        kind, self.order = self.order, None
        self.inflight = (kind, len(sentry['pending_ammo']), sentry['pending_heal'])
        return {'buy_ammo': {'amount': 100, 'remote': True}} if kind == 'ammo' else {'remote_heal': True}

    def observe_purchase(self, sentry):
        if self.inflight is None:
            return
        kind, old_ammo, old_heal = self.inflight
        accepted = (len(sentry['pending_ammo']) > old_ammo if kind == 'ammo'
                    else old_heal is None and sentry['pending_heal'] is not None)
        if accepted:
            self.counts[kind] += 1
        # Rejected requests can be retried by the production command sender.
        self.inflight = None

    def velocity(self):
        return self.move if time.monotonic()-self.move_rx <= 0.25 else (0., 0., 0.)

    def referee(self, obs, sentry, frame, fmap):
        self.observe_purchase(sentry)
        if self.suppress_referee:
            return
        team = obs['teams'][self.team]
        ref = obs['referee']
        robots = {int(r['slot']): r for r in frame['robots'] if r['team'] == self.team}
        hp = lambda slot: u16(robots[slot]['hp'])
        post = ref['outposts']
        progress = {'ready': 0, 'running': 4, 'finished': 5}[ref['status']]
        posture = {'attack': 1, 'defence': 2, 'defense': 2, 'move': 3}.get(sentry['posture'], 0)
        elapsed = obs['time']
        peace = sentry['alive'] and elapsed-sentry['last_shot_time'] >= 6 and elapsed-sentry['last_damage_time'] >= 6
        # The engine uses official 2027 economy; cap comes from its actual rules.
        cap = self.purchase_cap
        exchangeable = max(0, min(2047, cap-int(team['purchased_17'])))
        pos = {slot: fmap.to_field_xy(robots[slot]['x'], robots[slot]['y']) for slot in (1, 3, 4)}
        fx, fy = fmap.to_field_xy(sentry['x'], sentry['y'])
        # 0x0101 position control encoding: 0 own/full,1 own/partial,2 not controlled.
        def control(side):
            p = ref['positions'][side+'_outpost']
            return (0 if p['control'] == 'full' else 1) if p['controller'] == self.team else 2
        event = (control('red') << 28) | (control('blue') << 30)
        revive_cost = math.ceil(elapsed/60)*80 + int(sentry['level'])*20
        free_ready = not sentry['alive'] and sentry['respawn_progress'] >= sentry['respawn_required']
        revive_info = (int(free_ready) << 19) | (int(not sentry['alive']) << 20) | (min(1023,revive_cost) << 21)
        frames = [
            (1, struct.pack('<BHQ', progress << 4 | 1, u16(math.ceil(obs['remaining'])), int(elapsed*1e6))),
            (3, struct.pack('<HHHHhHHHHHHH', hp(1), 0, hp(3), hp(4), 0, hp(7),
                 u16(post['red']['hp']), u16(obs['bases'][self.team]['hp']), u16(post['blue']['hp']),
                 u16(obs['bases'][self.enemy]['hp']), u16(ref['control'][self.team]), u16(ref['control'][self.enemy]))),
            (0x101, struct.pack('<II', event, 0)),
            (0x201, struct.pack('<BBHHHHHfB', self.robot_id, sentry['level'], hp(7), u16(sentry['max_hp']),
                  u16(sentry['cooling_rate']), u16(sentry['max_heat']), u16(sentry['power_limit']), 30.,
                  7 if sentry['powered'] else 0)),
            (0x202, struct.pack('<HHfHHH', 0, 0, 0., u16(sentry['buffer_energy_j']), u16(sentry['heat']), 0)),
            (0x203, struct.pack('<fff', fx, fy, math.degrees(fmap.to_field_yaw(sentry['heading'])))),
            (0x208, struct.pack('<HHHH', u16(sentry['ammo']), 0, u16(team['resources']), 0)),
            (0x20b, struct.pack('<ffIIffffII', *pos[1], 0, 0, *pos[3], *pos[4], 0, 0)),
            (0x20d, struct.pack('<IHQ', (self.counts['ammo'] << 11) | (self.counts['hp'] << 15) | revive_info,
                 int(peace) | exchangeable << 1 | posture << 12, 0)),
        ]
        now = time.monotonic()
        for cmd, payload in frames:
            if now-self.periods.get(cmd, -1e9) < wire.CMD_PERIOD[cmd]:
                continue
            self.periods[cmd] = now; self.seq = (self.seq+1) & 255
            raw = wire.ref_frame(cmd, payload, self.seq)
            self.send('ref_frame', struct.pack('<BBHI', 0, self.seq, len(raw), int(elapsed*1000)) + raw.ljust(128, b'\0'))

    def chassis(self, sentry, fmap, fold_state):
        if self.suppress_chassis:
            return
        vx, vy = fmap.to_field_vec(sentry['vx'], sentry['vy'])
        posture = {'attack': 1, 'defence': 2, 'defense': 2, 'move': 3}.get(sentry['posture'], 0)
        self.send('chassis_state', struct.pack('<fffBBHHI', vx, vy, -sentry['angular_velocity'][2],
                  fold_state, posture, u16(sentry['buffer_energy_j']), u16(sentry['power_w']), 0))
        angle = -(sentry['aim']-sentry['heading'])
        self.send('yawmotor_angle', struct.pack('<f', math.atan2(math.sin(angle), math.cos(angle))))

    def close(self):
        if self.closed: return
        self.closed = True
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill(); self.proc.wait()
        self.log.close(); os.close(self.master); os.close(self.slave)
