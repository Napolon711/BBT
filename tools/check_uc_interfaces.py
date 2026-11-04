#!/usr/bin/env python3
"""Compare BBT's used message definitions and default topic wiring with 27BT-UC."""
import argparse
import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def fields(path):
    return [' '.join(line.split('#', 1)[0].split())
            for line in path.read_text().splitlines() if line.split('#', 1)[0].strip()]


def check(reference):
    errors = []
    local = ROOT / 'src/rm2027_interfaces/msg'
    upstream = reference / 'src/rm2027_interfaces/msg'
    messages = []
    for path in sorted(local.glob('*.msg')):
        other = upstream / path.name
        if not other.exists() or fields(path) != fields(other):
            errors.append(f'message differs: {path.name}')
        messages.append(path.name)
    bbt = yaml.safe_load((ROOT / 'src/bbt_decide/config/decision.yaml').read_text())
    uc = yaml.safe_load((reference / 'src/sentry_decide/config/decision.yaml').read_text())
    shared = ['nav_goal', 'nav_status', 'nav_cancel', 'odom', 'cmd_vel', 'chassis_mode',
              'spin', 'fold_cmd', 'fold_state', 'referee', 'buy_ammo', 'buy_hp',
              'data_source', 'iface_status', 'attack_cmd']
    for mode in ['sim', 'real']:
        for key in shared:
            other = 'aim_cmd' if key == 'attack_cmd' else key
            if bbt['topics'][mode][key] != uc['topics'][mode][other]:
                errors.append(f'topic differs: {mode}.{key}')
    for bbt_point in bbt['points']:
        uc_point = next((p for p in uc['points'] if p['id'] == bbt_point['id']), None)
        if uc_point is None or any(bbt_point[k] != uc_point[k] for k in ['x', 'y', 'region']):
            errors.append(f'canonical point differs: {bbt_point["id"]}')
    if bbt['tunnels'] != uc['tunnels']:
        errors.append('canonical tunnel geometry differs')
    return dict(passed=not errors, reference=str(reference.resolve()),
                messages=messages, shared_topic_fields=shared,
                optional_bbt_inputs=['/referee/operator_cmd', '/sentry/operator_point'],
                purchase_execution='/referee/sentry_command', errors=errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True, help='27BT-UC workspace root')
    args = parser.parse_args()
    report = check(args.reference)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
