#!/usr/bin/env python3
"""Check recorded BBT/Arena evidence, never infer crossing from region bookkeeping."""
import argparse
import json
import math
from pathlib import Path
import yaml


def check(path, tunnel=False, purchase=False, recovery=False):
    result=json.loads((path/'result.json').read_text())
    samples=json.loads((path/'physics.json').read_text())
    records=[json.loads(s) for s in (path/'interfaces.jsonl').read_text().splitlines()]
    config=yaml.safe_load((path/'decision.sim.yaml').read_text())
    checks={
        'referee_from_serial':result['serial_tx'].get('ref_frame',0)>0 and result['ros_counts'].get('referee',0)>0,
        'movement_from_serial':result['serial_rx'].get('chassis_data',0)>0 and result['distance_m']>.5,
        'no_invalid_commands':result['bad_commands']==0,
        'has_goals':result['goal_count']>0,
    }
    completed=[]
    for definition in config.get('tunnels',[]):
        for entry,exit in [(definition['align'][0]['p'],definition['align'][1]['p']),
                           (definition['align'][1]['p'],definition['align'][0]['p'])]:
            arrived=[s for s in samples if s['goal'] and math.dist(s['goal'],entry)<.01 and s['nav']==2 and math.dist(s['pos'],entry)<=.1]
            for start in arrived:
                end=next((s for s in samples if s['time']>start['time'] and s['goal'] and math.dist(s['goal'],exit)<.01 and s['nav']==2 and math.dist(s['pos'],exit)<=.1 and s['fold']==2),None)
                if end:
                    unfolded=next((s for s in samples if s['time']>end['time'] and s['fold']==4),None)
                    if unfolded:
                        completed.append({'name':definition['name'],'entry':entry,'exit':exit,
                                          'entry_s':start['time'],'exit_s':end['time'],'unfolded_s':unfolded['time']})
                        break
    if tunnel:
        checks['completed_physical_crossing']=bool(completed)
        checks['fold_and_unfold_commands']=1 in result['fold_commands'] and 2 in result['fold_commands']
        if completed:
            # Ensure the run continues after unfolding; merely parking at the exit is insufficient.
            t=completed[0]
            checks['moves_after_exit']=any(s['time']>t['unfolded_s'] and math.dist(s['pos'],t['exit'])>.8 for s in samples)
    buys=[e for e in result['events'] if e.get('type')=='ammo_purchase' and e.get('remote')]
    deliveries=[e for e in result['events'] if e.get('type')=='ammo_delivered']
    if purchase:
        requests=[r for r in records if r['kind']=='purchase']
        checks['actual_remote_purchase']=bool(buys) and result['purchases']['ammo']==len(buys)
        checks['actual_ammo_delivery']=any(d['robot']==b['robot'] and d['amount']==b['amount']==100 and abs(d['time']-b['time']-6)<.11 for b in buys for d in deliveries)
        checks['remote_counter_not_local_amount']=bool(requests) and all(r['data'][2]==0 for r in requests)
        checks['retransmission_idempotent']=len(requests)>len(buys) and result['purchases']['ammo']==len(buys)
    if recovery:
        fault=result.get('chassis_fault')
        checks['fault_exercised']=fault is not None
        if fault:
            mid=[s for s in samples if fault[0]+2.2<s['time']<fault[1]]
            checks['stopped_during_feedback_loss']=len(mid)>1 and max(math.dist(mid[0]['pos'],s['pos']) for s in mid)<.15
            commands=[r for r in records if r['kind']=='fold_cmd' and r['data']==1]
            checks['fold_reissued']=len(commands)>=2 and commands[-1]['wall_s']>fault[0]+1
            checks['resumed_after_feedback']=bool(completed) and any(t['exit_s']>fault[1] for t in completed)
    report={'passed':all(checks.values()),'checks':checks,'crossings':completed,
            'distance_m':result['distance_m'],'remote_ammo_purchases':len(buys),'remote_ammo_deliveries':len(deliveries)}
    (path/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('path',type=Path)
    ap.add_argument('--tunnel',action='store_true');ap.add_argument('--purchase',action='store_true');ap.add_argument('--recovery',action='store_true')
    args=ap.parse_args();report=check(args.path,args.tunnel,args.purchase,args.recovery)
    print(json.dumps(report,ensure_ascii=False,indent=2));raise SystemExit(0 if report['passed'] else 1)
