#!/usr/bin/env python3
"""Explain recorded fire gating and tunnel centre handoffs without editing capture."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path)
    ap.add_argument('--arena-root',type=Path,default=Path.home()/'rl_workspace/arena1_sim')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();run=args.run.resolve()
    replay=json.loads((run/'replay.json').read_text());meta=replay['metadata'];team=meta['own_team']
    rows=[json.loads(line) for line in (run/team/'interfaces.jsonl').open()]
    controls={round(e['time_s']*20):e['data'] for e in rows if e['kind']=='control'}
    phases=Counter();trace={};coarse=Counter();goals=[]
    for e in rows:
        if e['kind']=='decision_trace':trace=e['data']
        if e['kind']=='goal':goals.append(e)
        if e['kind']!='control':continue
        d=e['data'];phases[str(trace.get('phase',-1))]+=.1
        key=('decision_forbids_fire' if not d['attack_intent'][2] else
             'no_reachable_selected_target' if d['target'] is None else
             'fire_requested' if d['action']['fire'] else 'alignment_or_weapon_gate')
        coarse[key]+=.1
    import yaml
    from run_tunnel_suite import rotate_geometry
    cfg=yaml.safe_load((run/'decision.sim.yaml').read_text())
    centers={tuple(t['center']) for t in rotate_geometry(cfg,team)}
    handoffs=[]
    for e,nxt in zip(goals,goals[1:]):
        xy=e['data']['xy'];start=e['time_s'];end=nxt['time_s']
        if tuple(xy) not in centers:continue
        reached=next((q for q in rows if q['kind']=='control' and start<=q['time_s']<=end+.01
                      and q['data']['goal']==xy and q['data']['nav']==2),None)
        if reached:handoffs.append(dict(center=xy,arrival_s=reached['time_s'],
            next_goal_s=end,handoff_s=round(end-reached['time_s'],3)))
    sys.path.insert(0,str(args.arena_root));from rmuc_arena.engine import Match
    from rmuc_arena import sdk
    from run_bbt_match import auto_aim
    details=Counter();mismatches=0
    match=Match(seed=meta['seed'],duration=meta['duration'],map_name=meta['map'],lineups=meta['lineups'])
    try:
        for tick,action in enumerate(replay['actions']):
            if tick in controls:
                d=controls[tick];intent=d['attack_intent']
                if not intent[2]:details['decision_forbids_fire']+=.1
                else:
                    obs=match.observation(team,[7]);robot=next(r for r in obs['robots'] if r['slot']==7)
                    weapon,target=auto_aim(robot,obs,intent,sdk)
                    mismatches+=weapon['fire']!=d['action']['fire'] or target!=d['target']
                    if target is None:
                        visible=[r for r in obs['enemies'] if r['alive'] and r.get('targetable',True) and not r.get('invulnerable')]
                        details['no_visible_damageable_enemy' if intent[3] and not visible else 'no_reachable_selected_target']+=.1
                    elif not sdk.can_fire(robot):
                        if not robot['alive']:reason='dead'
                        elif robot.get('weak'):reason='weak_after_revive'
                        elif robot['ammo']<=0:reason='no_ammo'
                        elif robot['heat']+10>robot['max_heat']+1e-8 or robot.get('overheated'):reason='heat'
                        else:reason='other_weapon_gate'
                        details[reason]+=.1
                    elif not weapon['fire']:details['not_aimed_yet']+=.1
                    else:details['fire_requested']+=.1
            match.step(action)
            if (tick+1)%1000==0:print(f'analysis {tick+1}/{len(replay["actions"])}',flush=True)
        assert match.digest()==replay['result']['digest']
    finally:match.close()
    result=dict(source=str(run),coarse_seconds=dict(coarse),phase_seconds=dict(phases),
                fire_reason_seconds=dict(details),recomputed_weapon_mismatches=mismatches,
                center_handoffs=handoffs,verified_ticks=len(replay['actions']))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
