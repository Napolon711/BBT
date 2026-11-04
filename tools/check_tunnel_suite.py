#!/usr/bin/env python3
"""Check a fixed-target tunnel run using 20 Hz physical truth and ROS feedback.

Truth is used only for acceptance. The runner drives via ROS and the production
PTY bridge. A rectangle entry or a region change alone never establishes passage.
"""
import argparse
import json
import math
from pathlib import Path


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def roof_for(arena, entry, exit):
    length=math.dist(entry, exit)
    axis=[(exit[i]-entry[i])/length for i in range(2)]
    lateral=[-axis[1], axis[0]]
    matches=[]
    for roof in arena['obstacles']:
        if not roof['id'].endswith('_tunnel_roof'):
            continue
        # Arena obstacles store the unrotated lower-left and rotate about centre.
        centre=[roof['x']+roof['w']/2, 15-(roof['y']+roof['h']/2)]
        offset=[centre[i]-entry[i] for i in range(2)]
        if abs(dot(offset,lateral))>.1 or not 0<dot(offset,axis)<length:
            continue
        angle=-roof.get('rotation',0.)
        u=[math.cos(angle),math.sin(angle)]
        v=[-u[1],u[0]]
        half_axis=abs(dot(u,axis))*roof['w']/2+abs(dot(v,axis))*roof['h']/2
        half_width=abs(dot(u,lateral))*roof['w']/2+abs(dot(v,lateral))*roof['h']/2
        matches.append(dict(id=roof['id'],centre=centre,half_axis=half_axis,
                            half_width=half_width,axis=axis,lateral=lateral))
    if len(matches)!=1:
        raise ValueError(f'expected one physical roof, found {len(matches)}')
    return matches[0]


def validate(rows, case):
    assert rows, 'missing 20 Hz trajectory'
    roof=case['roof'];axis=roof['axis'];lateral=roof['lateral'];centre=roof['centre']
    assert case.get('routing_constraint')=='sentry-no-bumps-v1', 'missing current no-bumps routing contract'
    assert case.get('navigation_interface') in ('ros-nav-goal-cmd-vel-v1','nav2-through-poses-cmd-vel-v2'), 'missing navigation interface contract'
    assert all('bumps_clearance_m' in r and r['bumps_clearance_m']>0 for r in rows), 'sentry footprint entered road bumps'
    assert all('swept_bumps_clearance_m' in r and r['swept_bumps_clearance_m']>0 for r in rows), 'swept sentry footprint entered road bumps between physical samples'
    assert case.get('navigation_clearance_radius_m',0)>=.47 and all(
        r.get('navigation_swept_bumps_clearance_m',-math.inf)>0 for r in rows), 'navigation clearance circle entered road bumps'
    start=case['test_start_s'];exit=case['exit']
    production_goals=[g for g in case['goal_events'] if g['time_s']>=start]
    cursor=0;ordered_goals=[]
    waypoints=(case['entry'],) if case['navigation_interface']=='nav2-through-poses-cmd-vel-v2' else (case['entry'],case['center'],case['exit'])
    for waypoint in waypoints:
        match=next((i for i in range(cursor,len(production_goals))
                    if production_goals[i]['frame_id']=='map' and
                    math.dist(production_goals[i]['xy'],waypoint)<.01),None)
        assert match is not None, 'missing ordered production ROS entry/center/exit goals'
        ordered_goals.append(production_goals[match]);cursor=match+1
    route=None
    if case['navigation_interface']=='nav2-through-poses-cmd-vel-v2':
        route=next((r for r in case.get('route_events',[]) if r['time_s']>=start and
            r.get('action')=='NavigateThroughPoses' and r.get('intermediate_behavior')=='pass_through' and
            len(r['poses'])==2 and math.dist(r['poses'][0],case['center'])<.01 and math.dist(r['poses'][1],case['exit'])<.01),None)
        assert route is not None, 'missing atomic Nav2 via-centre/terminal-exit request'
        assert not any(math.dist(g['xy'],case['center'])<.01 for g in production_goals), 'centre was emitted as a stopping goal'
    assert case.get('preparation_route') and all(
        waypoint.get('reached_s',math.inf)<start and
        math.dist(waypoint.get('actual_pose',[math.inf,math.inf]),waypoint['goal'])<.13
        for waypoint in case['preparation_route']), 'physical preparation route was not completed'
    for a,b in zip(rows,rows[1:]):
        assert abs(b['time']-a['time']-.05)<1e-5, 'trajectory is not continuous 20 Hz'
        assert math.dist(a['position'],b['position'])<.35, 'discontinuous physical movement'
    for row in rows:
        assert row['model']==2773, 'sentry must remain folding mecanum 2773'
    test=[r for r in rows if r['time']>=start]
    assert test and test[0]['transform_progress']<=.001, 'test must start unfolded'
    assert math.dist(test[0]['position'],case['staging'])<.16, 'test must start at the selected outside staging point'
    assert dot([test[0]['position'][i]-case['entry'][i] for i in range(2)],axis)<-.3, 'test must start outside the folding zone'
    folded=next((r for r in test if r['transform_progress']>=.999),None)
    assert folded is not None, 'no physical folding completion'
    along=lambda r:dot([r['position'][i]-centre[i] for i in range(2)],axis)
    cross=lambda r:abs(dot([r['position'][i]-centre[i] for i in range(2)],lateral))
    assert along(folded)<-roof['half_axis']-.2, 'folding completed under the roof'
    moving=[r for r in test if r['time']>=folded['time']]
    gates=[];cursor=0
    tolerance=min(.15,roof['half_width']-case['vehicle_width_m']/2+.03)
    assert tolerance>0, 'vehicle does not fit configured roof width'
    for gate in (-roof['half_axis'],0.,roof['half_axis']):
        found=None
        for i in range(max(cursor,1),len(moving)):
            if along(moving[i-1])<gate<=along(moving[i]):
                found=i;break
        assert found is not None, f'missing real roof crossing at {gate:.3f} m'
        for row in moving[found-1:found+1]:
            assert row['transform_progress']>=.999, 'turret not folded at roof crossing'
            assert row['upright']>.9 and not row['tipped'], 'sentry tipped in tunnel'
            assert cross(row)<=tolerance, 'path misses the physical tunnel axis'
        gates.append(moving[found]);cursor=found
    first,last=gates[0]['time'],gates[-1]['time']
    inside=[r for r in moving if first<=r['time']<=last]
    center_passage=[r for r in moving if math.dist(r['position'],case['center'])<.1]
    center_min_speed=min((r['speed'] for r in center_passage),default=0.)
    if route:
        assert center_passage and center_min_speed>.15, 'sentry stopped/decelerated to arrival at the via centre'
        assert all(r['nav_state']==1 for r in center_passage), 'navigation must remain Running through the intermediate centre'
    assert all(r['transform_progress']>=.999 and r['upright']>.9 and not r['tipped']
               and cross(r)<=tolerance for r in inside), 'unsafe interval under roof'
    end=next((r for r in moving if r['time']>last and r['transform_progress']<=.001),None)
    assert end is not None, 'missing physical unfolding after passage'
    error=math.dist(end['position'],exit)
    assert error<.12, 'unfolding completed away from the exit alignment'
    assert end['speed']<=.15, 'unfolding completed before stopping at the exit'
    assert end['time']-folded['time']<25, 'crossing exceeded the production navigation timeout'
    window=case.get('feedback_window_s',1.)
    feedback=next((r for r in test if end['time']<=r['time']<=end['time']+window
                   and r.get('trace')
                   and r['trace'].get('source')=='fallback'
                   and r['trace'].get('current_region')==case['exit_region_code']
                   and r['trace'].get('tunnel')==-1),None)
    assert feedback is not None, 'missing timely ROS/BBT completion feedback'
    traces=[r['trace'] for r in test if r.get('trace')]
    assert traces and all(t.get('source')=='fallback' for t in traces), 'a non-fallback policy executed'
    assert len({t['seq'] for t in traces})>3, 'decision sequence did not advance'
    assert any(t.get('target_point')==case['target_id'] for t in traces), 'BBT did not select the unchanged discrete target'
    begin=case['serial_at_test_start'];final=case['serial_rx']
    assert final.get('chassis_data',0)>begin.get('chassis_data',0)+10, 'no production PTY motion traffic'
    assert final.get('fold_cmd',0)>=begin.get('fold_cmd',0)+2, 'fold/unfold did not cross PTY'
    assert case.get('trace_count',0)>3, 'no production BBT trace observations'
    continued=[r for r in test if r['time']>=feedback['time']+.3]
    assert continued and all(r['nav_state']!=3 for r in continued), 'normal navigation failed after tunnel exit'
    arrival=next((r for r in continued if math.dist(r['position'],case['target_xy'])<.15
                  and r['speed']<=.15),None)
    assert arrival is not None, 'normal navigation did not reach the unchanged discrete target after exit'
    return dict(passed=True,tunnel=case['tunnel_name'],direction=case['direction'],
                roof=roof['id'],model=2773,folded_s=folded['time'],
                gate_times_s=[r['time'] for r in gates],unfolded_s=end['time'],
                completion_feedback_s=feedback['time'],
                feedback_delay_s=feedback['time']-end['time'],exit_error_m=error,
                crossing_s=end['time']-folded['time'],
                max_roof_lateral_error_m=max(cross(r) for r in inside),
                minimum_bumps_clearance_m=min((r['bumps_clearance_m'] for r in rows if 'bumps_clearance_m' in r),default=None),
                minimum_swept_bumps_clearance_m=min(r['swept_bumps_clearance_m'] for r in rows),
                minimum_navigation_swept_bumps_clearance_m=min(r['navigation_swept_bumps_clearance_m'] for r in rows),
                routing_constraint=case.get('routing_constraint','historical; no bumps restriction checked'),
                navigation_interface=case['navigation_interface'],
                continuation_arrival_s=arrival['time'],
                continuation_target_error_m=math.dist(arrival['position'],case['target_xy']),
                ordered_ros_goals=ordered_goals,
                nav2_route=route,center_min_speed_mps=center_min_speed,
                source='BBT production fallback BT with fixed task stimulus; not a tactical baseline')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folder',type=Path)
    args=ap.parse_args()
    rows=[json.loads(line) for line in (args.folder/'trajectory.jsonl').read_text().splitlines()]
    case=json.loads((args.folder/'case.json').read_text())
    result=validate(rows,case)
    (args.folder/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))


if __name__=='__main__':main()
