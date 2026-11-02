#!/usr/bin/env python3
"""Use the original Arena 3D frontend with an isolated BBT replay catalog.

Run with Arena's existing runtime, without changing or rebuilding the engine:
/home/chengdong/rl_workspace/.venv/bin/python tools/view_arena_replays.py RUN_SUITE --port 8878
The original replay bytes are hard-linked (or copied) into the isolated viewer
catalog. Existing Arena match records and source files are never modified.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('suite',type=Path)
    ap.add_argument('--arena-root',type=Path,default=Path(os.environ.get('ARENA_ROOT',Path.home()/'rl_workspace/arena1_sim')))
    ap.add_argument('--data-root',type=Path,help='Defaults to RUN_SUITE/.arena_viewer; independent from Arena history.')
    ap.add_argument('--port',type=int,default=8878)
    args=ap.parse_args();suite=args.suite.resolve(strict=True)
    if not suite.is_dir() or not 1024<=args.port<=65535:ap.error('expected suite directory and port 1024..65535')
    data=(args.data_root or suite/'.arena_viewer').resolve()
    # Bind a catalog to exactly one capture tree. Reusing it after editing source
    # records would otherwise make a cached hard link display a different run.
    data.mkdir(parents=True,exist_ok=True)
    marker=data/'bbt_capture_root.txt'
    if marker.exists() and marker.read_text()!=str(suite):ap.error('data-root already belongs to another capture tree')
    marker.write_text(str(suite))
    sys.path.insert(0,str(args.arena_root.resolve()))
    from rmuc_arena.store import Store
    from rmuc_arena.server import create_app
    import uvicorn
    store=Store(data);replays=data/'replays';replays.mkdir(exist_ok=True)
    cases=[]
    for path in sorted(suite.rglob('case.json')):
        if data in path.parents:continue
        folder=path.parent;record=folder/'replay.json';validation=folder/'validation.json'
        if not record.is_file() or not validation.is_file():continue
        case=json.loads(path.read_text());check=json.loads(validation.read_text())
        if not check.get('passed'):continue
        replay=json.loads(record.read_text());metadata=copy.deepcopy(replay['metadata'])
        relative=folder.relative_to(suite).as_posix();match_id='bbt-'+hashlib.sha256(relative.encode()).hexdigest()[:12]
        metadata['test_match']=True
        own=case['team'];name=f'BBT {own} / {case["tunnel_index"]+1} {case["tunnel_name"]}'
        metadata['teams'][own].update(name=name,version='BBT-fallback',mode='team')
        other='red' if own=='blue' else 'blue'
        metadata['teams'][other].update(version='idle',mode='team')
        destination=replays/f'{match_id}.json'
        if not destination.exists():
            try:os.link(record,destination)
            except OSError:shutil.copy2(record,destination)
        if store.match(match_id) is None:
            store.create(match_id,metadata)
            store.finish(match_id,dict(winner=None,reason='stopped',time=case['final_time_s'],policy_errors=0),destination)
        cases.append(dict(id=match_id,case=relative,passed=True,replay=str(record)))
    for path in sorted(suite.rglob('result.json')):
        if data in path.parents:continue
        folder=path.parent;record=folder/'replay.json';validation=folder/'validation.json'
        if not record.is_file() or not validation.is_file():continue
        check=json.loads(validation.read_text())
        if not check.get('passed'):continue
        feature_demo=check.get('feature_demo') is True
        if not feature_demo and not check.get('checks',{}).get('real_match_complete'):continue
        result=json.loads(path.read_text());replay=json.loads(record.read_text())
        metadata=copy.deepcopy(replay['metadata'])
        relative=folder.relative_to(suite).as_posix();match_id='bbt-match-'+hashlib.sha256(relative.encode()).hexdigest()[:12]
        destination=replays/f'{match_id}.json'
        if not destination.exists():
            try:os.link(record,destination)
            except OSError:shutil.copy2(record,destination)
        if store.match(match_id) is None:
            store.create(match_id,metadata)
            store.finish(match_id,result,destination)
        cases.append(dict(id=match_id,case=relative,passed=True,replay=str(record),kind='feature-demo' if feature_demo else 'full-match',
            time=result['time'],winner=result['winner'],reason=result['reason']))
    if not cases:ap.error('no complete, passing tunnel cases or full matches found')
    (data/'bbt_cases.json').write_text(json.dumps(cases,indent=2))
    app=create_app(submission_root=data/'submissions',data_root=data,test_mode=True)
    print(f'Original Arena 3D viewer: http://127.0.0.1:{args.port}/',flush=True)
    print(f'{len(cases)} BBT replays registered; open 比赛与回放 and choose 播放.',flush=True)
    uvicorn.run(app,host='127.0.0.1',port=args.port,log_level='warning')


if __name__=='__main__':main()
