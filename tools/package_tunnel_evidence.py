#!/usr/bin/env python3
"""Export a compact, re-checkable tunnel suite without large replay/video files.

The checker can be rerun on each included case/trajectory. The full replay and
interface logs are omitted, so retain the original run for deeper investigation.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import zipfile

from build_acceptance_receipt import digest
from check_tunnel_suite import validate

ROOT = Path(__file__).resolve().parents[1]


def package(suite, destination):
    suite = suite.resolve()
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    summary = json.loads((suite/'suite.json').read_text())
    manifest = json.loads((suite/'source_manifest.json').read_text())
    checker = ROOT/'tools/check_tunnel_suite.py'
    recorded_checker = next((value for path, value in manifest.get('files', {}).items()
                             if path.endswith('/tools/check_tunnel_suite.py')), None)
    if recorded_checker != digest(checker):
        raise ValueError('Current checker does not match suite source manifest')
    receipt_info = manifest.get('build_receipt', {})
    if receipt_info.get('status') == 'verified_at_start_and_end':
        if not (suite/'build_receipt.json').is_file() or digest(suite/'build_receipt.json') != receipt_info.get('sha256'):
            raise ValueError('Suite build receipt is missing or does not match')
    files = [(suite/'suite.json', 'suite.json'), (suite/'source_manifest.json', 'source_manifest.json'),
             (ROOT/'tools/check_tunnel_suite.py', 'check_tunnel_suite.py')]
    if (suite/'build_receipt.json').is_file():
        files.append((suite/'build_receipt.json', 'build_receipt.json'))
    case_dirs = sorted(p for p in suite.iterdir() if p.is_dir() and (p/'validation.json').is_file())
    if len(case_dirs) != len(summary):
        raise ValueError('Missing case results; cannot export a complete suite')
    case_results = []
    for case_dir in case_dirs:
        result = json.loads((case_dir/'validation.json').read_text())
        case_results.append(result)
        if result.get('passed'):
            case = json.loads((case_dir/'case.json').read_text())
            rows = [json.loads(line) for line in (case_dir/'trajectory.jsonl').read_text().splitlines()]
            if validate(rows, case) != result:
                raise ValueError(f'Validation does not match recorded result: {case_dir.name}')
        for name in ('case.json', 'trajectory.jsonl', 'validation.json', 'config.diff', 'decision.sim.yaml'):
            path = case_dir/name
            if path.is_file():
                files.append((path, f'{case_dir.name}/{name}'))
    canonical = lambda value: json.dumps(value, sort_keys=True, ensure_ascii=False)
    if Counter(map(canonical, summary)) != Counter(map(canonical, case_results)):
        raise ValueError('Suite summary does not match individual validations')
    entries = {name: digest(path) for path, name in files}
    metadata = {'schema': 'bbt-tunnel-evidence-v1', 'suite_cases': len(summary),
                'build_receipt_status': manifest.get('build_receipt', {}).get('status', 'historical_unverified'),
                'limitations': 'Local, unsigned evidence. Replay, interface logs and ROS logs omitted; retain original run. Hashes do not establish authenticity.',
                'sha256': entries}
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for path, name in files:
            archive.write(path, name)
        archive.writestr('evidence_manifest.json', json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    package(args.suite, args.output)
    print(f'Evidence package: {args.output}')


if __name__ == '__main__':
    main()
