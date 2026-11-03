#!/usr/bin/env python3
"""Build in a fresh directory and record local, inspectable acceptance provenance.

A receipt is execution evidence from this host, not a signed attestation or proof
that an untrusted host/compiler produced the binary from the recorded inputs.
"""
import argparse
import hashlib
import json
import importlib.metadata
import os
import shutil
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('rm2027_interfaces', 'ros2_libxr', 'bbt_decide')
SCHEMA = 'bbt-local-build-receipt-v1'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def command(args, cwd=ROOT, env=None):
    return subprocess.check_output(args, cwd=cwd, env=env, text=True, stderr=subprocess.STDOUT).strip()


def source_snapshot(root=ROOT):
    # Include tracked and nonignored untracked source inputs, including interfaces,
    # package manifests and build definitions; record removals and symlinks.
    names = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard', '--', 'src', 'tools'], cwd=root).split(b'\0')
    files = {}
    for raw in sorted(set(filter(None, names))):
        name = os.fsdecode(raw)
        path = root / name
        if path.is_symlink():
            target = path.resolve(strict=True)
            if not target.is_file():
                raise ValueError(f'Unsupported source symlink: {name}')
            files[name] = {'link': os.readlink(path), 'sha256': digest(target)}
        elif path.is_file():
            files[name] = {'sha256': digest(path), 'executable': bool(path.stat().st_mode & 0o111)}
        else:
            files[name] = {'missing': True}
    return files


def install_snapshot(install):
    files = {}
    for package in PACKAGES:
        directory = install / package
        if not directory.is_dir():
            raise ValueError(f'Missing installed package: {directory}')
        for path in sorted(directory.rglob('*')):
            rel = path.relative_to(install).as_posix()
            if path.is_symlink():
                target = path.resolve(strict=True)
                if target.is_file():
                    files[rel] = {'link': os.readlink(path), 'sha256': digest(target)}
            elif path.is_file():
                files[rel] = {'sha256': digest(path), 'executable': bool(path.stat().st_mode & 0o111)}
    for name in ('bbt_decide/lib/bbt_decide/bbt_decide_node', 'ros2_libxr/lib/ros2_libxr/ros2_libxr_node'):
        if name not in files:
            raise ValueError(f'Missing executable: {name}')
    return files


def build_configuration(build):
    records = {}
    prefixes = ('CMAKE_BUILD_TYPE:', 'CMAKE_C_COMPILER:', 'CMAKE_CXX_COMPILER:',
                'CMAKE_C_FLAGS:', 'CMAKE_CXX_FLAGS:', 'CMAKE_PREFIX_PATH:',
                'BUILD_TESTING:', 'CMAKE_TOOLCHAIN_FILE:')
    for package in PACKAGES:
        base = build / package
        cache = base / 'CMakeCache.txt'
        if not cache.is_file():
            raise ValueError(f'Missing build configuration: {cache}')
        entry = {'cache_sha256': digest(cache),
                 'selected_cache_entries': [s for s in cache.read_text(errors='replace').splitlines() if s.startswith(prefixes)]}
        compile_commands = base / 'compile_commands.json'
        if compile_commands.is_file():
            entry['compile_commands_sha256'] = digest(compile_commands)
        records[package] = entry
    return records


def validate_receipt(receipt, root=ROOT):
    if receipt.get('schema') != SCHEMA or receipt.get('build', {}).get('returncode') != 0:
        raise ValueError('Invalid or unsuccessful build receipt')
    if source_snapshot(root) != receipt['sources']:
        raise ValueError('Source inputs differ from build receipt')
    install = Path(receipt['build']['install_base'])
    if install_snapshot(install) != receipt['installed_files']:
        raise ValueError('Installed artifacts differ from build receipt')
    return install


def create_build(output):
    output.mkdir(parents=True, exist_ok=False)
    build, install, logs = output/'build', output/'install', output/'log'
    workspace = output/'workspace'
    before = source_snapshot()
    # Build from the explicitly recorded source set, excluding ignored local
    # files which might otherwise silently influence CMake or package discovery.
    for name, record in before.items():
        if record.get('missing'):
            raise ValueError(f'Cannot build with missing tracked source: {name}')
        source = ROOT/name
        destination = workspace/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination, follow_symlinks=True)
        if digest(destination) != record['sha256']:
            raise RuntimeError(f'Source changed while copying: {name}')
    commit = command(['git', 'rev-parse', 'HEAD'])
    # Store status as evidence, including untracked paths; no assumption of a clean tree.
    status = command(['git', 'status', '--porcelain=v1', '--untracked-files=all', '--', 'src', 'tools'])
    env = os.environ.copy()
    # Never accidentally discover previous workspace outputs as build dependencies.
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
        if key in env:
            env[key] = os.pathsep.join(p for p in env[key].split(os.pathsep)
                                       if p and not Path(p).resolve().is_relative_to((ROOT/'install').resolve()))
    versions = {}
    for label, args in (('cmake', ['cmake', '--version']), ('cxx', ['c++', '--version']),
                        ('python', [sys.executable, '--version'])):
        try:
            versions[label] = command(args, env=env)
        except (OSError, subprocess.CalledProcessError) as exc:
            versions[label] = f'unavailable: {exc}'
    try:
        versions['colcon_core'] = importlib.metadata.version('colcon-core')
    except importlib.metadata.PackageNotFoundError:
        versions['colcon_core'] = 'unavailable'
    try:
        versions['debian_packages'] = command(['dpkg-query', '-W', '-f=${binary:Package}=${Version}\n'], env=env).splitlines()
    except (OSError, subprocess.CalledProcessError):
        versions['debian_packages'] = None
    build_cmd = ['colcon', '--log-base', str(logs), 'build', '--build-base', str(build), '--install-base', str(install), '--packages-select', *PACKAGES]
    print('Building:', ' '.join(build_cmd), flush=True)
    result = subprocess.run(build_cmd, cwd=workspace, env=env, check=False)
    if result.returncode:
        raise RuntimeError(f'Build failed with exit status {result.returncode}; no receipt issued')
    if source_snapshot() != before or command(['git', 'rev-parse', 'HEAD']) != commit:
        raise RuntimeError('Source or HEAD changed during build; no receipt issued')
    receipt = {'schema': SCHEMA, 'created_utc': datetime.now(timezone.utc).isoformat(),
               'scope': 'Local execution record; not a signed attestation or independent proof of compiler behavior.',
               'git': {'commit': commit, 'source_status': status}, 'sources': before,
               'build': {'command': build_cmd, 'working_directory': str(workspace), 'returncode': result.returncode,
                         'install_base': str(install), 'build_base': str(build),
                         'environment': {k: env.get(k) for k in ('ROS_DISTRO', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CC', 'CXX', 'CFLAGS', 'CXXFLAGS')},
                         'configuration': build_configuration(build), 'versions': versions},
               'installed_files': install_snapshot(install)}
    path = output/'build_receipt.json'
    path.write_text(json.dumps(receipt, indent=2, ensure_ascii=False)+'\n')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New build directory; build, install and receipt are retained here')
    args = parser.parse_args()
    path = create_build(args.output.resolve())
    print(f'Build receipt: {path}\nSource {path.parent / "install/setup.bash"} before acceptance.')


if __name__ == '__main__':
    main()
