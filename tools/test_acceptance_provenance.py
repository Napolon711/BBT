"""Regression tests for build receipts and shareable acceptance evidence."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_acceptance_receipt as receipt
import package_tunnel_evidence as package


class ReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        subprocess.run(['git', 'init', '-q', str(self.root)], check=True)
        (self.root/'src/pkg').mkdir(parents=True)
        (self.root/'src/pkg/a.cpp').write_text('old')
        subprocess.run(['git', '-C', str(self.root), 'add', 'src'], check=True)
        self.install = self.root/'install'
        for name in ('rm2027_interfaces/include/interface.h',
                     'bbt_decide/lib/bbt_decide/bbt_decide_node',
                     'ros2_libxr/lib/ros2_libxr/ros2_libxr_node'):
            path = self.install/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(name)
        self.record = {'schema': receipt.SCHEMA, 'build': {'returncode': 0, 'install_base': str(self.install)},
                       'sources': receipt.source_snapshot(self.root),
                       'installed_files': receipt.install_snapshot(self.install)}

    def test_source_change_and_untracked_source_are_rejected(self):
        receipt.validate_receipt(self.record, self.root)
        path = self.root/'src/pkg/a.cpp'
        path.write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Source inputs'):
            receipt.validate_receipt(self.record, self.root)
        path.write_text('old')
        (self.root/'src/pkg/new.hpp').write_text('untracked')
        with self.assertRaisesRegex(ValueError, 'Source inputs'):
            receipt.validate_receipt(self.record, self.root)

    def test_installed_binary_or_symlink_replacement_is_rejected(self):
        node = self.install/'bbt_decide/lib/bbt_decide/bbt_decide_node'
        node.write_text('replaced')
        with self.assertRaisesRegex(ValueError, 'artifacts'):
            receipt.validate_receipt(self.record, self.root)
        node.unlink()
        node.symlink_to(self.install/'ros2_libxr/lib/ros2_libxr/ros2_libxr_node')
        with self.assertRaisesRegex(ValueError, 'artifacts'):
            receipt.validate_receipt(self.record, self.root)

    def test_failed_or_invalid_receipt_is_rejected(self):
        self.record['build']['returncode'] = 1
        with self.assertRaisesRegex(ValueError, 'unsuccessful'):
            receipt.validate_receipt(self.record, self.root)


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.suite = self.root/'suite'
        self.suite.mkdir()
        self.result = {'passed': True, 'tunnel': 'example'}
        (self.suite/'suite.json').write_text(json.dumps([self.result]))
        checker = package.ROOT/'tools/check_tunnel_suite.py'
        self.manifest = {'files': {str(checker): receipt.digest(checker)}, 'build_receipt': {'status': 'not_provided'}}
        (self.suite/'source_manifest.json').write_text(json.dumps(self.manifest))
        self.case = self.suite/'0_example_forward_red'
        self.case.mkdir()
        (self.case/'case.json').write_text('{}')
        (self.case/'trajectory.jsonl').write_text('{}\n')
        (self.case/'validation.json').write_text(json.dumps(self.result))
        (self.case/'replay.json').write_text('large replay omitted')

    def test_export_contains_recheck_inputs_not_large_replay(self):
        output = self.root/'evidence.zip'
        with patch.object(package, 'validate', return_value=self.result):
            package.package(self.suite, output)
        with zipfile.ZipFile(output) as archive:
            self.assertIn('0_example_forward_red/trajectory.jsonl', archive.namelist())
            self.assertNotIn('0_example_forward_red/replay.json', archive.namelist())
            manifest = json.loads(archive.read('evidence_manifest.json'))
            self.assertEqual(manifest['build_receipt_status'], 'not_provided')

    def test_tampered_case_or_summary_cannot_be_exported(self):
        with patch.object(package, 'validate', return_value={'passed': True, 'tunnel': 'different'}):
            with self.assertRaisesRegex(ValueError, 'Validation does not match'):
                package.package(self.suite, self.root/'bad.zip')
        (self.suite/'suite.json').write_text(json.dumps([{'passed': False}]))
        with patch.object(package, 'validate', return_value=self.result):
            with self.assertRaisesRegex(ValueError, 'summary does not match'):
                package.package(self.suite, self.root/'bad2.zip')

    def test_receipt_hash_mismatch_is_rejected(self):
        self.manifest['build_receipt'] = {'status': 'verified_at_start_and_end', 'sha256': 'bad'}
        (self.suite/'source_manifest.json').write_text(json.dumps(self.manifest))
        (self.suite/'build_receipt.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'receipt'):
            package.package(self.suite, self.root/'bad.zip')

    def test_runner_rejects_nonempty_output_before_launch(self):
        result = subprocess.run([sys.executable, str(package.ROOT/'tools/run_tunnel_suite.py'),
                                 '--output', str(self.suite)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('must be empty', result.stderr)
        self.assertEqual(json.loads((self.suite/'suite.json').read_text()), [self.result])

    def test_runner_requires_a_build_receipt(self):
        output = self.root/'suite_without_receipt'
        result = subprocess.run([sys.executable, str(package.ROOT/'tools/run_tunnel_suite.py'),
                                 '--output', str(output)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--build-receipt is required', result.stderr)
        self.assertFalse((output/'source_manifest.json').exists())


if __name__ == '__main__':
    unittest.main()
