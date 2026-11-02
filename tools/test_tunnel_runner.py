"""Generated acceptance configurations remain valid production inputs."""
import sys
import unittest
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_tunnel_suite import ROOT, PreparationNavigator, case_config


class TunnelRunnerTests(unittest.TestCase):
    def test_ordinary_retry_requires_explicit_new_plan_after_physical_stop(self):
        class Client:
            def __init__(self, *_):
                self.goals = []

            def send_goal_async(self, goal):
                self.goals.append(goal)
                return Future()

        from builtin_interfaces.msg import Time
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)))
        with patch('rclpy.action.ActionClient', Client):
            navigator = PreparationNavigator(node)
        navigator.start(dict(side='staging', goal=[2.,5.]))
        navigator.error = 'physical controller blocked'
        self.assertFalse(navigator.retry_after_stop(.3, 10.))
        self.assertEqual(len(navigator.compute.goals), 1)
        for index in range(10):
            self.assertFalse(navigator.retry_after_stop(0., 10.+index*.05))
        self.assertTrue(navigator.retry_after_stop(0., 10.5))
        self.assertEqual(len(navigator.compute.goals), 2)
        self.assertEqual(navigator.status(), (False,None))
        navigator.error = 'second blocked'
        navigator.retry_after_stop(0., 11.)
        self.assertTrue(navigator.retry_after_stop(0., 11.5))
        navigator.error = 'third blocked'
        with self.assertRaisesRegex(RuntimeError, 'retry limit'):
            navigator.retry_after_stop(0., 12.)
        self.assertEqual(len(navigator.compute.goals), 3)

        navigator.start(dict(side='exit', center=[2.,4.2], goal=[2.,5.]))
        navigator.error = 'tunnel failure'
        with self.assertRaisesRegex(RuntimeError, 'tunnel failure'):
            navigator.retry_after_stop(0., 13.)
        self.assertEqual(len(navigator.compute.goals), 3)

    def test_cancel_cancels_goal_accepted_after_request_was_canceled(self):
        class DelayedClient:
            def __init__(self, *_):
                self.acceptance = Future()

            def send_goal_async(self, _):
                return self.acceptance

        class LateHandle:
            accepted = True
            canceled = 0

            def cancel_goal_async(self):
                self.canceled += 1

            def get_result_async(self):
                raise AssertionError('canceled stale goal must not install a result callback')

        from builtin_interfaces.msg import Time
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)))
        with patch('rclpy.action.ActionClient', DelayedClient):
            navigator = PreparationNavigator(node)
        navigator.start(dict(side='exit', center=[2.,4.2], goal=[2.,5.]))
        navigator.cancel()
        late = LateHandle()
        navigator.route.acceptance.set_result(late)
        self.assertEqual(late.canceled, 1)
        self.assertEqual(navigator.status(), (False, None))

    def test_single_target_fixture_updates_explicit_patrol_selection(self):
        original = yaml.safe_load((ROOT/'src/bbt_decide/config/decision.yaml').read_text())
        for index in range(len(original['tunnels'])):
            for direction in ('forward', 'reverse'):
                with self.subTest(index=index, direction=direction):
                    generated, case = case_config(original, index, direction, 'red')
                    self.assertEqual(generated['patrol_point_ids'], [case['target_id']])
                    self.assertEqual([p['id'] for p in generated['points'] if p['zone']=='patrol'],
                                     [case['target_id']])
                    self.assertEqual([(p['id'],p['x'],p['y'],p['region']) for p in generated['points']],
                                     [(p['id'],p['x'],p['y'],p['region']) for p in original['points']])


if __name__ == '__main__':
    unittest.main()
