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
    def node(self):
        from builtin_interfaces.msg import Time

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        publishers, subscriptions = {}, {}
        def create_publisher(message_type, topic, depth):
            publishers[topic] = Publisher()
            return publishers[topic]
        def create_subscription(message_type, topic, callback, depth):
            subscriptions[topic] = callback
            return SimpleNamespace()
        return SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)),
            create_publisher=create_publisher, create_subscription=create_subscription,
            publishers=publishers, subscriptions=subscriptions)

    def test_ordinary_goal_and_completion_use_topic_boundary(self):
        from std_msgs.msg import Int32
        node = self.node()
        with patch('rclpy.action.ActionClient') as action_client:
            navigator = PreparationNavigator(node)
        self.assertEqual(action_client.call_count, 1)
        self.assertEqual(action_client.call_args.args[-1], '/navigate_through_poses')
        navigator.start(dict(side='staging', goal=[2.,5.]))
        messages = node.publishers['/goal_pose'].messages
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0].header.frame_id, 'map')
        self.assertEqual((messages[0].pose.position.x,messages[0].pose.position.y), (2.,5.))
        status = node.subscriptions['/nav_status']
        status(Int32(data=2))  # A previous leg's result must not finish this goal.
        self.assertEqual(navigator.status(), (False,None))
        status(Int32(data=1))
        status(Int32(data=2))
        self.assertEqual(navigator.status(), (True,None))
        navigator.cancel()
        self.assertEqual(len(node.publishers['/nav_cancel'].messages), 1)
        status(Int32(data=3))
        self.assertEqual(navigator.status(), (False,None))

    def test_ordinary_retry_republishes_only_after_physical_stop(self):
        class Client:
            def __init__(self, *_):
                self.goals = []

            def send_goal_async(self, goal):
                self.goals.append(goal)
                return Future()

        from std_msgs.msg import Int32
        node = self.node()
        with patch('rclpy.action.ActionClient', Client):
            navigator = PreparationNavigator(node)
        navigator.start(dict(side='staging', goal=[2.,5.]))
        status = node.subscriptions['/nav_status']
        status(Int32(data=1))
        status(Int32(data=3))
        self.assertFalse(navigator.retry_after_stop(.3, 10.))
        self.assertEqual(len(node.publishers['/goal_pose'].messages), 1)
        self.assertEqual(len(node.publishers['/nav_cancel'].messages), 1)
        for index in range(10):
            self.assertFalse(navigator.retry_after_stop(0., 10.+index*.05))
        self.assertTrue(navigator.retry_after_stop(0., 10.5))
        self.assertEqual(len(node.publishers['/goal_pose'].messages), 2)
        self.assertEqual(navigator.status(), (False,None))
        navigator.error = 'second blocked'
        navigator.retry_after_stop(0., 11.)
        self.assertTrue(navigator.retry_after_stop(0., 11.5))
        navigator.error = 'third blocked'
        with self.assertRaisesRegex(RuntimeError, 'retry limit'):
            navigator.retry_after_stop(0., 12.)
        self.assertEqual(len(node.publishers['/goal_pose'].messages), 3)

        navigator.start(dict(side='exit', center=[2.,4.2], goal=[2.,5.]))
        navigator.error = 'tunnel failure'
        with self.assertRaisesRegex(RuntimeError, 'tunnel failure'):
            navigator.retry_after_stop(0., 13.)
        self.assertEqual(len(node.publishers['/goal_pose'].messages), 3)

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

        node = self.node()
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
