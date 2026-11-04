"""Navigation adapter contract tests; no engine or serial device is opened."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from arena.bridge import InterfaceBridge
from arena import support
from arena.support import FrameMap, NavController


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class StubNode:
    def __init__(self):
        self.actions = []

    def create_publisher(self, *_):
        return RecordingPublisher()

    def create_subscription(self, *_):
        pass

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))


class CallbackActionServer:
    def __init__(self, node, action, name, **callbacks):
        node.actions.append(name)
        self.callbacks = callbacks


class ActionHandle:
    def __init__(self, points):
        poses = [pose(*point) for point in points]
        self.request = SimpleNamespace(poses=poses)
        self.is_cancel_requested = False
        self.status = None

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'


def pose(x, y):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    return message


def bridge(config=None):
    config = config or {'tunnels': [{'center': [2., 4.],
        'align': [{'p': [2., 3.]}, {'p': [2., 5.]}]}]}
    with patch('rclpy.action.ActionServer', CallbackActionServer):
        return InterfaceBridge(StubNode(), config, FrameMap('red'),
                               SimpleNamespace(team='red'))


def finish(coroutine):
    try:
        coroutine.send(None)
    except StopIteration as done:
        return done.value
    raise AssertionError('Action still waiting')


class NavigationContractTests(unittest.TestCase):
    def test_only_configured_tunnel_route_has_a_simulated_action_server(self):
        self.assertEqual(bridge().node.actions, ['/navigate_through_poses'])

    def test_action_canceled_before_execution_cannot_install_a_goal(self):
        adapter = bridge()
        handle = ActionHandle([(2., 4.), (2., 5.)])
        handle.is_cancel_requested = True
        result = finish(adapter.route_server.callbacks['execute_callback'](handle))
        self.assertIsNotNone(result)
        self.assertEqual(handle.status, 'canceled')
        self.assertIsNone(adapter.orders()[0])

    def test_route_requires_explicit_center_instead_of_alignment_midpoint(self):
        from rclpy.action import GoalResponse
        adapter = bridge({'tunnels': [{'center': [2., 4.2],
            'align': [{'p': [2., 3.]}, {'p': [2., 5.]}]}]})
        configured = ActionHandle([(2., 4.2), (2., 5.)]).request
        midpoint = ActionHandle([(2., 4.), (2., 5.)]).request
        self.assertEqual(adapter.route_server.callbacks['goal_callback'](configured),
                         GoalResponse.ACCEPT)
        self.assertEqual(adapter.route_server.callbacks['goal_callback'](midpoint),
                         GoalResponse.REJECT)

    def test_tunnel_recovery_accepts_configured_exit_without_returning_to_center(self):
        from rclpy.action import GoalResponse
        adapter = bridge()
        request = ActionHandle([(2., 5.)]).request
        self.assertEqual(adapter.route_server.callbacks['goal_callback'](request),
                         GoalResponse.ACCEPT)
        request = ActionHandle([(7., 8.)]).request
        self.assertEqual(adapter.route_server.callbacks['goal_callback'](request),
                         GoalResponse.REJECT)

    def test_goal_pose_executes_only_the_requested_final_goal(self):
        adapter = bridge()
        adapter.on_goal(pose(2, 3))
        self.assertEqual(adapter.orders()[0], (2., 3.))
        adapter.on_goal(pose(7, 8))
        self.assertEqual(adapter.orders()[0], (7., 8.))
        self.assertEqual(adapter.goal_events[-1]['xy'], [7., 8.])

    def test_topic_cancel_stops_and_removes_pending_goal_before_replacement(self):
        adapter = bridge()
        adapter.on_goal(pose(2, 3))
        adapter.on_cancel(Empty())
        self.assertIsNone(adapter.orders()[0])
        self.assertEqual(adapter.pub_velocity.messages[-1].linear.x, 0.)
        self.assertEqual(adapter.pub_velocity.messages[-1].linear.y, 0.)
        adapter.on_goal(pose(7, 8))
        self.assertEqual(adapter.orders()[0], (7., 8.))

    def test_topic_request_starts_before_immediate_terminal_result(self):
        adapter = bridge()
        adapter.on_goal(pose(2, 3))
        self.assertEqual([msg.data for msg in adapter.pub_nav.messages], [1])

    def test_topic_goal_supersedes_tunnel_action_before_its_cancel_arrives(self):
        adapter = bridge()
        handle = ActionHandle([(2., 4.), (2., 5.)])
        action = adapter.route_server.callbacks['execute_callback'](handle)
        done = action.send(None)
        try:
            adapter.on_goal(pose(7, 8))
            self.assertEqual(adapter.orders()[0], (7., 8.))
            self.assertTrue(done.done())
            adapter.route_server.callbacks['cancel_callback'](handle)
            self.assertEqual(adapter.orders()[0], (7., 8.))
        finally:
            if not done.done():
                done.set_result(False)
            finish(action)
        self.assertEqual(handle.status, 'aborted')
        self.assertEqual(adapter.orders()[0], (7., 8.))

    def test_cancel_removes_already_queued_goal_but_allows_later_goal(self):
        adapter = bridge()
        handle = ActionHandle([(2., 4.), (2., 5.)])
        action = adapter.route_server.callbacks['execute_callback'](handle)
        action.send(None)
        handle.is_cancel_requested = True
        adapter.route_server.callbacks['cancel_callback'](handle)
        self.assertIsNone(adapter.orders()[0])
        self.assertEqual(adapter.pub_velocity.messages[-1].linear.x, 0.)
        self.assertEqual(adapter.pub_velocity.messages[-1].linear.y, 0.)
        finish(action)
        newer = ActionHandle([(2., 4.), (2., 3.)])
        replacement = adapter.route_server.callbacks['execute_callback'](newer)
        done = replacement.send(None)
        self.assertEqual(adapter.orders()[0], (2., 3.))
        adapter.route_server.callbacks['cancel_callback'](handle)
        self.assertEqual(adapter.orders()[0], (2., 3.))
        done.set_result(False)
        finish(replacement)

    def test_old_action_cancel_cannot_remove_subsequent_topic_goal(self):
        adapter = bridge()
        handle = ActionHandle([(2., 4.), (2., 5.)])
        action = adapter.route_server.callbacks['execute_callback'](handle)
        action.send(None)
        adapter.route_server.callbacks['cancel_callback'](handle)
        adapter.on_goal(pose(7, 8))
        try:
            adapter.route_server.callbacks['cancel_callback'](handle)
            self.assertEqual(adapter.orders()[0], (7., 8.))
        finally:
            finish(action)

    def test_old_goal_result_cannot_complete_new_route_or_emit_old_velocity(self):
        adapter = bridge()
        old_handle = ActionHandle([(2., 4.), (2., 3.)])
        old_route = adapter.route_server.callbacks['execute_callback'](old_handle)
        old_route.send(None)
        route_handle = ActionHandle([(2., 4.), (2., 5.)])
        route = adapter.route_server.callbacks['execute_callback'](route_handle)
        monitor = SimpleNamespace(crossing=lambda *_: None, corridors=[],
                                  is_alignment=lambda _: False)
        waiting = []

        class ReplacedDuringNavigation:
            state = 2

            def update(self, *_args, **_kwargs):
                waiting.append(route.send(None))
                return 1., 1.

        try:
            adapter.navigation(ReplacedDuringNavigation(), monitor,
                               {'x': 2., 'y': 12.}, {})
            self.assertFalse(waiting[0].done(), 'old success completed the new Action')
            self.assertEqual(adapter.pub_velocity.messages, [])
            self.assertEqual(adapter.pub_nav.messages, [])
        finally:
            if waiting and not waiting[0].done():
                waiting[0].set_result(False)
            try:
                route.send(None)
            except StopIteration:
                pass
            route.close()
            finish(old_route)

    def test_tunnel_route_waits_for_final_exit_controller_completion(self):
        adapter = bridge()
        handle = ActionHandle([(2., 4.), (2., 5.)])
        action = adapter.route_server.callbacks['execute_callback'](handle)
        done = action.send(None)
        monitor = SimpleNamespace(crossing=lambda *_: None, corridors=[],
                                  is_alignment=lambda _: False)

        class PhysicalController:
            state = 1
            goals = []

            def update(self, goal, *_args, **_kwargs):
                self.goals.append(goal)
                return 0., 1.

        controller = PhysicalController()
        adapter.navigation(controller, monitor, {'x': 2., 'y': 12.}, {})
        self.assertFalse(done.done())
        self.assertEqual(controller.goals, [(2., 5.)])
        controller.state = 2
        adapter.navigation(controller, monitor, {'x': 3., 'y': 10.}, {})
        finish(action)
        self.assertEqual(handle.status, 'succeeded')

    def test_ordinary_goal_status_tracks_observed_motion(self):
        adapter = bridge()
        monitor = SimpleNamespace(crossing=lambda *_: None, corridors=[],
                                  is_alignment=lambda _: False)
        controller = NavController(FrameMap('red'))
        robot = {'x': 1., 'y': 12., 'vx': 0., 'vy': 0., 'team': 'red'}

        class OpenArenaSDK:
            def _prepare_chapter4(self, *_):
                return [], [], lambda *_: True, lambda *_: set()

        adapter.on_goal(pose(2, 3))
        with patch.object(support, 'sdk', OpenArenaSDK()):
            adapter.navigation(controller, monitor, robot, {})
            self.assertGreater(adapter.pub_velocity.messages[-1].linear.x, 0.)
            robot.update(x=2., vx=.4)
            adapter.navigation(controller, monitor, robot, {})
            robot['vx'] = 0.
            adapter.navigation(controller, monitor, robot, {})
        self.assertEqual([msg.data for msg in adapter.pub_nav.messages], [1, 1, 1, 2])
        self.assertEqual([item['xy'] for item in adapter.goal_events], [[2., 3.]])


if __name__ == '__main__':
    unittest.main()
