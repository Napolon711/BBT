#!/usr/bin/env python3
"""Regression tests for the public physical tunnel-run acceptance interface."""
import unittest

from check_tunnel_suite import validate


def completed_tunnel_run():
    """A small 20 Hz physical run: fold outside, cross, unfold, continue."""
    rows = []
    for sample in range(36):
        rows.append({
            'time': sample * .05,
            'position': [-1.5 + sample * .1, 0.],
            'transform_progress': 1. if 1 <= sample < 25 else 0.,
            'speed': 0. if sample in (0, 25, 35) else 2.,
            'upright': 1.,
            'tipped': False,
            'model': 2773,
            'nav_state': 1,
            'bumps_clearance_m': 1.,
            'swept_bumps_clearance_m': 1.,
            'navigation_swept_bumps_clearance_m': 1.,
            'trace': {
                'source': 'fallback',
                'current_region': 2,
                'tunnel': -1,
                'seq': sample,
                'target_point': 4,
            },
        })
    case = {
        'roof': {
            'id': 'test_tunnel_roof',
            'centre': [0., 0.],
            'axis': [1., 0.],
            'lateral': [0., 1.],
            'half_axis': .5,
            'half_width': .5,
        },
        'routing_constraint': 'sentry-no-bumps-v1',
        'navigation_interface': 'nav2-through-poses-cmd-vel-v2',
        'navigation_clearance_radius_m': .47,
        'test_start_s': 0.,
        'entry': [-1., 0.],
        'center': [0., 0.],
        'exit': [1., 0.],
        'staging': [-1.5, 0.],
        'goal_events': [{'time_s': 0., 'frame_id': 'map', 'xy': [-1., 0.]}],
        'route_events': [{
            'time_s': .05,
            'action': 'NavigateThroughPoses',
            'intermediate_behavior': 'pass_through',
            'poses': [[0., 0.], [1., 0.]],
        }],
        'preparation_route': [{
            'reached_s': -.05,
            'actual_pose': [-1.5, 0.],
            'goal': [-1.5, 0.],
        }],
        'vehicle_width_m': .4,
        'exit_region_code': 2,
        'target_id': 4,
        'target_xy': [2., 0.],
        'serial_at_test_start': {'chassis_data': 0, 'fold_cmd': 0},
        'serial_rx': {'chassis_data': 20, 'fold_cmd': 2},
        'trace_count': len(rows),
        'tunnel_name': 'test',
        'direction': 'forward',
    }
    return rows, case


class TunnelAcceptanceTests(unittest.TestCase):
    def test_accepts_continuous_running_navigation_through_center(self):
        rows, case = completed_tunnel_run()

        self.assertTrue(validate(rows, case)['passed'])

    def test_rejects_any_non_running_navigation_at_center(self):
        # Navigation feedback contract: Idle=0, Running=1, Success=2, Failure=3.
        for nav_state in (0, 2, 3):
            with self.subTest(nav_state=nav_state):
                rows, case = completed_tunnel_run()
                for row in rows:
                    if abs(row['position'][0]) < .1:
                        row['nav_state'] = nav_state

                with self.assertRaises(AssertionError):
                    validate(rows, case)


if __name__ == '__main__':
    unittest.main()
