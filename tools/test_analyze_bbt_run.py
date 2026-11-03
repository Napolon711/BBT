"""Regression tests for recorded tunnel center handoff analysis."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_bbt_run import center_handoffs


class CenterHandoffTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            'geometry_reference': 'red',
            'tunnels': [{
                'center': [9.32, 2.15],
                'corners': [],
                'align': [{'p': [8., 2.]}, {'p': [10., 2.3]}],
            }],
        }

    def handoff(self, center):
        return [
            {'time_s': 1., 'kind': 'goal', 'data': {'xy': center}},
            {'time_s': 2., 'kind': 'control',
             'data': {'goal': center, 'nav': 2}},
            {'time_s': 2.3, 'kind': 'goal', 'data': {'xy': [4., 5.]}},
        ]

    def test_red_center_matches_without_rotation(self):
        center = [9.32, 2.15]

        self.assertEqual(center_handoffs(self.handoff(center), self.config, 'red'), [{
            'center': center,
            'arrival_s': 2.,
            'next_goal_s': 2.3,
            'handoff_s': .3,
        }])

    def test_blue_center_matches_rotated_geometry(self):
        center = [18.68, 12.85]

        self.assertEqual(center_handoffs(self.handoff(center), self.config, 'blue'), [{
            'center': center,
            'arrival_s': 2.,
            'next_goal_s': 2.3,
            'handoff_s': .3,
        }])

    def test_configured_blue_reference_keeps_blue_and_rotates_red(self):
        config = dict(self.config, geometry_reference='blue')
        blue_center = [9.32, 2.15]
        red_center = [18.68, 12.85]

        self.assertEqual(len(center_handoffs(self.handoff(blue_center), config, 'blue')), 1)
        self.assertEqual(len(center_handoffs(self.handoff(red_center), config, 'red')), 1)


if __name__ == '__main__':
    unittest.main()
