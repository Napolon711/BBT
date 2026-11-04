"""Feature-demo acceptance uses raw operator coordinates and real arrival."""
import copy
import unittest

from tools.run_bbt_demo import demo_checks


class DemoAcceptanceTests(unittest.TestCase):
    def test_operator_target_needs_raw_coordinates_and_navigation_success(self):
        result = dict(other_commanded_move_ticks=0, shots={}, controller=dict(
            fold_commands=[], fold_states=[], purchases={'hp': 0, 'ammo': 0}),
            event_counts={}, operator_command=dict(time_s=260., xy=[7.69, 3.92],
                arrival_tolerance_m=.08))
        row = dict(time_s=280., position=[7.69, 3.92], speed_mps=0., nav_state=2,
                   trace=dict(target_point=-3, target_xy=[7.69, 3.92],
                              operator_target=True), outposts={
                side+'_outpost': {'controller': 'red', 'control': 'full'}
                for side in ('red', 'blue')})
        cfg = {'points': []}
        checks = demo_checks([row], [], result, cfg, {'passed': True})
        self.assertTrue(checks['operator_coordinate_requested'])
        self.assertTrue(checks['operator_coordinate_arrived'])

        still_running = copy.deepcopy(row)
        still_running['nav_state'] = 1
        checks = demo_checks([still_running], [], result, cfg, {'passed': True})
        self.assertFalse(checks['operator_coordinate_arrived'])

        snapped = copy.deepcopy(row)
        snapped['trace']['target_point'] = 34
        snapped['trace']['target_xy'] = [7.57, 4.]
        checks = demo_checks([snapped], [], result, cfg, {'passed': True})
        self.assertFalse(checks['operator_coordinate_requested'])
        self.assertFalse(checks['operator_coordinate_arrived'])


if __name__ == '__main__':
    unittest.main()
