import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tools import align_map


class AlignMapTests(unittest.TestCase):
    def test_blue_team_field_rotation_matches_operator_rule(self):
        self.assertEqual(align_map.team_field_point((7.69, 3.92), "blue"),
                         (20.31, 11.08))
        self.assertEqual(align_map.team_field_point((7.69, 3.92), "red"),
                         (7.69, 3.92))

    def test_solve_two_tunnel_centers_for_blue_map(self):
        field = [align_map.team_field_point((2.0, 1.0), "blue"),
                 align_map.team_field_point((20.0, 10.0), "blue")]
        yaw = math.radians(37.0)
        c, s = math.cos(yaw), math.sin(yaw)
        pairs = [(p, (c * p[0] - s * p[1] + 3.25,
                      s * p[0] + c * p[1] - 4.75)) for p in field]
        theta, scale, dx, dy = align_map.solve(pairs)
        self.assertAlmostEqual(math.degrees(theta), 37.0)
        self.assertAlmostEqual(scale, 1.0)
        self.assertAlmostEqual(dx, 3.25)
        self.assertAlmostEqual(dy, -4.75)
        self.assertLess(max(align_map.residuals(pairs, theta, scale, dx, dy)), 1e-9)

    def test_single_origin_requires_known_yaw_and_solves_translation(self):
        pairs = [((20.31, 11.08), (2.0, -3.0))]
        with self.assertRaises(ValueError):
            align_map.solve(pairs)
        theta, scale, dx, dy = align_map.solve(pairs, fixed_yaw_deg=90.0)
        self.assertAlmostEqual(math.degrees(theta), 90.0)
        self.assertEqual(scale, 1.0)
        self.assertAlmostEqual(dx, 13.08)
        self.assertAlmostEqual(dy, -23.31)

    def test_blue_write_preserves_red_transform(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            path.write_text("field_to_map:\n  enabled: true\n  dx: 1\n  dy: 2\n"
                            "  dyaw_deg: 3\nsafety:\n  spin_w: 1\n",
                            encoding="utf-8")
            align_map.write_yaml(path, 4, 5, 6, "blue_field_to_map")
            import yaml
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(config["field_to_map"],
                             {"enabled": True, "dx": 1, "dy": 2, "dyaw_deg": 3})
            self.assertEqual(config["blue_field_to_map"],
                             {"enabled": True, "dx": 4, "dy": 5, "dyaw_deg": 6})
            self.assertEqual(config["safety"]["spin_w"], 1)

    def test_blue_write_supports_inline_legacy_transform(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            path.write_text("field_to_map: {enabled: true, dx: 1, dy: 2, dyaw_deg: 3}\n",
                            encoding="utf-8")
            align_map.write_yaml(path, 4, 5, 6, "blue_field_to_map")
            import yaml
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(config["field_to_map"]["dx"], 1)
            self.assertEqual(config["blue_field_to_map"]["dyaw_deg"], 6)

    def test_recalibration_without_final_newline_preserves_other_team(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            path.write_text("blue_field_to_map: {dx: 10, dy: 20, dyaw_deg: 30}\n"
                            "field_to_map:\n  enabled: true\n  dx: 1\n  dy: 2\n"
                            "  dyaw_deg: 3", encoding="utf-8")
            align_map.write_yaml(path, 4, 5, 6)
            import yaml
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(config["field_to_map"]["dyaw_deg"], 6)
            self.assertEqual(config["blue_field_to_map"],
                             {"dx": 10, "dy": 20, "dyaw_deg": 30})

    def test_cli_single_anchor_calibrates_blue_and_preserves_red(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            path.write_text("geometry_reference: red\n"
                            "field_to_map: {enabled: true, dx: 1, dy: 2, dyaw_deg: 3}\n",
                            encoding="utf-8")
            pairs = Path(temp) / "pairs.csv"
            pairs.write_text("id,field_x,field_y,map_x,map_y\n"
                             "origin,7.69,3.92,2,-3\n", encoding="utf-8")
            result = subprocess.run([sys.executable, str(Path(align_map.__file__)),
                                     str(pairs), "--config", str(path), "--team", "blue",
                                     "--dyaw-deg", "90", "--write"],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            import yaml
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(config["field_to_map"],
                             {"enabled": True, "dx": 1, "dy": 2, "dyaw_deg": 3})
            self.assertEqual(config["blue_field_to_map"],
                             {"enabled": True, "dx": 13.08, "dy": -23.31, "dyaw_deg": 90})

    def test_blue_write_stays_inside_yaml_document(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            path.write_text("field_to_map: {dx: 1}\n...\n", encoding="utf-8")
            align_map.write_yaml(path, 4, 5, 6, "blue_field_to_map")
            import yaml
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(config["blue_field_to_map"]["dx"], 4)

    def test_alias_transform_rejected_without_modifying_config(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "decision.yaml"
            original = "shared: &shared {dx: 1}\nfield_to_map: *shared\n"
            path.write_text(original, encoding="utf-8")
            with self.assertRaises(SystemExit):
                align_map.write_yaml(path, 4, 5, 6)
            self.assertEqual(path.read_text(encoding="utf-8"), original)


if __name__ == "__main__":
    unittest.main()
