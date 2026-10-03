import hashlib
import math
import pathlib
import sys
import tempfile
import unittest


SCRIPT_DIRECTORY = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from evaluate_localization import (  # noqa: E402
    Pose,
    associate_poses,
    compute_metrics,
    hash_map_directory,
    read_tum,
)


class EvaluateLocalizationTest(unittest.TestCase):
    def test_identical_trajectory_has_zero_error(self):
        poses = [
            Pose(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose(1.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose(2.0, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        metrics = compute_metrics(list(zip(poses, poses)), rpe_delta=1)
        self.assertEqual(metrics.matched_poses, 3)
        self.assertAlmostEqual(metrics.ate_rmse_m, 0.0)
        self.assertAlmostEqual(metrics.rpe_translation_rmse_m, 0.0)
        self.assertAlmostEqual(metrics.rpe_rotation_rmse_deg, 0.0)

    def test_absolute_offset_affects_ate_but_not_rpe(self):
        reference = [
            Pose(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose(1.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose(2.0, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        estimate = [
            Pose(p.stamp, (p.position[0] + 1.0, 0.0, 0.0), p.quaternion)
            for p in reference
        ]
        metrics = compute_metrics(list(zip(estimate, reference)), rpe_delta=1)
        self.assertAlmostEqual(metrics.ate_rmse_m, 1.0)
        self.assertAlmostEqual(metrics.rpe_translation_rmse_m, 0.0)
        self.assertAlmostEqual(metrics.rpe_rotation_rmse_deg, 0.0)

    def test_rpe_reports_relative_rotation(self):
        half = math.sqrt(0.5)
        reference = [
            Pose(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        estimate = [
            reference[0],
            Pose(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, half, half)),
        ]
        metrics = compute_metrics(list(zip(estimate, reference)), rpe_delta=1)
        self.assertAlmostEqual(metrics.rpe_rotation_rmse_deg, 90.0)

    def test_tum_reader_association_and_hashes_are_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            tum = root / "trajectory.tum"
            tum.write_text(
                "# timestamp tx ty tz qx qy qz qw\n"
                "0.01 0 0 0 0 0 0 1\n"
                "1.01 1 0 0 0 0 0 1\n",
                encoding="utf-8",
            )
            estimate = read_tum(tum)
            reference = [
                Pose(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                Pose(1.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ]
            matches = associate_poses(estimate, reference, max_time_diff=0.02)
            self.assertEqual(len(matches), 2)

            map_directory = root / "map"
            map_directory.mkdir()
            (map_directory / "b.txt").write_bytes(b"beta")
            (map_directory / "a.txt").write_bytes(b"alpha")
            report = hash_map_directory(map_directory)
            self.assertEqual([item.relative_path for item in report.files],
                             ["a.txt", "b.txt"])
            self.assertEqual(report.files[0].sha256,
                             hashlib.sha256(b"alpha").hexdigest())
            self.assertEqual(len(report.tree_sha256), 64)


if __name__ == "__main__":
    unittest.main()
