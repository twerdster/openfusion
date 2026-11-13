#!/usr/bin/env python3
"""Hand-computable trajectory, time association and coordinate regressions."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1]/'tools/evaluate_tum.py'
SPEC = importlib.util.spec_from_file_location('evaluate_tum', SCRIPT)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


class TrajectoryEvaluationTests(unittest.TestCase):
    def test_reflection_conjugation_and_units(self):
        # TUM moves 1/2/3 metres and rotates +90 degrees around camera y.
        # OpenFusion has the opposite camera z axis, so its relative rotation
        # is -90 degrees and its z displacement is -3000 mm.
        first = np.eye(4)
        later = np.array([[0, 0, 1, 1], [0, 1, 0, 2],
                          [-1, 0, 0, 3], [0, 0, 0, 1]], dtype=float)
        expected = np.array([[0, 0, -1, 1000], [0, 1, 0, 2000],
                             [1, 0, 0, -2232], [0, 0, 0, 1]], dtype=float)
        actual = evaluation.initial_anchored_ground_truth(first, later)
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1.0)

    def test_stationary_camera_at_nonidentity_global_pose(self):
        first = np.eye(4)
        first[:3, :3] = evaluation.quaternion_matrix([0, 0, 1, 1])
        first[:3, 3] = [2, -3, 5]
        actual = evaluation.initial_anchored_ground_truth(first, first)
        expected = np.eye(4)
        expected[2, 3] = 768
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_translation_and_quaternion_interpolation(self):
        # Halfway from identity to 180-degree z is 90 degrees z; q and -q
        # must interpolate along the same shortest rotation path.
        rows = np.array([[100, 0, 0, 0, 0, 0, 0, 1],
                         [100.02, 2, 4, 6, 0, 0, 1, 0]], dtype=float)
        pose, evidence = evaluation.associate_ground_truth(rows, 100.01)
        np.testing.assert_allclose(pose[:3, 3], [1, 2, 3], atol=1e-10)
        np.testing.assert_allclose(pose[:3, :3],
                                   [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-10)
        self.assertTrue(evidence['accepted'])
        q = np.array([0, 0, np.sin(0.2), np.cos(0.2)])
        np.testing.assert_allclose(evaluation.slerp(q, -q, 0.3), q, atol=1e-12)

    def test_rejects_timestamp_gap_and_no_extrapolation(self):
        rows = np.array([[100, 0, 0, 0, 0, 0, 0, 1],
                         [101, 1, 0, 0, 0, 0, 0, 1]], dtype=float)
        for timestamp in (99.999, 100.01, 100.5, 101.001):
            pose, evidence = evaluation.associate_ground_truth(rows, timestamp)
            self.assertIsNone(pose)
            self.assertFalse(evidence['accepted'])
        pose, _ = evaluation.associate_ground_truth(rows, 100.01, method='nearest')
        np.testing.assert_array_equal(pose, np.eye(4))

    def test_quaternion_roundtrip_including_180_degree_turns(self):
        for q in ([0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0],
                  [0, 0, 1, 0], [1, -2, 3, 4]):
            r = evaluation.quaternion_matrix(q)
            reconstructed = evaluation.quaternion_matrix(evaluation.matrix_quaternion(r))
            np.testing.assert_allclose(reconstructed, r, atol=1e-12)

    def test_rigid_alignment_fits_no_scale(self):
        x = np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]])
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        y = x @ rotation.T + [30, -20, 50]
        result = evaluation.rigid_position_alignment(x, y)
        self.assertLess(result['translation_error_mm']['rmse'], 1e-10)
        np.testing.assert_allclose(np.array(result['rotation_row_major']).reshape(3, 3), rotation, atol=1e-12)
        scaled = evaluation.rigid_position_alignment(x, 2*x)
        self.assertEqual(scaled['scale_fixed_at'], 1.0)
        self.assertGreater(scaled['translation_error_mm']['rmse'], 5)

    def fixture(self, directory):
        recording = Path(directory)
        # 10 mm TUM +z movements correspond to OpenFusion -z movements.
        (recording/'groundtruth.txt').write_text(
            '# timestamp tx ty tz qx qy qz qw\n'
            '1305031102.175304000 0 0 0 0 0 0 1\n'
            '1305031102.185304000 0 0 0.01 0 0 0 1\n'
            '1305031102.195304000 0 0 0.02 0 0 0 1\n')
        frames, estimates = [], []
        for i, timestamp in enumerate((1305031102.175304, 1305031102.185304, 1305031102.195304)):
            frames.append({'frame': i+1, 'source_index': i+7, 'depth_timestamp': timestamp})
            pose = np.eye(4)
            pose[2, 3] = 768-10*i
            if i == 2:
                pose[0, 3] = 3  # A hand-computable 3 mm final tracking error.
            estimates.append({'index': i, 'estimated_pose_row_major': pose.ravel().tolist()})
        (recording/'manifest.json').write_text(json.dumps({
            'frames': frames, 'ground_truth': {'file': 'groundtruth.txt'}}))
        metrics_path = recording/'metrics.json'
        metrics_path.write_text(json.dumps({'initial_camera_z_mm': 768, 'frames': estimates}))
        return recording, metrics_path

    def test_end_to_end_errors_baseline_and_export(self):
        with tempfile.TemporaryDirectory() as directory:
            recording, metrics_path = self.fixture(directory)
            result, trajectory = evaluation.evaluate(recording, metrics_path)
            errors = result['initial_anchored']['translation_error_mm']
            self.assertAlmostEqual(errors['rmse'], np.sqrt(3), places=9)
            self.assertEqual(errors['final'], 3)
            baseline = result['stationary_baseline']['translation_error_mm']
            self.assertAlmostEqual(baseline['rmse'], np.sqrt(500/3), places=9)
            self.assertAlmostEqual(baseline['final'], 20)
            self.assertAlmostEqual(trajectory[-1][1][2, 3], 0.02)
            self.assertAlmostEqual(trajectory[-1][1][0, 3], 0.003)
            export = recording/'estimated.txt'
            evaluation.write_tum_trajectory(export, trajectory)
            exported = evaluation.load_ground_truth(export)
            # Preserve submillisecond precision in UNIX timestamps, rather
            # than formatting them with a small significant-figure count.
            np.testing.assert_allclose(exported[:, 0], [row[0] for row in trajectory], atol=1e-8, rtol=0)

    def test_rejects_reflection_and_invalid_initialization(self):
        reflected = np.eye(4)
        reflected[2, 2] = -1
        with self.assertRaisesRegex(ValueError, 'proper rotation'):
            evaluation.checked_estimate(reflected.ravel())
        with tempfile.TemporaryDirectory() as directory:
            recording, metrics_path = self.fixture(directory)
            metrics = json.loads(metrics_path.read_text())
            metrics['frames'][0]['estimated_pose_row_major'][3] = 1
            metrics_path.write_text(json.dumps(metrics))
            with self.assertRaisesRegex(ValueError, 'initialization'):
                evaluation.evaluate(recording, metrics_path)

    def test_requires_adequate_associated_pairs(self):
        with tempfile.TemporaryDirectory() as directory:
            recording, metrics_path = self.fixture(directory)
            manifest = json.loads((recording/'manifest.json').read_text())
            manifest['frames'][-1]['depth_timestamp'] += 10
            (recording/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'only 2 ground-truth pairs'):
                evaluation.evaluate(recording, metrics_path)


if __name__ == '__main__':
    unittest.main()
