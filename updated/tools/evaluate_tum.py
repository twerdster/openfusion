#!/usr/bin/env python3
"""Evaluate saved OpenFusion camera poses against independent TUM motion capture.

This is an offline evaluator: ground truth is never input to ICP. Translation
and quaternion interpolation associate motion-capture samples with depth times.
The motion-capture pose associated with the first depth timestamp fixes the
coordinate origin; no trajectory fit is used for the primary errors. A separate
SE(3) position alignment estimates no scale.

Requires Python 3 and NumPy. Input groundtruth.txt uses TUM's timestamp,
tx ty tz qx qy qz qw format (camera-to-world poses, metres).
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


C = np.diag([1.0, 1.0, -1.0, 1.0])


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quaternion_matrix(q):
    """TUM xyzw quaternion to a proper rotation matrix."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


def matrix_quaternion(rotation):
    """Stable proper-rotation to xyzw conversion, including 180-degree turns."""
    r = np.asarray(rotation, dtype=float)
    # Largest-component construction avoids dividing by a small quaternion term.
    candidates = np.array([1+r[0, 0]-r[1, 1]-r[2, 2],
                           1-r[0, 0]+r[1, 1]-r[2, 2],
                           1-r[0, 0]-r[1, 1]+r[2, 2],
                           1+np.trace(r)])
    k = int(np.argmax(candidates))
    v = 0.5 * np.sqrt(max(0.0, candidates[k]))
    if v < 1e-12:
        raise ValueError('matrix is not a proper rotation')
    if k == 0:
        q = [v, (r[0, 1]+r[1, 0])/(4*v),
             (r[0, 2]+r[2, 0])/(4*v), (r[2, 1]-r[1, 2])/(4*v)]
    elif k == 1:
        q = [(r[0, 1]+r[1, 0])/(4*v), v,
             (r[1, 2]+r[2, 1])/(4*v), (r[0, 2]-r[2, 0])/(4*v)]
    elif k == 2:
        q = [(r[0, 2]+r[2, 0])/(4*v), (r[1, 2]+r[2, 1])/(4*v),
             v, (r[1, 0]-r[0, 1])/(4*v)]
    else:
        q = [(r[2, 1]-r[1, 2])/(4*v), (r[0, 2]-r[2, 0])/(4*v),
             (r[1, 0]-r[0, 1])/(4*v), v]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def slerp(q0, q1, alpha):
    q0 = np.asarray(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    cosine = float(q0 @ q1)
    if cosine < 0:
        q1 = -q1
        cosine = -cosine
    cosine = np.clip(cosine, -1.0, 1.0)
    if cosine > 0.9995:
        result = q0 + alpha * (q1-q0)
        return result / np.linalg.norm(result)
    angle = np.arccos(cosine)
    return (np.sin((1-alpha)*angle)*q0 + np.sin(alpha*angle)*q1) / np.sin(angle)


def pose_from_row(row):
    pose = np.eye(4)
    pose[:3, :3] = quaternion_matrix(row[4:8])
    pose[:3, 3] = row[1:4]
    return pose


def load_ground_truth(path):
    rows = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        try:
            row = np.array([float(x) for x in line.split()])
        except ValueError as error:
            raise ValueError(f'{path.name}:{line_number}: invalid number') from error
        if row.shape != (8,) or not np.isfinite(row).all():
            raise ValueError(f'{path.name}:{line_number}: expected eight finite values')
        quaternion_matrix(row[4:8])
        rows.append(row)
    if not rows:
        raise ValueError('ground truth has no poses')
    rows = np.array(rows)
    if len(rows) > 1 and np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('ground-truth timestamps must be strictly increasing')
    return rows


def associate_ground_truth(rows, timestamp, max_delta=0.02, method='interpolate'):
    """Return associated pose and explicit timing evidence, or a rejection."""
    times = rows[:, 0]
    index = int(np.searchsorted(times, timestamp))
    adjacent = [j for j in (index-1, index) if 0 <= j < len(times)]
    nearest = min(adjacent, key=lambda j: abs(times[j]-timestamp))
    result = {'depth_timestamp': float(timestamp),
              'nearest_ground_truth_timestamp': float(times[nearest]),
              'nearest_delta_seconds': float(abs(times[nearest]-timestamp)),
              'method': method}
    if method == 'nearest' or times[nearest] == timestamp:
        if abs(times[nearest]-timestamp) > max_delta:
            return None, dict(result, accepted=False, reason='nearest timestamp exceeds tolerance')
        return pose_from_row(rows[nearest]), dict(result, accepted=True,
            lower_timestamp=float(times[nearest]), upper_timestamp=float(times[nearest]),
            lower_delta_seconds=float(abs(times[nearest]-timestamp)),
            upper_delta_seconds=float(abs(times[nearest]-timestamp)), interpolation_fraction=0.0)
    if index == 0 or index == len(times):
        return None, dict(result, accepted=False, reason='outside ground-truth interval; no extrapolation')
    lo, hi = rows[index-1], rows[index]
    low_delta, high_delta = float(timestamp-lo[0]), float(hi[0]-timestamp)
    result.update(lower_timestamp=float(lo[0]), upper_timestamp=float(hi[0]),
                  lower_delta_seconds=low_delta, upper_delta_seconds=high_delta)
    if max(low_delta, high_delta) > max_delta:
        return None, dict(result, accepted=False, reason='interpolation endpoint exceeds tolerance')
    fraction = low_delta / (hi[0]-lo[0])
    pose = np.eye(4)
    pose[:3, 3] = lo[1:4]*(1-fraction) + hi[1:4]*fraction
    pose[:3, :3] = quaternion_matrix(slerp(lo[4:8], hi[4:8], fraction))
    return pose, dict(result, accepted=True, interpolation_fraction=float(fraction))


def initial_anchored_ground_truth(first_tum_pose, tum_pose, initial_z_mm=768.0):
    """Tinit C inv(TUM0) TUMi C; camera xyz basis changes on both sides."""
    relative = np.linalg.inv(first_tum_pose) @ tum_pose
    relative[:3, 3] *= 1000.0
    initial = np.eye(4)
    initial[2, 3] = initial_z_mm
    return initial @ C @ relative @ C


def project_rotation(rotation):
    u, _, vt = np.linalg.svd(rotation)
    correction = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    return u @ correction @ vt


def rotation_error_degrees(estimated, expected):
    # Saved float matrices can be slightly nonorthogonal. Only the angle uses
    # this nearest proper rotation; translation and the stored poses are intact.
    relative = expected[:3, :3].T @ project_rotation(estimated[:3, :3])
    return float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))


def error_summary(values):
    a = np.asarray(values, dtype=float)
    return {'rmse': float(np.sqrt(np.mean(a*a))), 'mean': float(np.mean(a)),
            'median': float(np.median(a)), 'maximum': float(np.max(a)),
            'final': float(a[-1]), 'samples': len(a)}


def rigid_position_alignment(estimated, expected):
    """Least-squares proper rotation/translation in mm. Scale is fixed at one."""
    x = np.asarray(estimated)
    y = np.asarray(expected)
    x_center, y_center = x.mean(axis=0), y.mean(axis=0)
    if len(x) < 3 or np.linalg.norm(x-x_center) < 1e-9:
        return {'available': False, 'reason': 'requires three pairs and nonstationary estimates'}
    u, singular_values, vt = np.linalg.svd((x-x_center).T @ (y-y_center))
    rotation = vt.T @ np.diag([1.0, 1.0, np.linalg.det(vt.T @ u.T)]) @ u.T
    translation = y_center - rotation @ x_center
    errors = np.linalg.norm(x @ rotation.T + translation-y, axis=1)
    return {'available': True, 'scope': 'position-only ATE after fitted SE(3), separate from initial-anchored errors',
            'scale_fixed_at': 1.0, 'translation_units': 'mm',
            'translation_error_mm': error_summary(errors),
            'rotation_row_major': rotation.ravel().tolist(),
            'translation_mm': translation.tolist(),
            'cross_covariance_singular_values_mm_squared': singular_values.tolist(),
            'cross_covariance_rank': int(np.linalg.matrix_rank((x-x_center).T @ (y-y_center))),
            'qualification': 'Low-rank motion does not constrain the full fitted rotation.'}


def checked_estimate(values):
    pose = np.asarray(values, dtype=float)
    if pose.shape != (16,) or not np.isfinite(pose).all():
        raise ValueError('estimated pose must have 16 finite row-major values')
    pose = pose.reshape(4, 4)
    if not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6, rtol=0):
        raise ValueError('estimated pose has invalid homogeneous last row')
    rotation = pose[:3, :3]
    orthogonality = float(np.linalg.norm(rotation.T @ rotation-np.eye(3)))
    if orthogonality > 0.001 or np.linalg.det(rotation) <= 0:
        raise ValueError('estimated pose rotation is not close to a proper rotation')
    return pose, orthogonality


def evaluate(recording, metrics_path, manifest_path=None, max_delta=0.02,
             method='interpolate', min_pairs=3):
    recording, metrics_path = Path(recording), Path(metrics_path)
    manifest_path = Path(manifest_path) if manifest_path else recording/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    metrics = json.loads(metrics_path.read_text())
    ground_truth_path = recording/manifest['ground_truth']['file']
    expected_hash = manifest['ground_truth'].get('sha256')
    if expected_hash and sha256(ground_truth_path) != expected_hash:
        raise ValueError('ground-truth hash does not match recording manifest')
    if not np.isfinite(max_delta) or max_delta <= 0:
        raise ValueError('max timestamp delta must be positive and finite')
    if method not in ('interpolate', 'nearest') or min_pairs < 2:
        raise ValueError('invalid association method or minimum pair count')
    rows = load_ground_truth(ground_truth_path)
    frames = {frame['frame']: frame for frame in manifest['frames']}
    if len(frames) != len(manifest['frames']):
        raise ValueError('manifest has duplicate frame numbers')
    estimates = metrics.get('frames', [])
    if not estimates or estimates[0]['index'] != 0:
        raise ValueError('metrics must start at recording frame zero')
    indices = [frame['index'] for frame in estimates]
    if indices != list(range(len(estimates))):
        raise ValueError('metrics frame indices must be consecutive from zero')
    timestamps = [float(frames[index+1]['depth_timestamp']) for index in indices]
    if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError('depth timestamps must be finite and strictly increasing')
    first_tum, anchor_timing = associate_ground_truth(rows, timestamps[0], max_delta, method)
    if first_tum is None:
        raise ValueError(f'first frame has no ground-truth anchor: {anchor_timing}')
    initial_z = float(metrics.get('initial_camera_z_mm', 768.0))
    initial_pose = np.eye(4)
    initial_pose[2, 3] = initial_z
    first_estimate, _ = checked_estimate(estimates[0]['estimated_pose_row_major'])
    if not np.allclose(first_estimate, initial_pose, atol=0.001, rtol=0):
        raise ValueError('first estimate differs from declared historical initialization')
    pairs, rejected, tum_export = [], [], []
    estimated_positions, expected_positions = [], []
    translation_errors, rotation_errors, stationary_translations, stationary_rotations = [], [], [], []
    for frame, timestamp in zip(estimates, timestamps):
        estimate, orthogonality = checked_estimate(frame['estimated_pose_row_major'])
        truth, timing = associate_ground_truth(rows, timestamp, max_delta, method)
        info = {'index': frame['index'], 'recorded_frame': frame['index']+1,
                'source_index': frames[frame['index']+1].get('source_index'), 'association': timing}
        # Export all estimates, even ones rejected by timestamp association.
        estimate_metres, initial_metres = estimate.copy(), initial_pose.copy()
        estimate_metres[:3, 3] /= 1000.0
        initial_metres[:3, 3] /= 1000.0
        tum_estimate = first_tum @ C @ np.linalg.inv(initial_metres) @ estimate_metres @ C
        tum_export.append((timestamp, tum_estimate))
        if truth is None:
            rejected.append(info)
            continue
        expected = initial_anchored_ground_truth(first_tum, truth, initial_z)
        te = float(np.linalg.norm(estimate[:3, 3]-expected[:3, 3]))
        re = rotation_error_degrees(estimate, expected)
        baseline_te = float(np.linalg.norm(initial_pose[:3, 3]-expected[:3, 3]))
        baseline_re = rotation_error_degrees(initial_pose, expected)
        info.update(translation_error_mm=te, rotation_error_degrees=re,
                    stationary_translation_error_mm=baseline_te,
                    stationary_rotation_error_degrees=baseline_re,
                    estimated_rotation_orthogonality_error=orthogonality,
                    expected_pose_row_major_mm=expected.ravel().tolist())
        pairs.append(info)
        translation_errors.append(te)
        rotation_errors.append(re)
        stationary_translations.append(baseline_te)
        stationary_rotations.append(baseline_re)
        estimated_positions.append(estimate[:3, 3])
        expected_positions.append(expected[:3, 3])
    if len(pairs) < min_pairs:
        raise ValueError(f'only {len(pairs)} ground-truth pairs, require {min_pairs}')
    expected_positions = np.asarray(expected_positions)
    motion = {'ground_truth_path_length_mm': float(np.linalg.norm(np.diff(expected_positions, axis=0), axis=1).sum()),
              'ground_truth_final_displacement_mm': stationary_translations[-1],
              'ground_truth_maximum_displacement_mm': max(stationary_translations),
              'ground_truth_maximum_rotation_degrees': max(stationary_rotations),
              'estimated_final_displacement_mm': float(np.linalg.norm(np.asarray(estimated_positions[-1])-initial_pose[:3, 3])),
              'span_seconds': pairs[-1]['association']['depth_timestamp']-pairs[0]['association']['depth_timestamp']}
    result = {
        'schema_version': 1, 'status': 'evaluated',
        'scope': 'camera trajectory versus independent motion capture; no surface-geometry ground truth',
        'accuracy_pass_threshold': None,
        'sources': {'manifest_file': manifest_path.name, 'manifest_sha256': sha256(manifest_path),
                    'metrics_file': metrics_path.name, 'metrics_sha256': sha256(metrics_path),
                    'ground_truth_file': ground_truth_path.name, 'ground_truth_sha256': sha256(ground_truth_path)},
        'association': {'method': method, 'maximum_timestamp_delta_seconds': max_delta,
                        'policy': ('both interpolation endpoints must be within tolerance; no extrapolation'
                                   if method == 'interpolate' else 'nearest pose must be within tolerance'),
                        'estimated_frames': len(estimates), 'matched_frames': len(pairs),
                        'rejected_frames': len(rejected), 'minimum_required_pairs': min_pairs,
                        'maximum_accepted_nearest_delta_seconds': max(p['association']['nearest_delta_seconds'] for p in pairs)},
        'coordinates': {'tum_pose': 'camera-to-world, xyz: right/down/forward, translation metres',
                        'openfusion_pose': 'camera-to-world, xyz: right/down/backward, translation millimetres',
                        'camera_basis_change_row_major': C.ravel().tolist(),
                        'expected_pose_formula': 'Tinit * C * inverse(TUM0) * TUMi * C (translation converted to mm)',
                        'initial_camera_z_mm': initial_z,
                        'angle_calculation': 'nearest proper rotation of saved float estimate; input orthogonality error <=0.001'},
        'initial_anchored': {'fit': 'none; first ground-truth pose defines origin and orientation',
                             'translation_error_mm': error_summary(translation_errors),
                             'rotation_error_degrees': error_summary(rotation_errors)},
        'stationary_baseline': {'definition': 'hold the historical initialized camera pose for every frame',
                                'translation_error_mm': error_summary(stationary_translations),
                                'rotation_error_degrees': error_summary(stationary_rotations)},
        'motion': motion,
        'motion_exceeds_1mm_or_0_1degree': motion['ground_truth_maximum_displacement_mm'] > 1 or motion['ground_truth_maximum_rotation_degrees'] > 0.1,
        'se3_aligned_position_ate': rigid_position_alignment(estimated_positions, expected_positions),
        'frames': pairs, 'rejected': rejected,
    }
    return result, tum_export


def write_tum_trajectory(path, trajectory):
    with Path(path).open('w') as stream:
        stream.write('# Estimated camera-to-world trajectory in original TUM global coordinates.\n')
        stream.write('# First mocap pose anchors coordinates only; no all-frame fit or scale fit.\n')
        stream.write('# timestamp tx ty tz qx qy qz qw; translations in metres.\n')
        for timestamp, pose in trajectory:
            quaternion = matrix_quaternion(project_rotation(pose[:3, :3]))
            stream.write(f'{timestamp:.9f} '+' '.join(
                f'{v:.12g}' for v in [*pose[:3, 3], *quaternion])+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--metrics', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--estimated-trajectory', type=Path)
    parser.add_argument('--max-delta-seconds', type=float, default=0.02)
    parser.add_argument('--association', choices=['interpolate', 'nearest'], default='interpolate')
    parser.add_argument('--min-pairs', type=int, default=3)
    args = parser.parse_args()
    result, trajectory = evaluate(args.recording, args.metrics, args.manifest,
                                  args.max_delta_seconds, args.association, args.min_pairs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path = args.estimated_trajectory or args.output.with_suffix('.estimated.txt')
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    write_tum_trajectory(trajectory_path, trajectory)
    result['estimated_tum_trajectory_file'] = trajectory_path.name
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ('status', 'association', 'initial_anchored', 'stationary_baseline', 'motion')}, indent=2))


if __name__ == '__main__':
    main()
