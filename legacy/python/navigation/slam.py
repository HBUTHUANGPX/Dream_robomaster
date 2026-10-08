"""Lightweight gravity-aligned ICP localization and online point mapping.

This is not FAST-LIO or full loop-closure SLAM. It has no inertial state,
roll/pitch estimation, global relocalization, or access to simulator truth.
ICP is local: it needs overlapping, informative geometry and a nearby odometry
prediction. Repeated geometry can still cause incorrect associations.
"""

import numpy as np
from scipy.spatial import cKDTree


_VOXEL = 0.10
_MIN_POINTS = 30
_MAX_MAP = 40_000


def _points(value):
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    return points[np.isfinite(points).all(axis=1)].copy()


def _vector(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (4,) or not np.isfinite(vector).all():
        raise ValueError('pose and odometry must be finite arrays of shape (4,)')
    return vector.copy()


def _voxelize(points):
    if not len(points):
        return points.copy()
    # Keep an actual measured point in each voxel rather than inventing surfaces.
    keys = np.floor(points / _VOXEL)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def _rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _world(points, pose):
    return points @ _rotation(pose[3]).T + pose[:3]


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LidarMapper:
    """Estimate [x, y, z, yaw] from leveled, body-relative metric scans.

    Scan coordinates have roll/pitch removed but NOT yaw. Odometry is
    [dx_body, dy_body, dz, d_yaw]; translation uses the previous yaw.
    ``prior_points`` are world coordinates and are never exposed through
    ``map_points``, which contains only transformed, accumulated scans.

    Invalid point rows are discarded; malformed shapes/nonfinite odometry
    raise ValueError before changing state. Sparse scans only advance odometry.
    Rejected registration preserves odometry and does not contaminate an
    existing map. The first usable scan bootstraps mapping without a prior.
    ``rmse`` and ``matched`` describe accepted, trimmed correspondences in
    metres; rejection/no registration sets them to infinity and zero.
    """

    def __init__(self, initial_pose, prior_points=None, prior_normals=None):
        self.pose = _vector(initial_pose)
        self.map_points = np.empty((0, 3), dtype=float)
        self.rmse = float('inf')
        self.matched = 0
        self._prior = (_voxelize(_points(prior_points)) if prior_points is not None
                       else np.empty((0, 3)))
        self._tree = cKDTree(self._prior) if len(self._prior) else None
        self._target = self._prior
        self._dirty = False
        self._normals = None
        if prior_normals is not None:
            raw = np.asarray(prior_points, dtype=float)
            normals = np.asarray(prior_normals, dtype=float)
            if (normals.shape != raw.shape or not np.isfinite(raw).all() or
                    not np.isfinite(normals).all() or
                    np.any(np.linalg.norm(normals, axis=1) < 1e-8)):
                raise ValueError('prior normals must be finite nonzero vectors aligned with points')
            _, indices = np.unique(np.floor(raw / _VOXEL), axis=0, return_index=True)
            self._normals = normals[np.sort(indices)]
            self._normals /= np.linalg.norm(self._normals, axis=1)[:, None]

    def _register_planes(self, source, predicted):
        # CAD surface normals constrain translation without letting dense ground
        # samples manufacture XY motion when wheel odometry slips.
        candidate = predicted.copy()
        for _ in range(20):
            world = _world(source, candidate)
            distance, index = self._tree.query(world, distance_upper_bound=.6)
            valid = np.flatnonzero(np.isfinite(distance))
            if len(valid) < max(_MIN_POINTS, .3 * len(source)):
                return None
            normals = self._normals[index[valid]]
            residual = np.sum((world[valid] - self._target[index[valid]]) * normals, axis=1)
            # Robust plane residuals, not global nearest-point trimming: walls
            # retain their horizontal information even if floor samples dominate.
            weights = np.minimum(1., .06 / np.maximum(abs(residual), 1e-8))
            relative = world[valid] - candidate[:3]
            yaw_column = -relative[:, 1] * normals[:, 0] + relative[:, 0] * normals[:, 1]
            jacobian = np.c_[normals, yaw_column]
            root = np.sqrt(weights)
            delta, _, rank, _ = np.linalg.lstsq(jacobian * root[:, None], -residual * root, rcond=1e-4)
            if rank < 4:
                return None
            candidate += delta
            candidate[3] = _wrap(candidate[3])
            if (np.linalg.norm(candidate[:3] - predicted[:3]) > .5 or
                    abs(_wrap(candidate[3] - predicted[3])) > .20):
                return None
            if np.linalg.norm(delta) < 1e-4:
                break
        error = float(np.sqrt(np.average(residual ** 2, weights=weights)))
        if error > .12:
            return None
        return candidate, error, len(valid)

    def _correspondences(self, world):
        distances, indices = self._tree.query(world, distance_upper_bound=0.6)
        valid = np.flatnonzero(np.isfinite(distances))
        if len(valid) < max(_MIN_POINTS, int(0.3 * len(world))):
            return None
        # Remove the worst 20% within the metric gate, including boundary clutter.
        valid = valid[np.argsort(distances[valid])[:int(0.8 * len(valid))]]
        if len(valid) < _MIN_POINTS:
            return None
        if len(np.unique(indices[valid])) < _MIN_POINTS:
            return None
        return world[valid], self._target[indices[valid]], distances[valid]

    def _register(self, source, predicted):
        if self._normals is not None:
            return self._register_planes(source, predicted)
        candidate = predicted.copy()
        for _ in range(20):
            pairs = self._correspondences(_world(source, candidate))
            if pairs is None:
                return None
            src, dst, _ = pairs
            src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
            a, b = src - src_mean, dst - dst_mean
            # Reject effectively point-like/collinear geometry. Planes are OK.
            if np.linalg.svd(a, compute_uv=False)[1] / np.sqrt(len(a)) < 0.08:
                return None
            cross = np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
            dot = np.sum(a[:, :2] * b[:, :2])
            yaw = np.arctan2(cross, dot)
            rotation = _rotation(yaw)
            translation = dst_mean - rotation @ src_mean
            revised = candidate.copy()
            revised[:3] = rotation @ candidate[:3] + translation
            revised[3] = _wrap(candidate[3] + yaw)
            # Reject the entire correction, rather than accepting a clipped fit.
            if (np.linalg.norm(revised[:3] - predicted[:3]) > 0.5 or
                    abs(_wrap(revised[3] - predicted[3])) > 0.20):
                return None
            change = np.linalg.norm(revised[:3] - candidate[:3])
            candidate = revised
            if change < 1e-4 and abs(yaw) < 1e-4:
                break
        pairs = self._correspondences(_world(source, candidate))
        if pairs is None:
            return None
        error = float(np.sqrt(np.mean(pairs[2] ** 2)))
        if error > 0.20:
            return None
        return candidate, error, len(pairs[0])

    def update(self, points, odometry):
        """Advance odometry, attempt bounded ICP, and return a copy of pose."""
        source = _voxelize(_points(points))
        odometry = _vector(odometry)
        predicted = self.pose.copy()
        predicted[:3] += _rotation(self.pose[3]) @ odometry[:3]
        predicted[3] = _wrap(predicted[3] + odometry[3])
        self.pose = predicted
        self.rmse, self.matched = float('inf'), 0
        if len(source) < _MIN_POINTS:
            return self.pose.copy()
        if self._dirty:
            self._target = self._prior if len(self._prior) else self.map_points
            self._tree = cKDTree(self._target)
            self._dirty = False
        if self._tree is not None:
            result = self._register(source, predicted)
            if result is None:
                return self.pose.copy()
            self.pose, self.rmse, self.matched = result
        self.map_points = _voxelize(np.vstack((_world(source, self.pose), self.map_points)))
        # New measurements take precedence, retaining a bounded recent map.
        self.map_points = self.map_points[:_MAX_MAP]
        self._dirty = not len(self._prior)
        return self.pose.copy()
