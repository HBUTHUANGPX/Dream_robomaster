"""Independent synthetic geometry; no simulator or model inputs."""
import unittest

import numpy as np

from navigation.slam import LidarMapper


def scene(n=2000):
    rng = np.random.default_rng(17)
    points = rng.uniform([-4, -3, -1], [5, 4, 3], (n, 3))
    points[:n // 3, 0] = -3.7
    points[n // 3:2 * n // 3, 1] = 3.2
    points[2 * n // 3:, 2] = -0.8
    return points


def body_points(world, pose):
    c, s = np.cos(pose[3]), np.sin(pose[3])
    return (world - pose[:3]) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class LidarMapperTests(unittest.TestCase):
    def test_recovers_transform_with_partial_overlap_and_outliers(self):
        prior = scene()
        actual = np.array([0.5, -0.3, 0.4, 0.3])
        mapper = LidarMapper(actual + [0.18, -0.14, 0.1, 0.045], prior)
        scan = body_points(prior[:1700], actual)
        scan = np.vstack((scan, np.random.default_rng(2).uniform(15, 25, (200, 3))))
        estimate = mapper.update(scan, np.zeros(4))
        np.testing.assert_allclose(estimate, actual, atol=0.025)
        self.assertLess(mapper.rmse, 0.04)
        self.assertGreater(mapper.matched, 1000)

    def test_prior_is_not_online_map(self):
        mapper = LidarMapper(np.zeros(4), scene())
        self.assertEqual(mapper.map_points.shape, (0, 3))
        mapper.update(np.empty((0, 3)), np.zeros(4))
        self.assertEqual(mapper.map_points.shape, (0, 3))

    def test_online_map_localizes_without_prior(self):
        points = scene()
        mapper = LidarMapper(np.zeros(4))
        mapper.update(points, np.zeros(4))
        actual = np.array([0.15, 0.1, 0.03, 0.025])
        result = mapper.update(body_points(points, actual), actual + [0.1, -0.08, 0.06, 0.02])
        np.testing.assert_allclose(result, actual, atol=0.025)
        self.assertGreater(len(mapper.map_points), 1000)

    def test_invalid_sparse_and_unmatched_scans_preserve_odometry(self):
        for scan in (np.empty((0, 3)), np.ones((5, 3)),
                     np.full((50, 3), np.nan), scene() + 100,
                     np.ones((100, 3))):
            with self.subTest(shape=scan.shape):
                mapper = LidarMapper(np.array([1, 2, 3, np.pi / 2]), scene())
                result = mapper.update(scan, np.array([0.3, 0.1, 0.2, 0.1]))
                np.testing.assert_allclose(result, [0.9, 2.3, 3.2, np.pi / 2 + 0.1])
                self.assertEqual(mapper.matched, 0)
                self.assertTrue(np.isinf(mapper.rmse))

    def test_nonfinite_rows_filtered_and_inputs_not_mutated(self):
        points = scene()
        original = points.copy()
        mapper = LidarMapper(np.zeros(4), points)
        mapper.update(np.vstack((points, [np.nan, 0, 0], [0, np.inf, 0])), np.zeros(4))
        self.assertTrue(np.isfinite(mapper.map_points).all())
        np.testing.assert_array_equal(points, original)

    def test_invalid_shapes_and_odometry_rejected_without_state_change(self):
        mapper = LidarMapper(np.zeros(4))
        for scan, odom in ((np.zeros((3, 2)), np.zeros(4)),
                           (scene(), [0, 0, np.nan, 0])):
            with self.assertRaises(ValueError):
                mapper.update(scan, odom)
            np.testing.assert_array_equal(mapper.pose, np.zeros(4))

    def test_excessive_correction_is_rejected(self):
        # A narrow XY extent makes a 0.3 rad yaw error match within the
        # distance gate, but it exceeds the per-update correction budget.
        points = scene() * [0.25, 0.25, 1]
        mapper = LidarMapper(np.zeros(4), points)
        scan = body_points(points, [0, 0, 0, 0.3])
        mapper.update(scan, np.zeros(4))
        np.testing.assert_array_equal(mapper.pose, np.zeros(4))
        self.assertEqual(mapper.matched, 0)
        self.assertEqual(len(mapper.map_points), 0)

    def test_map_cap_and_voxel_uniqueness(self):
        points = np.random.default_rng(31).uniform(-30, 30, (45000, 3))
        mapper = LidarMapper(np.zeros(4))
        mapper.update(points, np.zeros(4))
        self.assertEqual(len(mapper.map_points), 40000)
        keys = np.floor(mapper.map_points / 0.1)
        self.assertEqual(len(np.unique(keys, axis=0)), len(keys))

    def test_rejected_registration_does_not_pollute_existing_map(self):
        mapper = LidarMapper(np.zeros(4))
        mapper.update(scene(), np.zeros(4))
        before = mapper.map_points.copy()
        mapper.update(scene() + 100, np.zeros(4))
        np.testing.assert_array_equal(mapper.map_points, before)


if __name__ == '__main__':
    unittest.main()

class PlanePriorTests(unittest.TestCase):
    def test_floor_does_not_hide_slip_translation(self):
        rng = np.random.default_rng(14)
        floor = rng.uniform([-4, -4, 0], [4, 4, 0], (12000, 3))
        wall_x = rng.uniform([3, -3, .2], [3, 3, 2], (500, 3))
        wall_y = rng.uniform([-3, 3, .2], [3, 3, 2], (500, 3))
        prior = np.vstack((floor, wall_x, wall_y))
        normals = np.vstack((np.tile([0, 0, 1], (len(floor), 1)),
                             np.tile([1, 0, 0], (len(wall_x), 1)),
                             np.tile([0, 1, 0], (len(wall_y), 1))))
        # Independent samples: floor nearest neighbours must not dominate XY.
        scan = np.vstack((rng.uniform([-4, -4, 0], [4, 4, 0], (6000, 3)),
                          rng.uniform([3, -3, .2], [3, 3, 2], (250, 3)),
                          rng.uniform([-3, 3, .2], [3, 3, 2], (250, 3))))
        mapper = LidarMapper(np.zeros(4), prior, normals)
        for _ in range(5):
            estimate = mapper.update(scan, [.12, -.08, 0, .01])
            np.testing.assert_allclose(estimate, np.zeros(4), atol=.02)
