#!/usr/bin/env python

import numpy as np
import rosunit
import unittest

from nav_msgs.msg import MapMetaData

from planning.problems import PlanarProblem, R2Problem
from planning.samplers import LatticeSampler


class TestPlanarProblem(unittest.TestCase):
    def setUp(self):
        permissible_region = np.ones((5, 10), dtype=bool)
        permissible_region[3:7, 3:7] = 0
        self.problem = PlanarProblem(permissible_region)

    def test_planar_state_validity_extents(self):
        states = np.array(
            [
                [1, 1],
                [1, -1],
                [-1, 1],
                [-1, -1],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([1, 0, 0, 0], dtype=bool),
            err_msg="States below the extents are not valid",
        )

        states = np.array(
            [
                [9, 4],
                [9.9, 4.9],
                [9, 1],
                [10, 1],
                [11, 1],
                [11, 11],
                [1, 9],
                [1, 10],
                [1, 11],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([1, 1, 1, 0, 0, 0, 0, 0, 0], dtype=bool),
            err_msg="States above the extents are not valid",
        )

    def test_planar_state_validity_collision(self):
        permissible_region = np.ones((10, 10), dtype=bool)
        permissible_region[4:6, 3:7] = 0
        self.problem = PlanarProblem(permissible_region)

        states = np.array(
            [
                [3, 4],
                [4, 3],
                [3.1, 4.1],
                [4.1, 3.1],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([0, 1, 0, 1], dtype=bool),
            err_msg="States in collision are not valid",
        )

    def test_planar_state_validity_extents_with_map_resolution(self):
        permissible_region = np.ones((5, 10), dtype=bool)
        resolution = 0.1
        map_info = MapMetaData(resolution=resolution)
        self.problem = PlanarProblem(permissible_region, map_info)

        # The permissible region is 10 pixels by 5 pixels. A resolution of 0.1
        # meters/pixels means that map width/height is 1 meter x 0.5
        # meters. This results in extents of (0, 1) x (0, 0.5).
        np.testing.assert_equal(
            self.problem.extents,
            np.array([[0, 1], [0, 0.5]]),
            err_msg="extents do not match expected extents",
        )

        states = np.array(
            [
                [0.4, 0.3],
                [0.5, 0.3],
                [0.7, 0.2],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([1, 1, 1], dtype=bool),
            err_msg="States within the extents are valid",
        )

        states = np.array(
            [
                [4, 3],
                [5, 3],
                [7, 2],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([0, 0, 0], dtype=bool),
            err_msg="States outside the extents are not valid",
        )

    def test_planar_state_validity_extents_with_map_translation(self):
        permissible_region = np.ones((20, 20), dtype=bool)
        resolution = 0.1
        map_info = MapMetaData(resolution=resolution)
        shift = 1
        map_info.origin.position.x -= shift
        map_info.origin.position.y -= shift
        self.problem = PlanarProblem(permissible_region, map_info)

        # The permissible region is 20 pixels by 20 pixels. A resolution of 0.1
        # meters/pixels means that map width/height is 2 meters. Shifting the x
        # and y position of the origin by -1 meter results in extents of (-1, 1)
        # x (-1, 1).
        np.testing.assert_equal(
            self.problem.extents,
            np.array([[-1, 1], [-1, 1]]),
            err_msg="extents do not match expected extents",
        )

        states = np.array(
            [
                [0.4, 0.3],
                [0.5, 0.3],
                [0.7, 0.2],
                [-1.0, -1.0],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([1, 1, 1, 1], dtype=bool),
            err_msg="States within the extents are valid",
        )

        states = np.array(
            [
                [1.4, 1.3],
                [1.5, 1.3],
                [1.7, 1.2],
                [0, 0],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([0, 0, 0, 1], dtype=bool),
            err_msg="States outside the extents are not valid",
        )

        states = np.array(
            [
                [14, 13],
                [15, 13],
                [17, 12],
                [0, 0],
            ],
            dtype=float,
        )
        np.testing.assert_equal(
            self.problem.check_state_validity(states),
            np.array([0, 0, 0, 1], dtype=bool),
            err_msg="States outside the extents are not valid",
        )


class TestR2Problem(unittest.TestCase):
    def setUp(self):
        permissible_region = np.ones((10, 10), dtype=bool)
        permissible_region[3:7, 3:7] = 0
        self.problem = R2Problem(permissible_region)
        self.sampler = LatticeSampler(self.problem.extents)

    def test_r2_heuristic(self):
        # This sampler creates vertices for x and y = (1.67, 5.0, 8.33).
        # Only the state (5.0, 5.0) should be in collision.

        num_samples = 9
        samples = self.sampler.sample(num_samples)
        valid = self.problem.check_state_validity(samples)

        self.assertEqual(
            valid.sum(),
            8,
            msg="Only one sample should be in collision",
        )
        np.testing.assert_equal(
            samples[~valid, :],
            np.array([[5, 5]]),
            err_msg="The sample in collision should be (5, 5)",
        )

        samples = samples[valid, :]
        heuristics = self.problem.compute_heuristic(samples[0, :], samples[1:, :])

        distances = np.linalg.norm(samples[1:, :] - samples[0, :], axis=1)
        np.testing.assert_allclose(
            heuristics,
            distances,
            err_msg="The R2 heuristic should be the Euclidean norm",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_planar_problem", TestPlanarProblem)
    rosunit.unitrun("planning", "test_r2_problem", TestR2Problem)
