#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import rrt
from planning.samplers import HaltonSampler
from planning.problems import R2Problem, SE2Problem
from planning.roadmap import Roadmap


class TestRRTR2(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(permissible_region, check_resolution=0.5)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_rrt_r2problem(self):
        num_vertices = 10
        connection_radius = 5.0
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius)

        start_id = rm.add_node(np.array([1, 1]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8]), is_start=False)
        correct_path = [0, 1, 3, 5, 6, 8, 10, 13, 16, 25, 30, 66, 86, 88]

        path, parents = rrt.rrt(rm, start_id, goal_id)
        
        np.testing.assert_almost_equal(
            rm.compute_path_length(path),
            rm.compute_path_length(correct_path),
            err_msg="RRT implementation is incorrect",
        )

        self.assertEqual(
            rm.edges_evaluated, 123, msg="Incorrect number of edges evaluated"
        )

        self.assertEqual(
            path,
            correct_path,
            msg="RRT implementation is incorrect",
        )


class TestRRTSE2(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = SE2Problem(permissible_region, curvature=3.0)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_rrt_se2problem(self):
        num_vertices = 8
        connection_radius = 5.0
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius)

        start_id = rm.add_node(np.array([1, 1, 0]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8, 0]), is_start=False)
        correct_path = [0, 5, 6, 8, 9, 11, 13, 15, 17, 23, 31, 36, 39, 41, 43, 44, 49]

        path, parents = rrt.rrt(rm, start_id, goal_id, bias=0.05, eta=0.2, max_iter=2500)

        np.testing.assert_almost_equal(
            rm.compute_path_length(path),
            rm.compute_path_length(correct_path),
            err_msg="RRT implementation is incorrect",
        )

        self.assertEqual(
            rm.edges_evaluated, 158, msg="Incorrect number of edges evaluated"
        )

        self.assertEqual(
            path,
            correct_path,
            msg="RRT implementation is incorrect",
        )


if __name__ == "__main__":

    rosunit.unitrun("planning", "test_rrt_r2", TestRRTR2)
    rosunit.unitrun("planning", "test_rrt_se2", TestRRTSE2)
