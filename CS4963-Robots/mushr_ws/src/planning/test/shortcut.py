#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.samplers import HaltonSampler
from planning.problems import R2Problem
from planning.roadmap import Roadmap


class TestShortcut(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(permissible_region, check_resolution=0.5)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_shortcut(self):
        num_vertices = 10
        connection_radius = 5.0
        lazy = False
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)

        start_id = rm.add_node(np.array([1, 1]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8]), is_start=False)
        correct_path = [start_id, 9, 2, 7, goal_id]

        path, _ = search.astar(rm, start_id, goal_id)
        np.testing.assert_almost_equal(
            rm.compute_path_length(path),
            rm.compute_path_length(correct_path),
            err_msg="A* implementation is incorrect",
        )
        self.assertEqual(
            path,
            correct_path,
            msg="A* implementation is incorrect",
        )

        shortcut_path = search.shortcut(rm, path)
        self.assertLess(
            rm.compute_path_length(shortcut_path),
            rm.compute_path_length(path),
            msg="Shortcut should return a shorter path",
        )

        correct_shortcut_path = [start_id, 2, goal_id]
        np.testing.assert_almost_equal(
            rm.compute_path_length(shortcut_path),
            rm.compute_path_length(correct_shortcut_path),
            err_msg="Shortcut implementation is incorrect",
        )
        self.assertEqual(
            shortcut_path,
            correct_shortcut_path,
            msg="Shortcut implementation is incorrect",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_shortcut", TestShortcut)
