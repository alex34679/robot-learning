#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.samplers import HaltonSampler
from planning.problems import R2Problem, SE2Problem
from planning.roadmap import Roadmap


class TestAStarR2(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(permissible_region, check_resolution=0.5)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_astar_r2problem(self):
        num_vertices = 10
        connection_radius = 5.0
        lazy = False
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)

        start_id = rm.add_node(np.array([1, 1]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8]), is_start=False)
        correct_path = [start_id, 9, 2, 7, goal_id]

        path, parents = search.astar(rm, start_id, goal_id)
        self.assertEqual(
            path[0],
            start_id,
            msg="Path must begin at start",
        )
        self.assertEqual(
            path[-1],
            goal_id,
            msg="Path must end at goal",
        )
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

        self.assertEqual(
            parents[start_id],
            search.NULL,
            msg="Parent of start is -1 (sentinel value for null)",
        )
        for start_child in (5, 0, 9):
            self.assertEqual(
                parents[start_child],
                start_id,
                msg="Child of start should have start as parent",
            )
        for parent, child in zip(correct_path[:-1], correct_path[1:]):
            self.assertEqual(
                parents[child],
                parent,
                msg="Previous node in shortest path should be parent",
            )


class TestAStarSE2(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = SE2Problem(permissible_region, curvature=3.0)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_astar_se2problem(self):
        num_vertices = 8
        connection_radius = 5.0
        lazy = False
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)

        start_id = rm.add_node(np.array([1, 1, 0]), is_start=True)
        goal_id = rm.add_node(np.array([7, 1, 0]), is_start=False)
        correct_path = [start_id, 3, goal_id]

        path, parents = search.astar(rm, start_id, goal_id)
        self.assertEqual(
            path[0],
            start_id,
            msg="Path must begin at start",
        )
        self.assertEqual(
            path[-1],
            goal_id,
            msg="Path must end at goal",
        )
        self.assertEqual(
            path,
            correct_path,
            msg="A* implementation is incorrect",
        )

        self.assertEqual(
            parents[start_id],
            search.NULL,
            msg="Parent of start is -1 (sentinel value for null)",
        )
        for parent, child in zip(correct_path[:-1], correct_path[1:]):
            self.assertEqual(
                parents[child],
                parent,
                msg="Previous node in shortest path should be parent",
            )


class TestLazyAStar(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(permissible_region, check_resolution=0.5)
        self.sampler = HaltonSampler(self.problem.extents)

    def test_lazy_astar(self):
        num_vertices = 10
        connection_radius = 5.0
        lazy = True
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)

        start_id = rm.add_node(np.array([1, 1]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8]), is_start=False)
        correct_path = [start_id, 9, 2, 7, goal_id]

        path, parents = search.astar(rm, start_id, goal_id)
        self.assertEqual(
            path[0],
            start_id,
            msg="Path must begin at start",
        )
        self.assertEqual(
            path[-1],
            goal_id,
            msg="Path must end at goal",
        )
        np.testing.assert_almost_equal(
            rm.compute_path_length(path),
            rm.compute_path_length(correct_path),
            err_msg="Lazy A* implementation is incorrect",
        )
        self.assertEqual(
            path,
            correct_path,
            msg="Lazy A* implementation is incorrect",
        )

        self.assertEqual(
            parents[start_id],
            search.NULL,
            msg="Parent of start is -1 (sentinel value for null)",
        )
        for start_child in (5, 0, 9):
            self.assertEqual(
                parents[start_child],
                start_id,
                msg="Child of start should have start as parent",
            )
        for parent, child in zip(correct_path[:-1], correct_path[1:]):
            self.assertEqual(
                parents[child],
                parent,
                msg="Previous node in shortest path should be parent",
            )

        self.assertEqual(
            rm.edges_evaluated, 10, msg="Incorrect number of edges evaluated"
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_astar_r2", TestAStarR2)
    rosunit.unitrun("planning", "test_astar_se2", TestAStarSE2)
    rosunit.unitrun("planning", "test_lazy_astar", TestLazyAStar)
