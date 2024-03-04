#!/usr/bin/env python

import collections
import numpy as np
import rosunit
import unittest

from planning.problems import R2Problem
from planning.samplers import LatticeSampler, HaltonSampler
from planning.roadmap import Roadmap


class TestRoadmap(unittest.TestCase):
    def setUp(self):
        permissible_region = np.ones((10, 10), dtype=bool)
        permissible_region[3:7, 3:7] = 0
        self.problem = R2Problem(permissible_region)
        self.sampler = LatticeSampler(self.problem.extents)

    def _count_outdegree(self, edges, directed):
        counter = collections.Counter()
        for (u, v) in edges:
            counter[u] += 1
            if not directed:
                counter[v] += 1
        return counter

    def test_r2problem_roadmap_disconnected(self):
        num_vertices = 9
        connection_radius = 0.1
        lazy = False
        Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)

    def test_r2problem_roadmap_edge_collisions(self):
        # This lattice roadmap has vertices for x and y = (1.67, 5.0, 8.33).
        # Setting the connection radius to 15 creates a fully-connected graph.
        num_vertices = 9
        connection_radius = 15
        lazy = False
        rm = Roadmap(
            self.problem,
            self.sampler,
            num_vertices,
            connection_radius,
            lazy,
        )
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        for node in range(rm.graph.number_of_nodes()):
            deg = outdegree.get(node, 0)
            self.assertGreater(deg, 0, msg="All nodes should be connected")

        # Uncomment this line to visualize the roadmap
        # rm.visualize(show_edges=True)

        test_node = 1
        test_state = rm.vertices[test_node, :]
        self.assertEqual(
            outdegree[test_node],
            2,
            msg="State {} should be connected to two neighbors".format(test_state),
        )

    def test_r2problem_roadmap_four(self):
        # This lattice roadmap has vertices for x and y = (0.5, 1.5, ..., 9.5).
        # Setting the radius to 1.0 creates a four-connected lattice graph,
        # excluding vertices that are in collision.

        num_vertices = 100
        connection_radius = 1.0
        lazy = False
        rm = Roadmap(
            self.problem,
            self.sampler,
            num_vertices,
            connection_radius,
            lazy,
        )
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        for node in range(rm.graph.number_of_nodes()):
            deg = outdegree.get(node, 0)
            self.assertGreater(deg, 0, msg="All nodes should be connected")
            self.assertLessEqual(
                deg, 4, msg="All nodes should be connected to at most four neighbors"
            )

        # These start and goal states are in the center of a lattice cell, so
        # they should be connected to four neighbors (the corners of the cell).
        start_state = np.ones(2)
        start_node = rm.add_node(start_state, is_start=True)
        goal_state = 9 * np.ones(2)
        goal_node = rm.add_node(goal_state, is_start=False)
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        self.assertEqual(
            outdegree[start_node], 4, msg="Start should be connected to four neighbors"
        )
        self.assertEqual(
            outdegree[goal_node], 4, msg="Goal should be connected to four neighbors"
        )

        # Uncomment this line to visualize the roadmap
        # rm.visualize(show_edges=True)

    def test_r2problem_roadmap_eight(self):
        # This lattice roadmap has vertices for x and y = (0.5, 1.5, ..., 9.5).
        # Setting the radius to sqrt(2) creates an eight-connected lattice graph,
        # excluding vertices that are in collision.

        num_vertices = 100
        connection_radius = np.sqrt(2)
        lazy = False
        rm = Roadmap(
            self.problem,
            self.sampler,
            num_vertices,
            connection_radius,
            lazy,
        )
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        for node in range(rm.graph.number_of_nodes()):
            deg = outdegree.get(node, 0)
            self.assertGreater(deg, 0, msg="All nodes should be connected")
            self.assertLessEqual(
                deg, 8, msg="All nodes should be connected to at most four neighbors"
            )

        # This start state is on an edge of a lattice cell, so it should be
        # connected to six neighbors (0.5, 0.5), (1.5, 0.5), (2.5, 0.5),
        # (0.5, 1.5), (1.5, 1.5), (2.5, 1.5).
        start_state = np.array([1.5, 1.0])
        start_node = rm.add_node(start_state, is_start=True)
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        self.assertEqual(
            outdegree[start_node], 6, msg="Start should be connected to four neighbors"
        )

        # Uncomment this line to visualize the roadmap
        # rm.visualize(show_edges=True)

    def test_r2problem_roadmap_halton(self):
        self.sampler = HaltonSampler(self.problem.extents)

        num_vertices = 36
        connection_radius = 2.5
        lazy = False
        rm = Roadmap(
            self.problem,
            self.sampler,
            num_vertices,
            connection_radius,
            lazy,
        )
        self.assertEqual(
            rm.graph.number_of_nodes(), num_vertices, msg="Incorrect number of vertices"
        )

        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        for node in range(rm.graph.number_of_nodes()):
            deg = outdegree.get(node, 0)
            self.assertGreater(deg, 0, msg="All nodes should be connected")

        try:
            state = 5 * np.ones(2)
            node = rm.add_node(state, is_start=True)
            self.fail("state in collision should error")
        except ValueError:
            pass

        state = 0.1 * np.ones(2)
        node = rm.add_node(state, is_start=True)
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        self.assertEqual(
            outdegree[node], 2, msg="Node should be connected to two neighbors"
        )

        state = np.ones(2)
        node = rm.add_node(state, is_start=True)
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        self.assertEqual(
            outdegree[node], 5, msg="Node should be connected to five neighbors"
        )

        state = 9 * np.ones(2)
        node = rm.add_node(state, is_start=True)
        outdegree = self._count_outdegree(rm.graph.edges(), directed=False)
        self.assertEqual(
            outdegree[node], 4, msg="Node should be connected to four neighbors"
        )

        # Uncomment this line to visualize the roadmap
        # rm.visualize(show_edges=True)


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_roadmap", TestRoadmap)
