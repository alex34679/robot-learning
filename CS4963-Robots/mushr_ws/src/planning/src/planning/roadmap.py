from __future__ import division

import os

import matplotlib.collections
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import pickle

from planning import problems
from planning import samplers


class Roadmap(object):
    def __init__(
        self, problem, sampler, num_vertices, connection_radius, lazy=False, saveto=None
    ):
        """Construct a motion planning roadmap.

        Nodes in the roadmap are labeled with unique integers. These labels
        are indices into the graph's vertices.

        Args:
            problem: a problem description (either an R2Problem or SE2Problem)
            sampler: a sampler (either a HaltonSampler, LatticeSampler, RandomSampler)
            num_vertices: desired number of vertices in the roadmap
            connection_radius: connection radius between vertices
            lazy: whether the roadmap edges should be lazily collision-checked
            saveto: path to cached roadmap data

        """
        self.problem = problem
        self.sampler = sampler
        self.num_vertices = num_vertices
        self.connection_radius = connection_radius
        self.lazy = lazy
        self.saveto = saveto

        # R2 graph is undirected, SE3 graph is directed
        self.directed = isinstance(self.problem, problems.SE2Problem)

        self.start = None
        self.goal = None
        self.edges_evaluated = 0
        self.graph, self.vertices, self.weighted_edges = self.construct()

        # Print some graph summary statistics
        print("Lazy:", self.lazy)
        print("Vertices:", self.num_vertices)
        print("Edges:", self.weighted_edges.shape[0])

    def heuristic(self, n1, n2):
        """Compute the heuristic between two nodes in the roadmap.

        Args:
            n1, n2 (int): node labels

        Returns:
            heuristic: cost estimate between two nodes
        """
        return self.problem.compute_heuristic(
            self.vertices[n1, :],
            self.vertices[n2, :],
        ).item()

    def check_edge_validity(self, n1, n2):
        """Collision check the edge between two nodes in the roadmap.

        Args:
            n1, n2 (int): node labels

        Returns:
            valid: whether the edge is collision-free
        """
        self.edges_evaluated += 1
        return self.problem.check_edge_validity(
            self.vertices[n1, :],
            self.vertices[n2, :],
        )

    def check_weighted_edges_validity(self, weighted_edges):
        """Collision check the edges in weighted_edges.

        Args:
            weighted_edges: np.array of edges and edge lengths with shape
                num_edges x 3, where each row is (u, v, length) and u, v are
                node labels

        Returns:
            weighted_edges: a subset of the original weighted_edges, where only
                rows that correspond to collision-free edges are preserved
        """
        # Hint: call the check_edge_validity method from above.
        # BEGIN QUESTION 1.3
        mask = np.zeros((weighted_edges.shape[0],), dtype=bool)
        # it = np.nditer(weighted_edges)
        # for node in it:
        #     u,v,len = node
        #     if(self.check_edge_validity(int(u), int(v))):
        #         mask[it.index] = True

        # # Vectorized Version
        # vectorFunc = np.vectorize(self.check_edge_validity, otypes=[int])
        # res = vectorFunc(weighted_edges[:, :1].astype(int), weighted_edges[:, 1:2].astype(int))
        u = weighted_edges[:, 0].astype(int)
        v = weighted_edges[:, 1].astype(int)
        for i in range(weighted_edges.shape[0]):
            mask[i] = self.check_edge_validity(u[i], v[i])

        #mask = self.check_edge_validity(weighted_edges[:, :1].astype(int), weighted_edges[:, 1:2].astype(int))
        result = weighted_edges[mask]
        # END QUESTION 1.3
        return result

    def construct(self):
        """Construct the roadmap.

        Initialize the graph, vertices, and weighted_edges fields.

        Returns:
            graph: a NetworkX graph
            vertices: np.array of states with shape num_vertices x D,
                indexed by node labels
            weighted_edges: np.array of edges and edge lengths with shape
                num_edges x 3, where each row is (u, v, length) and u, v are
                node labels
        """
        # Load a cached roadmap, if one exists at that path.
        if self.saveto is not None and os.path.exists(self.saveto):
            try:
                with open(self.saveto, "rb") as f:
                    data = pickle.load(f)
                self.graph = data["graph"]
                self.vertices = data["vertices"]
                self.weighted_edges = data["weighted_edges"]
                print("Loaded roadmap from", self.saveto)
                return self.graph, self.vertices, self.weighted_edges
            except pickle.PickleError:
                pass

        # Compute the set of vertices and edges.
        self.vertices = self.sample_vertices()
        self.weighted_edges = self.connect_vertices(self.vertices)
        if not self.lazy:
            self.weighted_edges = self.check_weighted_edges_validity(
                self.weighted_edges
            )

        # Insert the vertices and edges into a NetworkX graph object
        if self.directed:
            self.graph = nx.DiGraph()
        else:
            self.graph = nx.Graph()
        vbunch = [
            (i, dict(config=config))
            for i, config in zip(np.arange(self.num_vertices, dtype=int), self.vertices)
        ]
        ebunch = [(int(u), int(v), float(w)) for u, v, w in self.weighted_edges]
        self.graph.add_nodes_from(vbunch)
        self.graph.add_weighted_edges_from(ebunch, "weight")

        # Cache this roadmap, if path is specified.
        if self.saveto is not None:
            with open(self.saveto, "wb") as f:
                data = {
                    "graph": self.graph,
                    "vertices": self.vertices,
                    "weighted_edges": self.weighted_edges,
                }
                pickle.dump(data, f)
                print("Saved roadmap to", self.saveto)
        return self.graph, self.vertices, self.weighted_edges

    def sample_vertices(self, max_iters=100):
        """Sample self.num_vertices vertices from self.sampler.

        Args:
            max_iters: maximum number of sampling iterations

        Returns:
            vertices: np.array of states with shape num_vertices x D
        """
        # Lattice sampler only samples a fixed set of samples. Some of them may
        # be in collision, so they're unusable; pretend we only asked for that
        # many vertices.
        if isinstance(self.sampler, samplers.LatticeSampler):
            samples = self.sampler.sample(self.num_vertices)
            valid = self.problem.check_state_validity(samples)
            self.num_vertices = valid.sum()
            return samples[valid, :]

        # For other samplers, sample until the target number of collision-free
        # vertices is reached.
        configs = []
        batch_size = self.num_vertices
        batches_sampled = 0
        valid_samples = 0
        total_samples = 0
        while valid_samples < self.num_vertices and batches_sampled < max_iters:
            samples = self.sampler.sample(batch_size)
            valid = self.problem.check_state_validity(samples)
            configs.append(samples[valid, :])
            valid_samples += valid.sum()
            total_samples += batch_size
            # This adaptively changes the sample batch size based on how hard it
            # is to get collision-free vertices.
            est_validity = valid_samples / total_samples
            if valid_samples > 0:
                batch_size = int((self.num_vertices - valid_samples) / est_validity) + 1
            batches_sampled += 1
        if valid_samples < self.num_vertices:
            print("Did not reach target number of collision-free vertices.")
        configs = np.vstack(configs)
        assert configs.shape[0] >= self.num_vertices
        return configs[: self.num_vertices, :]

    def connect_vertices(self, vertices):
        """Connect vertices within self.connection_radius.

        Returns:
            weighted_edges: np.array of edges and edge lengths with shape
                num_edges x 3, where each row is (u, v, length) and u, v are
                node labels (vertex indices)
        """
        h = np.zeros((self.num_vertices, self.num_vertices))
        for i in range(self.num_vertices):
            u = vertices[i, :]
            # For undirected graphs, only compute heuristic to all later vertices.
            # For directed graphs, compute heuristic to previous vertices as well.
            h[i, i + 1 :] = self.problem.compute_heuristic(u, vertices[i + 1 :, :])

            if self.directed:
                h[i, :i] = self.problem.compute_heuristic(u, vertices[:i, :])

        # Edges where the heuristic distance is greater than the connection
        # radius are dropped. Find the indices to all nonzero heuristics; i is
        # the sources and j is the targets.
        h[h > self.connection_radius] = 0
        i, j = h.nonzero()
        i = i.reshape((-1, 1))
        j = j.reshape((-1, 1))
        return np.hstack([i, j, h[i, j]])

    def add_node(self, state, is_start):
        """Add a node for the state, which is either a start or goal.

        Args:
            state: new state to add to the roadmap
            is_start: whether this new state is the start

        Returns:
            new_index: the integer label for the added state
        """
        state = state.astype(float)
        assert state.ndim == 1
        valid = self.problem.check_state_validity(state.reshape((1, -1))).item()
        if not valid:
            raise ValueError("state {} is invalid".format(state))

        index = self.graph.number_of_nodes()

        # This logic is very similar to connect_vertices. Either compute the
        # heuristic from the start to all other vertices, or from all other
        # vertices to the goal.
        if is_start:
            self.start = index
            h = self.problem.compute_heuristic(state, self.vertices)
        else:
            self.goal = index
            h = self.problem.compute_heuristic(self.vertices, state)
        h[h > self.connection_radius] = 0
        (i,) = h.nonzero()
        i = i.reshape((-1, 1))
        index_arr = index * np.ones_like(i)
        if is_start:
            weighted_edges = np.hstack([index_arr, i, h[i]])
        else:
            weighted_edges = np.hstack([i, index_arr, h[i]])

        # Update self.graph, self.vertices, and self.weighted_edges.
        self.graph.add_node(index, config=state)
        self.vertices = np.vstack([self.vertices, state.reshape((1, -1))])
        self.num_vertices += 1
        if not self.lazy:
            weighted_edges = self.check_weighted_edges_validity(weighted_edges)
        ebunch = [(int(u), int(v), float(w)) for u, v, w in weighted_edges]
        self.graph.add_weighted_edges_from(ebunch)
        self.weighted_edges = np.vstack([self.weighted_edges, weighted_edges])
        return index

    def compute_path_length(self, vpath):
        """Compute the path length of a sequence of vertices.

        Args:
            vpath: sequence of vertex labels

        Returns:
            length: path length
        """
        q = self.vertices[vpath, :]
        return self.problem.compute_heuristic(q[:-1, :], q[1:, :]).sum()

    def compute_qpath(self, vpath):
        """Compute a sequence of states from a sequence of vertices.

        Args:
            vpath: sequence of vertex labels

        Returns:
            qpath: sequence of configuration states
        """
        edges = []
        for i in range(1, len(vpath)):
            u, v = vpath[i - 1 : i + 1]
            q1 = self.vertices[u, :]
            q2 = self.vertices[v, :]
            edge, _ = self.problem.steer(q1, q2)
            edges.append(edge)
        return np.vstack(edges)

    def visualize(self, show_edges=False, vpath=None, saveto=None):
        """Visualize the roadmap.

        Args:
            show_edges: whether the roadmap's edges should be shown
            vpath: sequence of vertex labels (or None)
            saveto: path to save roadmap plot (or None)
        """
        plt.imshow(
            self.problem.permissible_region,
            cmap=plt.cm.gray,
            aspect="equal",
            interpolation="none",
            vmin=0,
            vmax=1,
            origin="lower",
            extent=self.problem.extents.ravel()[:4],
        )

        if show_edges:
            edges = []
            for u, v in self.graph.edges():
                q1 = self.vertices[u, :]
                q2 = self.vertices[v, :]
                edge, _ = self.problem.steer(
                    q1, q2, resolution=0.1, interpolate_line=False
                )
                edges.append(edge[:, :2])
            edges = matplotlib.collections.LineCollection(
                edges, colors="#dddddd", zorder=1
            )
            plt.gca().add_collection(edges)

        if vpath is not None:
            qpath = self.compute_qpath(vpath)
            plt.plot(qpath[:, 0], qpath[:, 1], c="#0000ff", zorder=1)

        plt.scatter(self.vertices[:, 0], self.vertices[:, 1], c="k", zorder=2)
        if self.start is not None:
            plt.scatter(
                self.vertices[self.start, 0],
                self.vertices[self.start, 1],
                c="g",
                zorder=3,
            )
        if self.goal is not None:
            plt.scatter(
                self.vertices[self.goal, 0],
                self.vertices[self.goal, 1],
                c="r",
                zorder=3,
            )
        plt.xlim(self.problem.extents[0, :])
        plt.ylim(self.problem.extents[1, :])

        if saveto is not None:
            plt.savefig(saveto, bbox_inches="tight")
            print("Saved graph image to", saveto)
        plt.show()
