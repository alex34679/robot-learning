import numpy as np

from cse478 import utils
from planning import dubins


class PlanarProblem(object):
    def __init__(self, permissible_region, map_info=None, check_resolution=0.1):
        """Construct a planar planning problem.

        Args:
            permissible_region: Boolean np.array with shape map height x map width,
                where one indicates that the location is permissible
            map_info: map information, returned by get_map
            check_resolution: collision-checking resolution
        """
        self.permissible_region = permissible_region
        self.map_info = map_info
        self.check_resolution = check_resolution

        height, width = permissible_region.shape
        self.extents = np.zeros((3, 2))
        self.extents[0, 1] = width
        self.extents[1, 1] = height

        if map_info is not None:
            map_angle = utils.quaternion_to_angle(map_info.origin.orientation)
            assert map_angle == 0
            utils.map_to_world(self.extents.T, map_info)
        self.extents = self.extents[:2, :]

    def check_state_validity(self, states):
        """Return whether states are valid.

        Valid states are within the extents of the map and collision-free.

        Args:
            states: np.array with shape N x D (where D may be 2 or 3)

        Returns:
            valid: np.array of Booleans with shape N
        """
        x = states[:, 0]
        y = states[:, 1]
        valid = np.ones_like(x, dtype=bool)  # feel free to delete this line
        #validityMtrx = np.zeros_like(states)

        # Check that x and y are within the extents of the map.
        # BEGIN QUESTION 1.2
        # Check X Validity
        xMin, xMax = self.extents[0]
        yMin, yMax = self.extents[1]
        xvals = (xMax <= x) | (x < xMin)
        yvals = (yMax <= y) | (y < yMin)
        #valid[:] = np.invert((yvals[:] | xvals[:]))
        valid[:] = np.invert((yvals[:] | xvals[:]))
        # END QUESTION 1.2

        # The units of the state are meters and radians. We need to convert the
        # meters to pixels, in order to index into the permissible region. This
        # function converts them in place.
        if self.map_info is not None:
            utils.world_to_map(states, self.map_info)

        # For states within the extents of the map, collision check by reading
        # the corresponding entry of self.permissible_region. For simplicity,
        # we'll assume the robot is a point robot: just index directly with the
        # robot state x and y pixel indices into self.permissible_region.
        #
        # Hint: use the `astype` method to cast the x and y pixel positions into
        # integers. Then, index into self.permissible_region, remembering that
        # the zeroth dimension is the height.
        # BEGIN QUESTION 1.2

        # Get valid positions and normalize them
        valid[valid] = self.permissible_region[y[valid].astype(int), x[valid].astype(int)] & valid[valid]
        # END QUESTION 1.2

        # Convert the units back from pixels to meters for the caller
        if self.map_info is not None:
            utils.map_to_world(states, self.map_info)

        return valid

    def check_edge_validity(self, q1, q2):
        """Return whether an edge is valid.

        Args:
            q1, q2: np.arrays with shape D (where D may be 2 or 3)

        Returns:
            valid: True or False
        """
        path, length = self.steer(q1, q2)
        if length == 0:
            return False
        return self.check_state_validity(path).all()

    def compute_heuristic(self, q1, q2):
        """Compute an admissible heuristic between two states.

        Args:
            q1, q2: np.arrays with shape (N, D) (where D may be 2 or 3)

        Returns:
            heuristic: np.array with shape N of cost estimates between pairs of states
        """
        # Subclasses can override this with more efficient implementations.
        start, end = np.atleast_2d(q1), np.atleast_2d(q2)
        start_ind = np.arange(start.shape[0])
        end_ind = np.arange(end.shape[0])
        # We'll use broadcasting semantics to match up
        # potentially differently shaped inputs
        broad = np.broadcast(start_ind, end_ind)
        num_pairs = broad.size
        heuristic_cost = np.empty((num_pairs))
        for i, (start_i, end_i) in enumerate(zip(*broad.iters)):
            _, length = self.steer(start[start_i], end[end_i])
            heuristic_cost[i] = length
        return heuristic_cost

    def steer(self, q1, q2, **kwargs):
        """Return a local path connecting two states.

        Intermediate states are used for edge collision-checking.

        Args:
            q1, q2: np.arrays with shape D (where D may be 2 or 3)

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        raise NotImplementedError


class R2Problem(PlanarProblem):
    def compute_heuristic(self, q1, q2):
        """Compute the Euclidean distance between two states.

        Args:
            q1, q2: np.arrays with shape (N, 2)

        Returns:
            heuristic: cost estimate between two states
        """
        return np.linalg.norm(np.atleast_2d(q2) - np.atleast_2d(q1), axis=1)

    def steer(self, q1, q2, resolution=None, interpolate_line=True):
        """Return a straight-line path connecting two R^2 states.

        Args:
            q1, q2: np.arrays with shape 2
            resolution: the space between waypoints in the resulting path
            interpolate_line: whether to provide fine waypoint discretization
             for line segments

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        if resolution is None:
            resolution = self.check_resolution
        q1 = q1.reshape((1, -1))
        q2 = q2.reshape((1, -1))
        dist = np.linalg.norm(q2 - q1)
        if not interpolate_line or dist < resolution:
            return np.vstack((q1, q2)), dist
        q1_toward_q2 = (q2 - q1) / dist
        steps = np.hstack((np.arange(0, dist, resolution), np.array([dist]))).reshape(
            (-1, 1)
        )
        return q1 + q1_toward_q2 * steps, dist


class SE2Problem(PlanarProblem):
    def __init__(
        self, permissible_region, map_info=None, check_resolution=0.01, curvature=1.0
    ):
        super(SE2Problem, self).__init__(permissible_region, map_info, check_resolution)
        self.curvature = curvature
        self.extents = np.vstack((self.extents, np.array([[-np.pi, np.pi]])))

    def compute_heuristic(self, q1, q2):
        """Compute the length of the Dubins path between two SE(2) states.

        Args:
            q1, q2: np.arrays with shape (N, 3)

        Returns:
            heuristic: cost estimate between two states
        """
        start, end = np.atleast_2d(q1), np.atleast_2d(q2)
        # This function will handle broadcasting start and end,
        # if they're compatible
        heuristic_cost = dubins.path_length(start, end, self.curvature)
        return heuristic_cost

    def steer(self, q1, q2, resolution=None, interpolate_line=True):
        """Return a Dubins path connecting two SE(2) states.

        Args:
            q1, q2: np.arrays with shape 3
            resolution: the space between waypoints in the resulting path
            interpolate_line: whether to provide fine waypoint discretization
             for line segments

        Returns:
            path: sequence of states on Dubins path between q1 and q2
            length: length of local path
        """
        if resolution is None:
            resolution = self.check_resolution
        path, length = dubins.path_planning(
            q1,
            q2,
            self.curvature,
            resolution=resolution,
            interpolate_line=interpolate_line,
        )
        return path, length
