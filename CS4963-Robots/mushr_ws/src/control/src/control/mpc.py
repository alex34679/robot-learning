from __future__ import division
import numpy as np

from cse478 import utils
from localization.motion_model import KinematicCarMotionModel
from control.controller import BaseController


class ModelPredictiveController(BaseController):
    def __init__(self, **kwargs):
        self.__properties = {
            "car_length",
            "car_width",
            "collision_w",
            "error_w",
            "min_delta",
            "max_delta",
            "K",
            "T",
            "kinematics_params",
            "permissible_region",
            "map_info",
        }

        if not self.__properties.issubset(set(kwargs)):
            raise ValueError(
                "Invalid keyword argument provided",
                self.__properties.difference(set(kwargs)),
            )
        self.__dict__.update(kwargs)
        assert self.K > 0 and self.T >= 0
        self.motion_model = KinematicCarMotionModel(
            self.car_length, **self.kinematics_params
        )

        super(ModelPredictiveController, self).__init__(
            **{k: kwargs[k] for k in set(kwargs) if k not in self.__properties}
        )

    def sample_controls(self):
        """Sample K T-length sequences of controls (velocity, steering angle).

        In your implementation, each of the K sequences corresponds to a
        particular steering angle, applied T times. The K sequences should
        evenly span the steering angle range [self.min_delta, self.max_delta].

        This method is called once per path tracking request, since the sampled
        steering angles are always the same. The velocity returned by this
        method is a placeholder that will later be replaced with the reference
        state's velocity. The return value of this method is stored at
        self.sampled_controls, and the velocity is modified by other methods.

        Returns:
            controls: np.array of controls with shape K x T x 2
                controls[k, t, :] corresponds to the t'th action from the k'th
                control sequence
        """
        controls = np.empty((self.K, self.T, 2))
        controls[:, :, 0] = 0  # to be pulled from the reference path later

        # Hint: you may find the np.linspace function useful for computing the
        # sampled steering angles, and NumPy array broadcasting useful for
        # putting the sampled steering angles into controls.
        # BEGIN QUESTION 4.1
        for item in range(0, self.T):
            controls[:, item, 1] = np.linspace(self.min_delta, self.max_delta, self.K)
        
        # END QUESTION 4.1
        return controls

    def get_rollout(self, pose, controls, dt=0.1):
        """For each of the K control sequences, collect the corresponding
        (T+1)-length sequence of resulting states (including the current state).
        The sequence of states is called a rollout.

        Starting from the current state, call self.motion_model.compute_changes
        with each control to compute the resulting state (according to the
        noise-free kinematic car model).

        Note: this method has to be efficient because it's in the control loop
        (called at each control iteration) at 50Hz. Be sure to vectorize your
        implementation as much as you can! (You should be able to vectorize the
        t'th kinematic car model update across all K.)

        Args:
            pose: np.array of current robot state [x, y, heading]
            controls: np.array of actions to follow with shape K x T x 2
            dt: duration to apply each control in the kinematic car model

        Returns:
            rollouts: np.array of states with shape K x (T+1) x 3
                rollouts[k, t, :] corresponds to the t'th state after following
                actions 0 through t-1 from the k'th control sequence
        """
        assert controls.shape == (self.K, self.T, 2)

        rollouts = np.empty((self.K, self.T + 1, 3))
        rollouts[:, 0, :] = pose  # all K rollouts start at current state

        # BEGIN QUESTION 4.2
        # for each T, vectorize across all K
        for item in range(1, self.T + 1):
            prevState = rollouts[:, item - 1]
            prevSmplCtrl = controls[:, item - 1]
            changes = self.motion_model.compute_changes(prevState, prevSmplCtrl, dt)
            rollouts[:, item] = prevState + changes
        # END QUESTION 4.2
        return rollouts

    def compute_distance_cost(self, rollouts, reference_xyt):
        """Compute the distance cost for each of the K rollouts.

        The distance cost for a rollout is the distance between the final state
        along the rollout and the reference state, multiplied by self.error_w.

        Args:
            rollouts: np.array of states with shape K x T+1 x 3
            reference_xyt: reference state to target [x, y, heading]

        Returns:
            costs: np.array of cost with shape K (one value for each rollout)
        """
        assert rollouts.shape == (self.K, self.T + 1, 3)
        output = np.zeros(self.K)

        # Note: the distance is the norm of the difference between the x- and y-
        # coordinate of the final rollout state, and the x- and y- coordinate of
        # the reference state
        # BEGIN QUESTION 4.3
        output[:] = np.linalg.norm(rollouts[:, self.T, :2] - reference_xyt[:2], axis=1) * self.error_w
        return output
        # END QUESTION 4.3

    def compute_collision_cost(self, rollouts, _):
        """Compute the cumulative collision cost for each of the K rollouts.

        The cumulative collision cost for a rollout is the number of states in
        the rollout that are in collision, multiplied by self.collision_w.

        Args:
            rollouts: np.array of states with shape K x T+1 x 3

        Returns:
            costs: np.array of cost with shape K
                (one cumulative cost value for all the states in each rollout)
        """
        assert rollouts.shape == (self.K, self.T + 1, 3)

        # Check the states in the rollouts for collisions, count the number of
        # collisions in each rollout, and weight by self.collision_w.
        #
        # Hint: the check_collisions_in_map helper method takes a np.array of
        # states (with shape N x 3) and returns a np.array of collision check
        # results (with shape N x 1). You may find the np.reshape function
        # useful for converting the rollout states into the expected format and
        # the collision check results back into a useful shape. You should only
        # need one call to check_collisions_in_map.

        # BEGIN QUESTION 4.3
        newarr = np.reshape(rollouts, (self.K * (self.T + 1), 3))
        collisions = self.check_collisions_in_map(newarr)
        reshaped = collisions.reshape(self.K, self.T + 1)
        total = np.sum(reshaped * self.collision_w, axis=1)
        return total
        # END QUESTION 4.3

    def compute_rollout_cost(self, rollouts, reference_xyt):
        """Compute the cumulative cost for each of the K rollouts.

        Args:
            rollouts: np.array of states with shape K x T+1 x 3
            reference_xyt: reference state to target [x, y, heading]

        Returns:
            costs: np.array of cost with shape K
                (one cumulative cost value for each rollout)
        """
        assert rollouts.shape == (self.K, self.T + 1, 3)
        dist_cost = self.compute_distance_cost(rollouts, reference_xyt)
        coll_cost = self.compute_collision_cost(rollouts, reference_xyt)
        return dist_cost + coll_cost

    def get_error(self, pose, reference_xytv):
        # MPC uses a more complex cost function than this error vector,
        # but we need to measure the distance between the two states.
        return reference_xytv[:2] - pose[:2]

    def get_control(self, pose, reference_xytv, _):
        """Compute the MPC control law.

        First, roll out the K control sequences from the current vehicle state.
        Then, compute the cost of each rollout. Finally, return the first action
        of the control sequence that minimizes cost.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            control: np.array of velocity and steering angle
        """
        assert reference_xytv.shape[0] == 4

        # Set the velocity from the reference velocity
        self.sampled_controls[:, :, 0] = reference_xytv[3]

        # BEGIN QUESTION 4.4
        #rollouts = np.zeros((self.K, self.T + 1, 3))
        rollouts = self.get_rollout(pose, self.sampled_controls)
        # END QUESTION 4.4x
        # BEGIN QUESTION 4.4
        #costs = np.zeros(self.K)
        costs = self.compute_rollout_cost(rollouts, reference_xytv)
        # END QUESTION 4.4

        # Set the controller's rollouts and costs (for visualization purposes).
        with self.state_lock:
            self.rollouts = rollouts
            self.costs = costs

        # Hint: you may find the np.argmin function useful. Note that the
        # reference velocity has already been stored in self.sampled_controls.
        # BEGIN QUESTION 4.4
        return self.sampled_controls[np.argmin(costs), 0]
        # END QUESTION 4.4


    def reset_state(self):
        super(ModelPredictiveController, self).reset_state()
        with self.state_lock:
            self.sampled_controls = self.sample_controls()
            self.map_poses = np.zeros((self.K * (self.T + 1), 3))
            self.bbox_map = np.zeros((self.K * (self.T + 1), 2, 4))
            self.collisions = np.zeros(self.K * (self.T + 1), dtype=bool)
            self.obstacle_map = ~self.permissible_region
            car_l = self.car_length
            car_w = self.car_width
            self.car_bbox = (
                np.array(
                    [
                        [car_l / 2.0, car_w / 2.0],
                        [car_l / 2.0, -car_w / 2.0],
                        [-car_l / 2.0, car_w / 2.0],
                        [-car_l / 2.0, -car_w / 2.0],
                    ]
                )
                / self.map_info.resolution
            )

    ################################
    # Collision checking utilities #
    ################################

    def check_collisions_in_map(self, poses):
        """Compute which poses are in collision with pixels in the map.

        Args:
            poses: np.array of states to collision check with shape N x 3

        Returns:
            collisions: np.array of collision check results with shape N x 1,
                where 1.0 means in collision and 0.0 means no collision for the
                input state at the corresponding index
        """
        assert poses.ndim == 2 and poses.shape[1] == 3

        utils.world_to_map(poses, self.map_info, out=self.map_poses)
        points = self.map_poses[:, :2]
        thetas = self.map_poses[:, 2]

        rot = np.array(
            [[np.cos(thetas), -np.sin(thetas)], [np.sin(thetas), np.cos(thetas)]]
        ).T

        # Rotate all the the map positions then find points that are
        # offset to match the corners of the car bounding box
        # We're taking advantage of broadcasting to take care
        # of operating over an arbitrary number of bbox points.
        self.bbox_map = (
            np.matmul(self.car_bbox[np.newaxis, ...], rot) + points[:, np.newaxis]
        )
        # Rounding is about 10x more expensive then a truncating cast.
        # We're okay with because we have a wide range of rollouts
        # and we recompute quickly; if we aren't in collision after
        # truncating, by the next timestep we'll be closer and  we'll
        # catch it.
        # bbox_idx = np.empty_like(self.bbox_map, dtype=int)
        # np.rint(self.bbox_map, out=bbox_idx, casting='unsafe')
        bbox_idx = self.bbox_map.astype(int)
        # Clip rollouts to be within map bounds. Assume points outside the
        # map are traversable if the boundary cell is.
        np.clip(
            bbox_idx[..., 1], 0, self.obstacle_map.shape[0] - 1, out=bbox_idx[..., 1]
        )
        np.clip(
            bbox_idx[..., 0], 0, self.obstacle_map.shape[1] - 1, out=bbox_idx[..., 0]
        )
        self.collisions = self.obstacle_map[bbox_idx[..., 1], bbox_idx[..., 0]].max(
            axis=1
        )
        return self.collisions
