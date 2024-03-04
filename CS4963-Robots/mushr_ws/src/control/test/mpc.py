#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from nav_msgs.msg import MapMetaData

from control.mpc import ModelPredictiveController


class TestMPCController(unittest.TestCase):
    def setUp(self):
        permissible_region = np.ones((1000, 1000), dtype=bool)
        permissible_region[500:600, 500:600] = False

        self.defaults = {
            "frequency": 50,
            "finish_threshold": 0.2,
            "exceed_threshold": 4.0,
            "distance_lookahead": 1.0,
            "min_speed": 0.5,
            "car_length": 0.33,
            "car_width": 0.15,
            "min_delta": -0.34,
            "max_delta": 0.34,
            "error_w": 1.0,
            "collision_w": 1.0,
            "K": 3,
            "T": 2,
            "kinematics_params": {
                "vel_std": 0.1,
                "delta_std": 0.1,
                "x_std": 0.1,
                "y_std": 0.1,
                "theta_std": 0.1,
            },
            "permissible_region": permissible_region,
            "map_info": MapMetaData(resolution=0.1),
        }

    def test_sample_controls(self):
        params = self.defaults.copy()
        params["K"] = 10
        params["T"] = 4
        controller = ModelPredictiveController(**params)

        controls = controller.sample_controls()
        self.assertEqual(
            controls.shape,
            (controller.K, controller.T, 2),
            msg="sample_controls should return K T-length control sequences",
        )

        for k in range(controller.K):
            np.testing.assert_array_equal(
                controls[k, :, 1],
                controls[k, 0, 1],
                err_msg="Steering angle should be same across time",
            )

        delta_diffs = np.diff(controls[:, 0, 1])
        np.testing.assert_allclose(
            delta_diffs,
            delta_diffs[0],
            err_msg="Change in steering angle should be same across K controls",
        )

    def test_get_rollout(self):
        params = self.defaults.copy()
        params["K"] = 10
        params["T"] = 4
        controller = ModelPredictiveController(**params)
        dt = 0.1

        controls = controller.sample_controls()
        controls[:, :, 0] = 1.0  # set velocity

        pose = np.array([0, 0, np.pi / 4])
        rollouts = controller.get_rollout(pose, controls, dt)
        self.assertEqual(
            rollouts.shape,
            (controller.K, controller.T + 1, 3),
            msg="get_rollout should return K (T+1)-length state sequences",
        )

        for k in range(controller.K):
            np.testing.assert_allclose(
                rollouts[k, 0, :],
                pose,
                err_msg="Rollouts start from the current state",
            )

        translated_pose = np.array([47, 8, np.pi / 4])
        translated_rollouts = controller.get_rollout(translated_pose, controls, dt)
        self.assertEqual(
            translated_rollouts.shape,
            (controller.K, controller.T + 1, 3),
            msg="get_rollout should return K (T+1)-length state sequences",
        )

        for k in range(controller.K):
            np.testing.assert_allclose(
                translated_rollouts[k, 0, :],
                translated_pose,
                err_msg="Rollouts start from the current state",
            )

        # Compare original rollout to translated rollout
        np.testing.assert_allclose(
            translated_rollouts - translated_pose.reshape((1, 1, 3)),
            rollouts - pose.reshape((1, 1, 3)),
            err_msg="Rollouts are relative to the current state",
        )

        diffs = rollouts[:, 1 : controller.T, :] - rollouts[:, 0 : controller.T - 1, :]
        translated_diffs = (
            translated_rollouts[:, 1 : controller.T, :]
            - translated_rollouts[:, 0 : controller.T - 1, :]
        )
        for t in range(diffs.shape[1]):
            np.testing.assert_allclose(
                diffs[:, t, :],
                controller.motion_model.compute_changes(
                    rollouts[:, t, :], controls[:, t, :], dt
                ),
                err_msg="Difference between rollout states should match compute_changes",
            )
            np.testing.assert_allclose(
                translated_diffs[:, t, :],
                controller.motion_model.compute_changes(
                    translated_rollouts[:, t, :], controls[:, t, :], dt
                ),
                err_msg="Difference between rollout states should match compute_changes",
            )
            self.assertGreater(
                np.linalg.norm(diffs[:, t, :]),
                0,
                msg="compute_changes should be different at each time step",
            )
            self.assertGreater(
                np.linalg.norm(translated_diffs[:, t, :]),
                0,
                msg="compute_changes should be different at each time step",
            )
        np.testing.assert_allclose(
            translated_diffs,
            diffs,
            err_msg="compute_changes is relative to the rollout state",
        )

    def test_distance_cost(self):
        params = self.defaults.copy()
        params["K"] = 1
        params["T"] = 1
        controller = ModelPredictiveController(**params)

        rollouts = np.array([[0, 0, np.pi / 2], [0, 0, np.pi / 2]]).reshape((1, 2, 3))
        reference_xytv = np.array([3, 4, np.pi, 1])
        cost = controller.compute_distance_cost(rollouts, reference_xytv)[0]
        np.testing.assert_allclose(
            cost,
            5,
            err_msg="The cost should be the distance from the last state of the rollout",
        )

        params = self.defaults.copy()
        params["K"] = 1
        params["T"] = 7
        controller = ModelPredictiveController(**params)

        rollouts = np.ones((1, 8, 3))
        rollouts[0, -1, :] = [0, 0, np.pi / 2]
        reference_xytv = np.array([3, 4, np.pi, 1])
        cost = controller.compute_distance_cost(rollouts, reference_xytv)[0]
        np.testing.assert_allclose(
            cost,
            5,
            err_msg="The cost should be the distance from the last state of the rollout",
        )

    def test_collision_cost_no_obstacle(self):
        params = self.defaults.copy()
        params["K"] = 1
        params["T"] = 1
        controller = ModelPredictiveController(**params)

        rollouts = np.array([[5, 5, 0], [5, 5, 0]]).reshape((1, 2, 3))
        cost = controller.compute_collision_cost(rollouts, None)[0]
        np.testing.assert_allclose(
            cost,
            0,
            err_msg="There isn't an obstacle at this position",
        )

    def test_collision_cost_obstacle(self):
        params = self.defaults.copy()
        params["K"] = 1
        params["T"] = 1
        params["collision_w"] = 100
        controller = ModelPredictiveController(**params)

        rollouts = np.array([[5, 5, 0], [50, 50, 0]]).reshape((1, 2, 3))
        cost = controller.compute_collision_cost(rollouts, None)[0]
        self.assertGreater(
            cost,
            0,
            msg="There is an obstacle at this position",
        )

        num_collisions = 1
        self.assertEqual(
            cost,
            100 * num_collisions,
            msg="Be sure to multiply by the collision weight self.collision_w",
        )

    def test_rollout_cost(self):
        params = self.defaults.copy()
        params["K"] = 1
        params["T"] = 1
        controller = ModelPredictiveController(**params)

        poses = np.array([[5, 5, np.pi / 2], [50, 50, np.pi / 2]]).reshape((1, 2, 3))
        reference_xytv = np.array([53, 54, np.pi, 2])
        cost = controller.compute_rollout_cost(poses, reference_xytv)[0]
        np.testing.assert_allclose(
            cost,
            6,
            err_msg="Total cost should be 6 (distance cost of 5, collision cost of 1)",
        )


if __name__ == "__main__":
    rosunit.unitrun("control", "test_mpc", TestMPCController)
