#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from control.controller import BaseController, compute_position_in_frame


class TestController(unittest.TestCase):
    def setUp(self):
        self.defaults = {
            "frequency": 50,
            "finish_threshold": 0.2,
            "exceed_threshold": 2,
            "distance_lookahead": 0.8,
            "min_speed": 0.5,
        }
        self.controller = BaseController(**self.defaults)

    def test_compute_position_in_frame(self):
        p = np.array([2, 2, np.pi / 2])
        frame = np.array([1, 1, 0])

        p_in_frame = compute_position_in_frame(p, frame)
        self.assertEqual(
            p_in_frame.shape[0],
            2,
            msg="compute_position_in_frame should produce a 2D vector",
        )
        np.testing.assert_allclose(
            p_in_frame,
            [1, 1],
            err_msg="The position didn't match our expected position",
        )

        p = np.array([2, 2, np.pi / 2])
        frame = np.array([1, 1, np.pi])

        p_in_frame = compute_position_in_frame(p, frame)
        self.assertEqual(
            p_in_frame.shape[0],
            2,
            msg="compute_position_in_frame should produce a 2D vector",
        )
        np.testing.assert_allclose(
            p_in_frame,
            [-1, -1],
            err_msg="The position didn't match our expected position",
        )

    def test_get_reference_index_on_path(self):
        straight_path_xytv = np.array([[x, 0, 0, 1] for x in range(10)], dtype=np.float)
        pose_index = 4
        pose = straight_path_xytv[pose_index, :3]
        reference_index = self.controller.get_reference_index(
            pose, straight_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreater(
            reference_index,
            pose_index,
            msg="Reference index should be ahead of the current state",
        )

        distance = np.linalg.norm(straight_path_xytv[reference_index, :2] - pose[:2])
        # States on the straight path differ by a distance of 1, so the distance
        # to the reference state should be within 1 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            1.0,
            msg="There is a reference index that would match the lookahead better",
        )

    def test_get_reference_index_near_path(self):
        straight_path_xytv = np.array([[x, 0, 0, 1] for x in range(10)], dtype=np.float)
        closest_index = 4
        pose = straight_path_xytv[closest_index, :3]
        pose += np.array([0.15, 0, 0])
        reference_index = self.controller.get_reference_index(
            pose, straight_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreaterEqual(
            reference_index,
            closest_index,
            msg="Reference index should be ahead of the nearest state",
        )

        distance = np.linalg.norm(straight_path_xytv[reference_index, :2] - pose[:2])
        # States on the straight path differ by a distance of 1, so the distance
        # to the reference state should be within 1 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            1.0,
            msg="There is a reference index that would match the lookahead better",
        )

    def test_get_reference_index_near_dense_path(self):
        dense_path_xytv = np.zeros((100, 4))
        dense_path_xytv[:, 0] = np.linspace(0, 20, 100)
        dense_path_xytv[:, 3] = 1
        closest_index = 30
        pose = dense_path_xytv[closest_index, :3]
        pose += np.array([0.01, 0, 0])
        reference_index = self.controller.get_reference_index(
            pose, dense_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreaterEqual(
            reference_index,
            closest_index,
            msg="Reference index should be ahead of the nearest state",
        )

        distance = np.linalg.norm(dense_path_xytv[reference_index, :2] - pose[:2])
        # States on the dense path differ by a distance of ~0.2, so the distance
        # to the reference state should be within 0.2 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            0.21,
            msg="There is a reference index that would match the lookahead better",
        )


if __name__ == "__main__":
    rosunit.unitrun("control", "test_controller", TestController)
