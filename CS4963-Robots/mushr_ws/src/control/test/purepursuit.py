#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from control.purepursuit import PurePursuitController


class TestPPController(unittest.TestCase):
    def setUp(self):
        self.defaults = {
            "frequency": 50,
            "distance_lookahead": 0.1,
            "finish_threshold": 1.0,
            "exceed_threshold": 4.0,
            "min_speed": 0.5,
            "car_length": 1,
        }

        self.controller = PurePursuitController(**self.defaults)

        self.pose = np.array([2, 2, np.pi / 2])
        self.reference_xytv = np.array([1, 1, np.pi, 0.5])

    def test_get_error(self):
        self.pose = np.array([2, 2, np.pi])
        error = self.controller.get_error(self.pose, self.reference_xytv)
        self.assertEqual(
            error.shape[0],
            2,
            msg="get_error should produce a 2D vector",
        )
        np.testing.assert_allclose(
            error,
            [1, 1],
            err_msg="The error didn't match our expected error",
        )

    def test_get_control(self):
        error = np.array([1, 1])
        control = self.controller.get_control(self.pose, self.reference_xytv, error)
        self.assertEqual(
            control.shape,
            (2,),
            msg="get_control should return a 2D vector [velocity, steering angle]",
        )
        self.assertEqual(
            control[0],
            self.reference_xytv[3],
            msg="Velocity should match reference velocity",
        )
        # Remember to incorporate the car_length into your calculation!
        np.testing.assert_almost_equal(
            control[1],
            np.pi / 4,
            err_msg="The steering angle didn't match our expected steering angle",
        )


if __name__ == "__main__":
    rosunit.unitrun("control", "test_pure_pursuit", TestPPController)
