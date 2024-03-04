#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from control.pid import PIDController


class TestPIDController(unittest.TestCase):
    def setUp(self):
        self.defaults = {
            "frequency": 50,
            "finish_threshold": 0.2,
            "exceed_threshold": 4.0,
            "distance_lookahead": 1.0,
            "min_speed": 0.5,
            "kp": 1.0,
            "kd": 1.0,
        }

        self.controller = PIDController(**self.defaults)

        self.pose = np.array([2, 2, np.pi / 2])
        self.reference_xytv = np.array([1, 1, np.pi, 0.5])

    def test_get_error(self):
        error = self.controller.get_error(self.pose, self.reference_xytv)
        self.assertEqual(
            error.shape[0],
            2,
            msg="get_error should produce a 2D vector",
        )
        np.testing.assert_allclose(
            error,
            [-1, -1],
            err_msg="The error didn't match our expected error",
        )

    def test_get_control_p(self):
        self.controller.kp = 10
        self.controller.kd = 0

        error = np.array([0.3, 0.4])
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
        self.assertLess(
            control[1],
            0,
            msg="Steering angle should be negative for positive cross-track error",
        )
        self.assertEqual(
            abs(control[1]),
            abs(10 * error[1]),
            msg="Steering angle should be proportional to cross-track error",
        )

        error = np.array([0.3, -0.4])
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
        self.assertGreater(
            control[1],
            0,
            msg="Steering angle should be positive for negative cross-track error",
        )
        self.assertEqual(
            abs(control[1]),
            abs(10 * error[1]),
            msg="Steering angle should be proportional to cross-track error",
        )

    def test_get_control_d(self):
        self.controller.kp = 0
        self.controller.kd = 10

        error = np.array([0.3, 0.4])
        control = self.controller.get_control(self.pose, self.reference_xytv, error)

        self.assertEqual(
            control[1],
            5,
            msg="Steering angle should be proportional to derivative",
        )

    def test_get_control_pd(self):
        self.controller.kp = 1
        self.controller.kd = 1

        error = np.array([0.3, 0.4])
        control = self.controller.get_control(self.pose, self.reference_xytv, error)

        self.assertEqual(
            control[0],
            self.reference_xytv[3],
            msg="Velocity should match reference velocity",
        )
        np.testing.assert_allclose(
            control,
            [0.5, 0.1],
            err_msg="The control didn't match our expected control",
        )


if __name__ == "__main__":
    rosunit.unitrun("control", "test_pid", TestPIDController)
