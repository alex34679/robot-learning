#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import rostest
import tf2_ros
import time
import unittest

from mushr_sim.srv import CarPose

from cse478.frechet import frdist
from control.control_ros import (
    controllers,
    ControlROS,
    get_ros_params,
)
from control.path_generator import line, circle, wave, left_turn, right_turn


class TestControllerPerformance(unittest.TestCase):
    def setUp(self):
        controller = controllers[controller_type](**TestControllerPerformance.params)
        # Give the simulator time to come up
        rospy.wait_for_service("/mushr_sim/reposition", timeout=4.0)
        self.reposition_proxy = rospy.ServiceProxy("/mushr_sim/reposition", CarPose)
        success = self.reposition_proxy("car", 0, 0, 0)
        self.assertTrue(success, msg="Car could not be repositioned")
        self.controller = ControlROS(
            controller, transform_listener=TestControllerPerformance.tl
        )
        self.controller.start()
        self.controller.controller.ready_event.wait()

    def tearDown(self):
        self.controller.shutdown()
        self.reposition_proxy.close()

    def _test_controller_performance(self, path, frechet_threshold):
        self.controller.follow_path_with_speed(path, 2.0)
        duration = 30
        end_time = time.time() + duration
        car_trace = []
        while time.time() < end_time and self.controller.controller.path is not None:
            start_stamp = time.time()
            self.controller.controller.looped_event.wait(timeout=1.0)
            elapsed = time.time() - start_stamp
            self.assertLess(elapsed, 1.0, msg="No control was received for one second")
            car_trace.append(self.controller._get_car_pose())
        car_trace = np.array(car_trace)
        finished, errored = self.controller.wait_for_finish(timeout=20.0)
        frechet_distance = frdist(path[:2], car_trace[:2])
        self.assertFalse(
            errored, msg="Controller did not reach the last target position"
        )
        self.assertLess(
            frechet_distance,
            frechet_threshold,
            msg="Controller did not follow path as closely as expected",
        )


if __name__ == "__main__":
    # We're going to take this name so that the topic names are the same as when
    # running `control_node`. Convenient for RViz
    rospy.init_node("controller")

    # We've templatized this test. What follows is trickery to call the single
    # test (defined above) in different configurations and present the correct
    # facade to the test runner.
    controller_type, params = get_ros_params()
    params["min_speed"] = 0.0  # remain consistent with old behavior for tests

    performance_expectations = [
        ("line", line(), 0.12),
        ("left_turn", left_turn(), 0.12),
        ("right_turn", right_turn(), 0.12),
        ("circle", circle(), 0.12),
        ("wave", wave(), 1.1),
    ]
    for name, path, frechet_threshold in performance_expectations:

        def test_wrap(path_cap, frechet_threshold_cap):
            # Scope is lexical. If we created this lambda in the outer scope,
            # the lambda would only be able to reference the loop variables.
            # Because these change before the test gets called, all of the tests
            # would just see the values from the last iteration of the loop! By
            # creating a new scope, we are "capturing" the correct values.
            # See "thunk" or "closure".
            return lambda self: self._test_controller_performance(
                path_cap, frechet_threshold_cap
            )

        # The xml report will use the method's name, so we have to manually
        # mangle it with the controller name to ensure all entries appear.
        setattr(
            TestControllerPerformance,
            "test_{}_{}_performance".format(controller_type, name),
            test_wrap(path, frechet_threshold),
        )
    TestControllerPerformance.params = params
    TestControllerPerformance.tl = tf2_ros.TransformListener(tf2_ros.Buffer())
    rostest.rosrun(
        "localization",
        "test_controller_performance_{}".format(controller_type),
        TestControllerPerformance,
    )
