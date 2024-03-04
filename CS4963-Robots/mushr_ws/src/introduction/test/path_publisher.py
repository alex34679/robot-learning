#!/usr/bin/env python

import numpy as np
import rospy
import rostest
import unittest

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from cse478.collector import MessageCollector
from cse478.utils import pose_stamped_to_pq


class TestPublisher(unittest.TestCase):
    def test_path_publisher(self):
        rospy.init_node("test_introduction_publisher")
        control_collector = MessageCollector("/control", AckermannDriveStamped)
        control_collector.start()

        car_pose_cov = rospy.wait_for_message(
            "/initialpose", PoseWithCovarianceStamped, timeout=3
        )
        self.assertIsNotNone(car_pose_cov)
        np.testing.assert_equal(
            pose_stamped_to_pq(car_pose_cov.pose),
            (np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
        )
        rospy.sleep(5)
        control_msgs = control_collector.stop()
        self.assertTrue(len(control_msgs) > 40)


if __name__ == "__main__":
    rostest.rosrun("introduction", "test_publisher", TestPublisher)
