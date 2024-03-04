#!/usr/bin/env python

import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped, Pose, Point

from introduction.listener import PoseListener


class TestSubscriber(unittest.TestCase):
    def setUp(self):
        self.listener = PoseListener()

    def test_path_publisher(self):
        pub = rospy.Publisher("/car/car_pose", PoseStamped)
        r = rospy.Rate(100)
        for i in range(self.listener.size * 3):
            pub.publish(PoseStamped(pose=Pose(position=Point(x=1, y=2))))
            r.sleep()
        self.assertTrue(self.listener.done)
        self.assertEqual(len(self.listener.storage), self.listener.size)
        self.assertEqual(self.listener.storage[20][0], 1)


if __name__ == "__main__":
    rospy.init_node("test_pose_listener")
    rostest.rosrun("introduction", "test_pose_listener", TestSubscriber)
