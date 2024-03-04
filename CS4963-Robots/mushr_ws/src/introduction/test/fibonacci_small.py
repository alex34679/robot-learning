#!/usr/bin/env python

import rospy
import rostest
import unittest

from std_msgs.msg import Int64


class TestFibonacciSmall(unittest.TestCase):
    def test_fibonacci_small(self):
        message = rospy.wait_for_message("/fibonacci_output", Int64, timeout=5)
        self.assertEqual(message.data, 89)


if __name__ == "__main__":
    rospy.init_node("test_fibonacci_small")
    rostest.rosrun("introduction", "test_fibonacci_small", TestFibonacciSmall)
