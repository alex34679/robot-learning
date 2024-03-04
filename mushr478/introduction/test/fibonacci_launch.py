#!/usr/bin/env python

import rospy
import rosunit
import unittest


class TestFibonacciLaunch(unittest.TestCase):
    def test_launch(self):
        self.assertEqual(rospy.get_param("/fibonacci/fibonacci_index"), 123)
        self.assertEqual(
            rospy.get_param("/fibonacci/output_topic"), "test_unique_value"
        )


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_fibonacci_launch", TestFibonacciLaunch)
