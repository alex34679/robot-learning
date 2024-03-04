#!/usr/bin/env python

import rosunit
import unittest

from introduction.fibonacci import compute_fibonacci


class TestFibonacci(unittest.TestCase):
    def test_small(self):
        self.assertEqual(compute_fibonacci(0), 0)
        self.assertEqual(compute_fibonacci(1), 1)
        self.assertEqual(compute_fibonacci(10), 55)

    def test_large(self):
        self.assertEqual(compute_fibonacci(45), 1134903170)
        self.assertEqual(compute_fibonacci(46), 1836311903)
        self.assertEqual(compute_fibonacci(47), 2971215073)

    def test_fibonacci_property(self):
        sequence = [compute_fibonacci(i) for i in range(20)]
        for i in range(2, len(sequence)):
            self.assertEqual(sequence[i - 1] + sequence[i - 2], sequence[i])


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_fibonacci", TestFibonacci)
