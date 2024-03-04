#!/usr/bin/env python

import rosunit
import unittest
import numpy as np

from introduction.indexing import (
    extract_fibonacci_rows,
    increment_rows_with_odd_first_element,
)


class TestIndexing(unittest.TestCase):
    def test_integer_array_indexing(self):
        sequence = [0, 1, 1, 2, 3, 5, 8]
        data = np.arange(60).reshape((10, 6))
        fib_rows = extract_fibonacci_rows(data)
        self.assertEqual(fib_rows.shape, (7, 6), msg="Incorrect output shape")
        for i, fib in enumerate(sequence):
            np.testing.assert_equal(
                fib_rows[i, :],
                data[fib, :],
                err_msg="Fibonacci row does not match",
            )

    def test_boolean_array_indexing(self):
        data = np.arange(18).reshape((6, 3))
        old_data = data.copy()

        increment_rows_with_odd_first_element(data)

        np.testing.assert_equal(
            data[::2, :],
            old_data[::2, :],
            err_msg="Rows with even first element should be unchanged",
        )

        np.testing.assert_equal(
            data[1::2, :],
            old_data[1::2, :] + 1,
            err_msg="Rows with odd first element should be incremented by one",
        )


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_indexing", TestIndexing)
