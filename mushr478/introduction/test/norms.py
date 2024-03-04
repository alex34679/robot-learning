#!/usr/bin/env python

import numpy as np
import rosunit
import time
import unittest

from introduction.listener import norm_python, norm_numpy


def gen_prim_pyth_trips(limit=None):
    """Generate Pythagorean triplets."""
    u = np.mat(" 1  2  2; -2 -1 -2; 2 2 3")
    a = np.mat(" 1  2  2;  2  1  2; 2 2 3")
    d = np.mat("-1 -2 -2;  2  1  2; 2 2 3")
    uad = np.array([u, a, d])
    m = np.array([3, 4, 5])
    while m.size:
        m = m.reshape(-1, 3)
        if limit:
            m = m[m[:, 2] <= limit]
        for item in m:
            yield item
        m = np.dot(m, uad)


class TestListener(unittest.TestCase):
    def test_norm_python_is_norm(self):
        scalar = np.random.random(1)
        data = np.random.random((100, 2))

        # triangle inequality
        delta = np.full((100, 2), scalar)
        r1 = norm_python(data + delta)
        r2 = norm_python(data) + norm_python(delta)
        self.assertTrue(np.all(r1 < r2))

        # scaling commutes
        r1 = norm_python(scalar * data)
        r2 = scalar * norm_python(data)
        np.testing.assert_allclose(r1, r2)

        # positive definite
        zero_vec = np.zeros((100, 2))
        # Must map to 0
        self.assertFalse(np.any(norm_python(zero_vec)))

    def test_norm_python_is_euclidean(self):
        trips = np.array(list(gen_prim_pyth_trips(1000)))
        np.testing.assert_allclose(norm_python(trips[:, :2]), trips[:, 2])

    def test_norm_python_is_norm_nd(self):
        scalar = np.random.random(1)
        data = np.random.random((47, 8))

        # triangle inequality
        delta = np.full((47, 8), scalar)
        r1 = norm_python(data + delta)
        r2 = norm_python(data) + norm_python(delta)
        self.assertTrue(np.all(r1 < r2))

        # scaling commutes
        r1 = norm_python(scalar * data)
        r2 = scalar * norm_python(data)
        np.testing.assert_allclose(r1, r2)

        # positive definite
        zero_vec = np.zeros((47, 8))
        # Must map to 0
        self.assertFalse(np.any(norm_python(zero_vec)))

    def test_norm_numpy_is_norm(self):
        scalar = np.random.random(1)
        data = np.random.random((100, 2))

        # triangle inequality
        delta = np.full((100, 2), scalar)
        r1 = norm_numpy(data + delta)
        r2 = norm_numpy(data) + norm_numpy(delta)
        self.assertTrue(np.all(r1 < r2))

        # scaling commutes
        r1 = norm_numpy(scalar * data)
        r2 = scalar * norm_numpy(data)
        np.testing.assert_allclose(r1, r2)

        # positive definite
        zero_vec = np.zeros((100, 2))
        # Must map to 0
        self.assertFalse(np.any(norm_numpy(zero_vec)))

    def test_norm_numpy_is_euclidean(self):
        trips = np.array(list(gen_prim_pyth_trips(1000)))
        np.testing.assert_allclose(norm_numpy(trips[:, :2]), trips[:, 2])

    def test_norm_numpy_is_norm_nd(self):
        scalar = np.random.random(1)
        data = np.random.random((47, 8))

        # triangle inequality
        delta = np.full((47, 8), scalar)
        r1 = norm_numpy(data + delta)
        r2 = norm_numpy(data) + norm_numpy(delta)
        self.assertTrue(np.all(r1 < r2))

        # scaling commutes
        r1 = norm_numpy(scalar * data)
        r2 = scalar * norm_numpy(data)
        np.testing.assert_allclose(r1, r2)

        # positive definite
        zero_vec = np.zeros((47, 8))
        # Must map to 0
        self.assertFalse(np.any(norm_numpy(zero_vec)))

    def test_norms_equivalent(self):
        data = np.random.random((2022, 478))
        r1 = norm_python(data)
        r2 = norm_numpy(data)
        np.testing.assert_allclose(r1, r2)

    def test_norm_numpy_faster(self):
        data = np.random.random((10000, 19))

        t1_begin = time.time()
        _ = norm_python(data)
        t1_end = time.time()

        t2_begin = time.time()
        _ = norm_numpy(data)
        t2_end = time.time()
        self.assertTrue(t1_end - t1_begin > t2_end - t2_begin)


if __name__ == "__main__":
    rosunit.unitrun("introduction", "test_listener", TestListener)
