#!/usr/bin/env python

from __future__ import division

import numpy as np
import rosunit
import unittest

from planning.samplers import HaltonSampler, LatticeSampler


class TestHaltonSampler(unittest.TestCase):
    def setUp(self):
        extents = np.zeros((2, 2))
        extents[:, 1] = 1
        self.sampler = HaltonSampler(extents)
        self.batch_size = 1000

    def test_compute_sample_base_two(self):
        # The Halton sequence for base two begins with
        # 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875

        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 2) for i in range(0, 1)],
            np.array([1]) / 2,
            err_msg="The Halton sequence does not match.",
        )
        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 2) for i in range(1, 3)],
            np.array([1, 3]) / 4,
            err_msg="The Halton sequence does not match.",
        )
        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 2) for i in range(3, 7)],
            np.array([1, 5, 3, 7]) / 8,
            err_msg="The Halton sequence does not match.",
        )
        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 2) for i in range(7, 15)],
            np.array([1, 9, 5, 13, 3, 11, 7, 15]) / 16,
            err_msg="The Halton sequence does not match.",
        )

    def test_compute_sample_base_three(self):
        # The Halton sequence for base three begins with
        # 1/3, 2/3, 1/9, 4/9, 7/9, 2/9, 5/9, 8/9

        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 3) for i in range(0, 2)],
            np.array([1, 2]) / 3,
            err_msg="The Halton sequence does not match.",
        )
        np.testing.assert_allclose(
            [self.sampler.compute_sample(i, 3) for i in range(2, 8)],
            np.array([1, 4, 7, 2, 5, 8]) / 9,
            err_msg="The Halton sequence does not match.",
        )

    def test_sample_2d(self):
        batch1 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch1.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch1 > 0.5).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        batch2 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch2.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch2 > 0.5).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        self.assertFalse(
            np.array_equal(batch1, batch2),
            msg="Each call to sample should return different samples",
        )

    def test_sample_2d_scaled_upper_extent(self):
        extents = np.zeros((2, 2))
        extents[:, 1] = 10
        self.sampler = HaltonSampler(extents)

        batch1 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch1.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch1 > 5).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        batch2 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch2.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch2 > 5).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        self.assertFalse(
            np.array_equal(batch1, batch2),
            msg="Each call to sample should return different samples",
        )

    def test_sample_2d_scaled_lower_extent(self):
        extents = 10 * np.ones((2, 2))
        extents[:, 0] = -10
        self.sampler = HaltonSampler(extents)

        batch1 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch1.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch1 > 0).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        batch2 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch2.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch2 > 0).all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.001,
            err_msg="samples are not distributed uniformly across the extents",
        )

        self.assertFalse(
            np.array_equal(batch1, batch2),
            msg="Each call to sample should return different samples",
        )

    def test_sample_2d_scaled_asymmetric_extent(self):
        batch_size = 20
        extents = np.array([[-5, 10], [4, 8]])
        self.sampler = HaltonSampler(extents)

        batch = self.sampler.sample(batch_size)
        self.assertEqual(
            batch.shape,
            (batch_size, 2),
            msg="sample should return the number of samples requested",
        )

        xmin, xmax = extents[0, :]
        ymin, ymax = extents[1, :]
        self.assertTrue(
            (batch[:, 0] >= xmin).all(),
            msg="sampled x values should be at least the minimum x value",
        )
        self.assertTrue(
            (batch[:, 0] < xmax).all(),
            msg="sampled x values should be less than the maximum x value",
        )
        self.assertTrue(
            (batch[:, 1] >= ymin).all(),
            msg="sampled y values should be at least the minimum y value",
        )
        self.assertTrue(
            (batch[:, 1] < ymax).all(),
            msg="sampled y values should be less than the maximum y value",
        )

    def test_sample_2d_scaled_domain(self):
        # Mock out the Halton sample generator with two samples: (0.4, 0.6) and (0.3, 0.7)
        mocked_halton_gen = np.array([[0.4, 0.6], [0.3, 0.7]])
        sample_size = mocked_halton_gen.shape[0]

        scale = 10
        extents = np.zeros((2, 2))
        extents[:, 1] = scale

        # The Halton samples should be scaled from the range (0, 1) x (0, 1) to
        # match the extents (0, 10) x (0, 10). Therefore, the expected batch of
        # samples is (4, 6) and (3, 7).
        self.sampler = HaltonSampler(extents)
        self.sampler.gen = mocked_halton_gen
        batch = self.sampler.sample(sample_size)
        np.testing.assert_equal(
            batch,
            scale * mocked_halton_gen,
            err_msg="Halton samples were scaled incorrectly",
        )

        shift = 5
        extents -= shift

        # The Halton samples should be scaled from the range (0, 1) x (0, 1) to
        # match the extents (-5, 5) x (-5, 5). Therefore, the expected batch of
        # samples is (-1, 1) and (-2, 2).
        self.sampler = HaltonSampler(extents)
        self.sampler.gen = mocked_halton_gen
        batch = self.sampler.sample(sample_size)
        np.testing.assert_equal(
            batch,
            scale * mocked_halton_gen - shift,
            err_msg="Halton samples were scaled incorrectly",
        )

    def test_sample_3d(self):
        extents = np.zeros((3, 2))
        extents[:, 1] = 1
        self.sampler = HaltonSampler(extents)

        batch1 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch1.shape,
            (self.batch_size, 3),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch1 > 0.5)[:, :2].all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )
        np.testing.assert_allclose(
            (batch1 > 0.5).all(axis=1).sum() / self.batch_size,
            0.125,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )

        batch2 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch2.shape,
            (self.batch_size, 3),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch2 > 0.5)[:, :2].all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )
        np.testing.assert_allclose(
            (batch2 > 0.5).all(axis=1).sum() / self.batch_size,
            0.125,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )

        self.assertFalse(
            np.array_equal(batch1, batch2),
            msg="Each call to sample should return different samples",
        )

    def test_sample_3d_scaled(self):
        extents = np.zeros((3, 2))
        extents[:, 1] = 10
        self.sampler = HaltonSampler(extents)

        batch1 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch1.shape,
            (self.batch_size, 3),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch1 > 5)[:, :2].all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )
        np.testing.assert_allclose(
            (batch1 > 5).all(axis=1).sum() / self.batch_size,
            0.125,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )

        batch2 = self.sampler.sample(self.batch_size)
        self.assertEqual(
            batch2.shape,
            (self.batch_size, 3),
            msg="sample should return the number of samples requested",
        )
        np.testing.assert_allclose(
            (batch2 > 5)[:, :2].all(axis=1).sum() / self.batch_size,
            0.25,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )
        np.testing.assert_allclose(
            (batch2 > 5).all(axis=1).sum() / self.batch_size,
            0.125,
            atol=0.005,
            err_msg="samples are not distributed uniformly across the extents",
        )

        self.assertFalse(
            np.array_equal(batch1, batch2),
            msg="Each call to sample should return different samples",
        )


class TestLatticeSampler(unittest.TestCase):
    def setUp(self):
        extents = np.zeros((2, 2))
        extents[:, 1] = 1
        self.sampler = LatticeSampler(extents)
        self.batch_size = 9

    def test_sample_2d(self):
        samples = self.sampler.sample(self.batch_size)
        self.assertEqual(
            samples.shape,
            (self.batch_size, 2),
            msg="sample should return the number of samples requested",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_halton_sampler", TestHaltonSampler)
    rosunit.unitrun("planning", "test_lattice_sampler", TestLatticeSampler)
