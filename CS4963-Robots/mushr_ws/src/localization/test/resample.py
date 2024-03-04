#!/usr/bin/env python
from __future__ import division
from collections import Counter
import numpy as np
import rosunit
import unittest

from localization.resampler import LowVarianceSampler


def naive_sample(weights, target_population_size):
    cum_weights = np.cumsum(weights)
    sampled = np.random.random_sample(size=target_population_size)
    return np.searchsorted(cum_weights, sampled, side="left")


class TestResample(unittest.TestCase):
    def test_resampler_inplace(self):
        n_particles = 100  # number of particles

        particles = np.arange(n_particles)
        particles = particles[..., np.newaxis]
        weights = np.arange(n_particles, dtype=np.float)
        weights /= np.sum(weights)
        prev_particles = particles.copy()
        prev_weights = weights.copy()
        rs = LowVarianceSampler(particles, weights)
        rs.resample()
        self.assertTrue(
            np.allclose(rs.particles, particles),
            msg="Resampler should modify particles in-place",
        )
        self.assertFalse(
            np.allclose(prev_particles, particles),
            msg="Resampler should modify particles in-place",
        )
        self.assertTrue(
            np.allclose(rs.weights, weights),
            msg="Resampler should modify weights in-place",
        )
        self.assertFalse(
            np.allclose(prev_weights, weights),
            msg="Resampler should modify weights in-place",
        )

    def test_resampler_is_fair(self):
        n_particles = 100  # number of particles
        k_val = 50  # number of particles that have non-zero weight
        trials = 10  # number of resamplings to do

        # Count how often each particle has been sampled across trials
        histogram = Counter()
        for i in range(trials):
            # Create a set of particles with weights proportional to their index
            # Particles with index greater than k_val are given zero weight
            particles = np.arange(n_particles)
            particles = particles[..., np.newaxis]
            weights = np.arange(n_particles, dtype=np.float)
            weights[k_val:] = 0.0
            weights /= np.sum(weights)

            rs = LowVarianceSampler(particles, weights)
            rs.resample()

            # Add the number of times each particle was sampled
            histogram.update(particles[:, 0])

        count_pairs = np.array(list(histogram.items()))
        counts = np.zeros((n_particles), dtype=int)
        counts[count_pairs[:, 0]] = count_pairs[:, 1]
        self.assertFalse(
            np.any(counts[k_val:] > 0),
            msg="Particles with 0 weight should never be sampled",
        )
        for i in range(4):
            # Compare mean number of samples for first half vs second half of particles.
            # Move up the the start index for the total range we're looking at
            # by powers of 2 (1/2, 1/4, 1/8 ...) so we're focusing on progressively
            # smaller slices of the end of the non-zero region.
            start = int(round(k_val * (1 - (2 ** -i))))
            end = k_val
            middle = (start + end) // 2
            self.assertLess(
                counts[start:middle].mean(),
                counts[middle:end].mean(),
                msg="Particles with less weight should be sampled less than those with more weight",
            )

    def test_resampler_is_complete(self):
        n_particles = 100  # number of particles
        trials = 100

        for i in range(trials):
            histogram = Counter()

            # Create a set of particles with uniform weights
            particles = np.arange(n_particles)
            particles = particles[..., np.newaxis]
            weights = np.full([n_particles], 1.0 / n_particles)

            rs = LowVarianceSampler(particles, weights)
            rs.resample()

            # Add the number of times each particle was sampled
            histogram.update(particles[:, 0])
            count_pairs = np.array(list(histogram.items()), dtype=int)
            counts = np.zeros((n_particles))
            counts[count_pairs[:, 0]] = count_pairs[:, 1]
            self.assertFalse(
                np.any(counts == 0),
                msg="All particles should have been sampled at least once",
            )

    def test_resampler_resets_weights(self):
        n_particles = 100  # number of particles

        # Create a set of particles with uniform weights
        particles = np.arange(n_particles)
        particles = particles[..., np.newaxis]
        weights = np.random.uniform(0, 1, n_particles)
        weights /= weights.sum()

        rs = LowVarianceSampler(particles, weights)
        rs.resample()

        np.testing.assert_equal(
            weights,
            np.full(n_particles, 1.0 / n_particles),
            err_msg="The resampler should set the weights to be uniform",
        )


if __name__ == "__main__":
    rosunit.unitrun("localization", "test_resample", TestResample)
