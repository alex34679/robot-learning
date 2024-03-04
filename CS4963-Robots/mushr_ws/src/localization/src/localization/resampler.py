#!/usr/bin/env python
from __future__ import division

from threading import Lock

import numpy as np


class LowVarianceSampler:
    """Low-variance particle sampler."""

    def __init__(self, particles, weights, state_lock=None):
        """Initialize the particle sampler.

        Args:
            particles: the particles to update
            weights: the weights to update
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
        """
        self.particles = particles
        self.weights = weights
        self.state_lock = state_lock or Lock()
        self.n_particles = particles.shape[0]

        # You may want to cache some intermediate variables here for efficiency
        # BEGIN SOLUTION NO PROMPT
        self.step_array = np.arange(self.n_particles, dtype=np.float32)
        self.step_array /= self.n_particles
        self.indices = np.zeros(self.n_particles, dtype=int)
        # END SOLUTION

    def resample(self):
        """Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        """
        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # BEGIN SOLUTION "QUESTION 3.2"
            # Choose an initial value from half open interval [0, 1/M)
            initval = np.random.uniform(0, self.n_particles ** -1)

            # Get the bin partitions
            bin_parts = initval + self.step_array
            # The last and highest value in bin_parts = r + 1 - (1/M)
            # where r in [0, 1/M)

            # Get the cumulative sum of the weights. Notice how the
            # the space between entries is an interval proportional to
            # the weight of the particle at that index
            cum_weights = np.cumsum(self.weights)

            # Concise O(M log M) implementation
            self.indices = np.searchsorted(cum_weights, bin_parts, side="left")

            assert np.all(self.indices < self.n_particles)
            self.particles[:] = self.particles[self.indices, :]

            # Uniformly weight new particles
            self.weights.fill(1.0 / self.n_particles)
            # END SOLUTION
