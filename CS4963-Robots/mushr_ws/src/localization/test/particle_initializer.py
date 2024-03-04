#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest
from geometry_msgs.msg import Pose

from cse478 import utils
from localization.particle_filter import ParticleInitializer


class TestParticleInitializer(unittest.TestCase):
    def setUp(self):
        self.num_particles = 50
        self.particle_initializer = ParticleInitializer(
            x_std=0.05,
            y_std=0.05,
            theta_std=0.1,
        )

        self.initial_pose = np.array([-0.7, 1.3, 0.6])
        x, y, theta = self.initial_pose
        self.initial_pose_msg = Pose()
        self.initial_pose_msg.position.x = x
        self.initial_pose_msg.position.y = y
        self.initial_pose_msg.orientation = utils.angle_to_quaternion(theta)

    def test_initializer_inplace(self):
        particles = np.zeros((self.num_particles, 3))
        weights = np.arange(self.num_particles, dtype=float)
        weights[:] /= sum(weights)
        prev_particles = particles.copy()
        prev_weights = weights.copy()
        self.particle_initializer.reset_click_pose(
            self.initial_pose_msg, particles, weights
        )
        self.assertFalse(
            np.allclose(prev_particles, particles),
            msg="Initializer should modify particles in-place",
        )
        self.assertFalse(
            np.allclose(prev_weights, weights),
            msg="Initializer should modify weights in-place",
        )

    def test_initializer_particles_are_centered(self):
        self.num_particles = 1000000
        particles = np.zeros((self.num_particles, 3))
        weights = np.zeros(self.num_particles)
        self.particle_initializer.reset_click_pose(
            self.initial_pose_msg, particles, weights
        )
        weighted_avg_particles = np.einsum("ij,i->j", particles, weights)
        np.testing.assert_equal(
            weights,
            np.full_like(weights, 1.0 / self.num_particles),
            err_msg="Initializer should set the weights to be uniform",
        )
        self.assertTrue(
            np.allclose(weighted_avg_particles, self.initial_pose, atol=1e-3),
            msg="Initializer should produce a distribution of particles centered around the given initial pose",
        )


if __name__ == "__main__":
    rosunit.unitrun(
        "localization", "test_particle_initializer", TestParticleInitializer
    )
