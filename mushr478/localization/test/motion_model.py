#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest
import time

from localization.motion_model import KinematicCarMotionModel


def particle_error(particles, target):
    """Compute the error between a set of particles and a target state.

    Args:
        particles: States (from particle filter)
        target: Ground truth state

    Returns:
        position_errors: Positional errors for each particle
        angular_errors: Angular errors for each particle
    """
    position_errors = np.linalg.norm(particles[:, :2] - target[:2], axis=1)
    particle_angles = particles[:, 2]
    target_angle = target[2]
    angular_errors = np.arctan2(
        np.sin(target_angle - particle_angles), np.cos(target_angle - particle_angles)
    )
    return position_errors, angular_errors


def error_within_epsilon(particles, target, epsilon):
    """Return whether the mean particle error is within epsilon of the target."""
    position_errors, angular_errors = particle_error(particles, target)
    return position_errors.mean() < epsilon and np.abs(angular_errors).mean() < epsilon


class TestMotionModel(unittest.TestCase):
    # Radius to count particles around ground truth
    epsilon = 0.1

    def setUp(self):
        self.num_particles = 100
        self.motion_model = KinematicCarMotionModel(
            0.33,
            vel_std=0.03,
            delta_std=0.1,
            x_std=0.01,
            y_std=0.01,
            theta_std=0.05,
        )
        self.particles = np.zeros((self.num_particles, 3))

    def test_compute_changes_is_vectorized(self):
        self.num_particles = 1000000
        self.particles = np.random.uniform(size=(self.num_particles, 3))
        controls = np.random.uniform([-1, -np.pi], [1, np.pi], (self.num_particles, 2))
        start = time.time()
        self.motion_model.compute_changes(self.particles, controls, 0.1)
        end = time.time()
        self.assertLess(
            end - start,
            2,
            msg="Your implementation should be able to "
            "compute changes for one million particles in less than 2 seconds",
        )

    def test_compute_changes_shape(self):
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1.0, 0]]), 0.123
        )
        self.assertEqual(
            (1, 3),
            changes.shape,
            msg="compute_changes should produce an Nx3 matrix, where N is the number of particles",
        )

    def test_compute_changes_result_type(self):
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1, 0]]), 0.123
        )
        # This isn't strictly necessary for the system to work, but the behavior if this case isn't handled
        # correctly can be surprising when manually testing, so we'll catch it early.
        self.assertEqual(
            changes.dtype,
            np.float,
            msg="compute_changes should return a floating point array, even when provided integer type arguments",
        )

    def test_compute_changes_timestep(self):
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1, 2]]), 0.0
        )
        np.testing.assert_allclose(
            changes, 0, err_msg="No update should be produced for a timestep of 0"
        )

    def test_compute_changes_linear_in_time(self):
        t = 0.1
        scale = 10
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1, 0]]), t
        )
        changes_t_scaled = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1, 0]]), t * scale
        )
        changes_scaled = scale * changes
        np.testing.assert_allclose(
            changes_t_scaled,
            changes_scaled,
            err_msg="A linear control's response is linear in time",
        )

    def test_compute_changes_position_invariant(self):
        controls = np.random.uniform([-1, -np.pi], [1, np.pi], (self.num_particles, 2))
        t = 0.1
        changes = self.motion_model.compute_changes(self.particles, controls, t)
        # Make particles with random coordinates but a 0rad heading
        self.particles = np.random.uniform(
            [-10, -10, 0], [10, 10, 0], self.particles.shape
        )
        changes_position_randomized = self.motion_model.compute_changes(
            self.particles, controls, t
        )
        np.testing.assert_allclose(
            changes,
            changes_position_randomized,
            err_msg="The input states' position components should not impact the calculation of changes",
        )

    def test_compute_changes_correct_for_large_angular_change(self):
        # Make car length a simple size
        self.motion_model = KinematicCarMotionModel(1.0)
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1.0, np.pi / 2]]), 1.0
        )
        np.testing.assert_almost_equal(
            changes[:, :2],
            np.array([[0, 0]]),
            err_msg="Turning right or left at a 90 degree angle should "
            "result in no positional change",
        )

    def test_compute_changes_linear_motion(self):
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0]]), np.array([[1.0, 0]]), 0.123
        )
        self.assertEqual(
            0.123,
            changes[0, 0],
            "A non-zero linear velocity and a zero steering velocity should result in linear motion",
        )

    def test_compute_changes_respects_current_heading(self):
        controls = np.array([[1, 0]])
        t = 1.0
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, np.pi]]), controls, t
        )
        np.testing.assert_almost_equal(
            changes[0, :],
            [-1.0, 0.0, 0.0],
            err_msg="Linear motion should point in the direction of the current heading",
        )
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, np.pi / 4]]), np.array([[1.0, 0.0]]), 2.0
        )
        # 45-45-90 right triangle with hypotenuse length 2.0
        np.testing.assert_almost_equal(
            changes[0, :],
            [np.sqrt(2), np.sqrt(2), 0],
            err_msg="Linear motion should point in the direction of the current heading",
        )

    def test_compute_changes_turn_respects_current_heading(self):
        self.num_particles = 1
        self.particles = np.array([[0.0, 0.0, np.pi / 2]])
        # Make the model deterministic
        linear_control = 1.0
        angular_control = np.pi / 4
        dt = 1.0
        changes = self.motion_model.compute_changes(
            self.particles, np.array([[linear_control, angular_control]]), dt
        )
        np.testing.assert_allclose(
            changes[0, :],
            [-0.66, 0.037, 3.03],
            atol=0.01,
            err_msg="Angular motion should take into account the direction of the current heading",
        )

    def test_compute_changes_steering_angle_threshold(self):
        threshold = 1e-2
        changes = self.motion_model.compute_changes(
            np.array([[0, 0, 0], [0, 0, 0]]),
            np.array([[1.0, 5e-3], [1.0, -5e-3]]),
            1.0,
            delta_threshold=threshold,
        )
        np.testing.assert_equal(
            changes[0, :],
            [1, 0, 0],
            "A near-zero steering angle should be treated as a zero steering angle, "
            "with the result being linear motion",
        )
        np.testing.assert_equal(
            changes[1, :],
            [1, 0, 0],
            "A negative, near-zero steering angle should be treated as a zero steering angle, "
            "with the result being linear motion",
        )

    def test_apply_is_vectorized(self):
        self.num_particles = 1000000
        self.particles = np.random.uniform(size=(self.num_particles, 3))
        start = time.time()
        self.motion_model.apply_motion_model(self.particles, -1, np.pi / 2, 0.1)
        end = time.time()
        self.assertLess(
            end - start,
            2,
            msg="Your implementation should be able to update "
            "one million particles in less than 2 seconds",
        )

    def test_apply_modifies_particles(self):
        particles = np.random.uniform(size=self.particles.shape)
        self.particles = particles.copy()
        self.motion_model.apply_motion_model(self.particles, 1.0, np.pi / 2, 1.0)
        self.assertFalse(
            np.allclose(particles, self.particles),
            msg="Motion model should modify particles in-place",
        )

    def test_apply_normalizes_theta(self):
        # Make deterministic
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0, vel_std=0, x_std=0, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 1.0, np.pi / 2, 1.0)
        thetas = self.particles[:, 2]
        self.assertTrue(
            np.all((-np.pi < thetas) & (thetas <= np.pi)),
            msg="Motion model should enforce that resulting particle angles are in (-pi, pi]",
        )

        # Check low end is  open
        self.particles = np.array([[0, 0, -np.pi], [0, 0, -3 * np.pi], [0, 0, np.pi]])
        self.motion_model.apply_motion_model(self.particles, 0.0, 0.0, 1.0)
        thetas = self.particles[:, 2]
        self.assertTrue(
            np.all((-np.pi < thetas) & (thetas <= np.pi)),
            msg="Motion model should enforce that resulting particle angles are in (-pi, pi]",
        )

        # Check top end is closed
        self.particles = np.array([[0, 0, np.pi], [0, 0, 3 * np.pi]])
        self.motion_model.apply_motion_model(self.particles, 0.0, 0.0, 1.0)
        thetas = self.particles[:, 2]
        self.assertTrue(
            np.all((-np.pi < thetas) & (thetas <= np.pi)),
            msg="Motion model should enforce that resulting particle angles are in (-pi, pi]",
        )

    def test_apply_model_noise(self):
        # We need a lot of particles to get a good estimate of the variance
        self.num_particles = 100000

        self.particles = np.zeros((self.num_particles, 3))
        x_std = 1.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0, vel_std=0, x_std=x_std, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 0, 0, 0.1)
        np.testing.assert_allclose(
            self.particles[:, 0].std(),
            x_std,
            atol=0.01,
            err_msg="The x_std parameter should control the scale of the variance "
            "in the updated x positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 1],
            0,
            err_msg="The y_std parameter should control the scale of the variance "
            "in the updated y positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 2],
            0,
            err_msg="The theta_std parameter should control the scale of the variance "
            "in the updated headings",
        )

        self.particles = np.zeros((self.num_particles, 3))
        y_std = 1.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0, vel_std=0, x_std=0, y_std=y_std, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 0, 0, 0.1)
        np.testing.assert_allclose(
            self.particles[:, 0],
            0,
            err_msg="The x_std parameter should control the scale of the variance "
            "in the updated x positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 1].std(),
            y_std,
            atol=0.01,
            err_msg="The y_std parameter should control the scale of the varience "
            "in the updated y positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 2],
            0,
            err_msg="The theta_std parameter should control the scale of the variance "
            "in the updated headings",
        )

        self.particles = np.zeros((self.num_particles, 3))
        theta_std = 1.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0, vel_std=0, x_std=0, y_std=0, theta_std=theta_std
        )
        self.motion_model.apply_motion_model(self.particles, 0, 0, 0.1)

        np.testing.assert_allclose(
            self.particles[:, 0],
            0,
            err_msg="The x_std parameter should control the scale of the variance "
            "in the updated x positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 1],
            0,
            err_msg="The y_std parameter should control the scale of the variance "
            "in the updated y positions",
        )
        np.testing.assert_allclose(
            self.particles[:, 2].std(),
            theta_std,
            atol=0.01,
            err_msg="The theta_std parameter should control the scale of the varience "
            "in the updated headings",
        )

    def test_apply_action_noise(self):
        # We need a lot of particles to get a good estimate of the variance
        self.num_particles = 100000

        self.particles = np.zeros((self.num_particles, 3))
        delta_std = 1.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=delta_std, vel_std=0, x_std=0, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 0, 0, 0.1)
        np.testing.assert_allclose(
            self.particles,
            0,
            err_msg="Angular control noise should have no impact if there is no control",
        )

        self.particles = np.zeros((self.num_particles, 3))
        vel_std = 1.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0, vel_std=vel_std, x_std=0, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 0, 0, 1.0)
        np.testing.assert_allclose(
            self.particles[:, 0].std(),
            vel_std,
            rtol=0.01,
            err_msg="Linear velocity control noise should cause changes "
            "in particle position even when no control is applied",
        )

        self.particles = np.zeros((self.num_particles, 3))
        # Note that this won't hold for higher values of delta_std, I think
        # because the mass of the tails will wrap around [-pi, pi] and distort
        delta_std = 0.1
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=delta_std, vel_std=0, x_std=0, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 1.0, 0, 1.0)
        angles = self.particles[:, 2]
        # Might be worth looking at circular variance measure in the future:
        # S = np.sin(angles).mean()
        # C = np.cos(angles).mean()
        # circvar = 1 - np.hypot(S, C)
        np.testing.assert_allclose(
            angles.std(),
            delta_std,
            rtol=0.02,
            err_msg="delta_std parameter should control scale of variance in heading "
            "changes for a linear control",
        )

    def test_apply_action_noise_impact_grows_with_time(self):
        self.num_particles = 100000
        self.particles = np.zeros((self.num_particles, 3))
        delta_std = 0.1
        time_scale = 3.0
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=delta_std, vel_std=0, x_std=0, y_std=0, theta_std=0
        )
        self.motion_model.apply_motion_model(self.particles, 1.0, 0, 1.0 * time_scale)
        np.testing.assert_allclose(
            self.particles[:, 2].std(),
            delta_std * time_scale,
            atol=0.01,
            err_msg="The scale of the variance in heading should "
            "be proportional to the timestep",
        )

    def test_apply_motion_model_fine(self):
        self.num_particles = 100000
        self.particles = np.zeros((self.num_particles, 3))
        self.motion_model.apply_motion_model(self.particles, 1.01, 0.3, 0.1)
        np.testing.assert_allclose(
            self.particles.mean(axis=0),
            np.array([0.1, 0.0, 0.1]),
            atol=0.01,
            err_msg="The mean particle didn't match our expectations given the controls",
        )
        self.assertTrue(
            error_within_epsilon(
                self.particles, np.array([0.1, 0.0, 0.1]), self.epsilon
            ),
            msg="The particle dispersion didn't match our expectations given the controls",
        )

    def test_apply_motion_model_coarse(self):
        self.num_particles = 100000
        self.particles = np.zeros((self.num_particles, 3))
        self.motion_model.apply_motion_model(self.particles, 3.04, 0.4, 0.5)
        # We loosen epsilon because the timestep is large
        np.testing.assert_allclose(
            self.particles.mean(axis=0),
            np.array([0.71, 1.0, 1.82]),
            atol=0.01,
            err_msg="The mean particle didn't match our expectations given the controls",
        )
        self.assertTrue(
            error_within_epsilon(
                self.particles, np.array([0.71, 1.0, 1.82]), self.epsilon * 5
            ),
            msg="The particle dispersion didn't match our expectations given the controls",
        )

    def test_apply_motion_model_turn(self):
        self.num_particles = 1
        self.particles = np.zeros((self.num_particles, 3))
        # Make the model deterministic
        self.motion_model = KinematicCarMotionModel(
            1.0, delta_std=0.0, vel_std=0, x_std=0, y_std=0, theta_std=0
        )
        linear_control = 1.0
        angular_control = 0.3
        dt = 0.01
        path = []
        for i in range(10):
            self.motion_model.apply_motion_model(
                self.particles, linear_control, angular_control, dt
            )
            path.append(self.particles[0].copy())
        path = np.array(path)
        # First diff is displacement. Displacement over time is velocity
        velocities = np.diff(path, axis=0) / dt
        accelerations = np.diff(velocities, axis=0) / dt
        # Magnitude of the vector formed by the individual x and y acceleration components
        linear_velocity = np.linalg.norm(velocities[:, [0, 1]], axis=1)
        np.testing.assert_allclose(
            linear_velocity,
            linear_control,
            atol=0.001,
            err_msg="The linear velocity of the trajectory should remain constant",
        )
        # angular velocity ~= lin * angcontrol, only for small angular velocities
        np.testing.assert_allclose(
            velocities[:, 2],
            angular_control * linear_control,
            atol=0.05 * np.abs(angular_control),
            err_msg="The angular velocity of the trajectory should remain constant",
        )
        np.testing.assert_almost_equal(
            accelerations[:, 2],
            0,
            err_msg="There should be no angular acceleration for a constant velocity angular control",
        )


if __name__ == "__main__":
    rosunit.unitrun("localization", "test_motion_model", TestMotionModel)
