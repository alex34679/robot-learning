#!/usr/bin/env python
from __future__ import division
from threading import Lock
import numpy as np
import range_libc
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg


class SingleBeamSensorModel:
    """The single laser beam sensor model."""

    def __init__(self, **kwargs):
        """Initialize the single-beam sensor model.

        Args:
            **kwargs (object): any number of optional keyword arguments:
                hit_std (float): Noise value for hit reading
                z_hit (float): Weight for hit reading
                z_short (float): Weight for short reading
                z_max (float): Weight for max reading
                z_rand (float): Weight for random reading
        """
        defaults = {
            "hit_std": 1.0,
            "z_hit": 0.5,
            "z_short": 0.05,
            "z_max": 0.05,
            "z_rand": 0.5,
        }
        if not set(kwargs).issubset(set(defaults)):
            raise ValueError("Invalid keyword argument provided")
        # These next two lines set the instance attributes from the defaults and
        # kwargs dictionaries. For example, the key "hit_std" becomes the
        # instance attribute self.hit_std.
        self.__dict__.update(defaults)
        self.__dict__.update(kwargs)

        if (
            self.z_short == 0
            and self.z_max == 0
            and self.z_rand == 0
            and self.z_hit == 0
        ):
            raise ValueError(
                "The model is undefined for the given parameters."
                "You must provide a non-0 value for at least one portion of the model."
            )

    def precompute_sensor_model(self, max_r):
        """Precompute sensor model probabilities for all pairs of simulated and observed
        distance measurements.

        The probabilities are stored in a 2D array, where the element at index
        (r, d) is the probability of observing measurement r when the simulated
        (expected) measurement is d.

        You will need to normalize the table to ensure probabilities sum to 1, i.e.
        sum P(r | d) over all r should be 1, for all d.

        Args:
            max_r (int): The maximum range (in pixels)

        Returns:
            prob_table: np.array of shape (max_r+1, max_r+1) containing
                the sensor probabilities P(r | d), or P(z_t^k | z_t^k*) from lecture.
        """
        table_width = int(max_r) + 1
        prob_table = np.zeros((table_width, table_width))

        # Get matrices of the same shape as prob_table,
        # where each entry holds the real measurement r (obs_r)
        # or the simulated (expected) measurement d (sim_r).
        obs_r, sim_r = np.mgrid[0:table_width, 0:table_width]

        # Use obs_r and sim_r to vectorize the sensor model precomputation.
        diff = sim_r - obs_r
        # BEGIN QUESTION 2.1
        "*** REPLACE THIS LINE ***"
        # END QUESTION 2.1

        return prob_table


class LaserScanSensorModelROS:
    """A ROS subscriber that reweights particles according to the sensor model.

    This applies the sensor model to the particles whenever it receives a
    message from the laser scan topic.

    These implementation details can be safely ignored, although you're welcome
    to continue reading to better understand how the entire state estimation
    pipeline is connected.
    """

    def __init__(
        self, particles, weights, sensor_params=None, state_lock=None, **kwargs
    ):
        """Initialize the laser scan sensor model ROS subscriber.

        Args:
            particles: the particles to update
            weights: the weights to update
            sensor_params: a dictionary of parameters for the sensor model
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
            **kwargs: Required unless marked otherwise
                laser_ray_step (int): Step for downsampling laser scans
                exclude_max_range_rays (bool): Whether to exclude rays that are
                    beyond the max range
                map_msg (nav_msgs.msg.MapMetaData): Map metadata to use
                car_length (float): the length of the car
                theta_discretization (int): Discretization of scanning angle. Optional
                inv_squash_factor (float): Make the weight distribution less peaked

        """
        if not particles.shape[0] == weights.shape[0]:
            raise ValueError("Must have same number of particles and weights")
        self.particles = particles
        self.weights = weights
        required_keyword_args = {
            "laser_ray_step",
            "exclude_max_range_rays",
            "max_range_meters",
            "map_msg",
            "car_length",
        }
        if not required_keyword_args.issubset(set(kwargs)):
            raise ValueError("Missing required keyword argument")
        defaults = {
            "theta_discretization": 112,
            "inv_squash_factor": 0.2,
        }
        # These next two lines set the instance attributes from the defaults and
        # kwargs dictionaries.
        self.__dict__.update(defaults)
        self.__dict__.update(kwargs)

        self.half_car_length = self.car_length / 2
        self.state_lock = state_lock or Lock()
        sensor_params = {} if sensor_params is None else sensor_params
        sensor_model = SingleBeamSensorModel(**sensor_params)

        # Create map
        o_map = range_libc.PyOMap(self.map_msg)
        # the max range in pixels of the laser
        max_range_px = int(self.max_range_meters / self.map_msg.info.resolution)
        self.range_method = range_libc.PyCDDTCast(
            o_map, max_range_px, self.theta_discretization
        )
        # Alternative range method that can be used for ray casting
        # self.range_method = range_libc.PyRayMarchingGPU(o_map, max_range_px)
        self.range_method.set_sensor_model(
            sensor_model.precompute_sensor_model(max_range_px)
        )

        self.queries = None
        self.ranges = None
        self.laser_angles = None  # The angles of each ray
        self.do_resample = False  # Set for outside code to know when to resample

        self.initialized = False
        # Subscribe to laser scans
        self.laser_sub = rospy.Subscriber(
            "scan", numpy_msg(LaserScan), self.lidar_callback, queue_size=1
        )

    def start(self):
        self.initialized = True

    def lidar_callback(self, msg):
        """Apply the sensor model to downsampled laser measurements.

        Args:
            msg: a sensor_msgs/LaserScan message
        """
        # Initialize angles
        if self.laser_angles is None:
            self.laser_angles = np.linspace(
                msg.angle_min, msg.angle_max, len(msg.ranges)
            )

        if not self.initialized:
            return

        ranges, angles = self.downsample(msg.ranges)

        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            self.apply_sensor_model(ranges, angles)
            self.weights /= np.sum(self.weights)

        self.last_laser = msg
        self.do_resample = True

    def apply_sensor_model(self, obs_ranges, obs_angles):
        """Apply the sensor model to self.weights based on the observed laser scan.

        Args:
            obs_ranges: observed distance measurements
            obs_angles: observed laser beam angles
        """
        num_rays = obs_angles.shape[0]

        # Initialize buffer
        num_particles = self.particles.shape[0]
        if self.queries is None:
            self.queries = np.zeros((num_particles, 3), dtype=np.float32)
        if self.ranges is None:
            self.ranges = np.zeros(num_rays * num_particles, dtype=np.float32)

        self.queries[:, :] = self.particles[:, :]
        self.queries[:, 0] += self.half_car_length * np.cos(self.queries[:, 2])
        self.queries[:, 1] += self.half_car_length * np.sin(self.queries[:, 2])

        # Raycasting to get expected measurements
        self.range_method.calc_range_repeat_angles(
            self.queries, obs_angles, self.ranges
        )

        # Evaluate the sensor model
        self.range_method.eval_sensor_model(
            obs_ranges, self.ranges, self.weights, num_rays, self.particles.shape[0]
        )

        # Squash weights to prevent too much peakiness
        np.power(self.weights, self.inv_squash_factor, self.weights)

    def downsample(self, ranges):
        """Downsample the laser rays.

        Args:
            ranges: all observed distance measurements

        Returns:
            ranges: downsampled observed distance measurements
            angles: downsampled observed laser beam angles
        """
        if not self.exclude_max_range_rays:
            angles = np.copy(self.laser_angles[0 :: self.laser_ray_step]).astype(
                np.float32
            )
            sampled = ranges[:: self.laser_ray_step].copy()
            sampled[np.isnan(sampled)] = self.max_range_meters
            sampled[np.abs(sampled) < 1e-3] = self.max_range_meters
            return sampled, angles

        # We're trying to avoid copying the ranges here, so
        # we silence errors from comparison to NaN instead of overriding these values
        with np.errstate(invalid="ignore"):
            valid_indices = np.logical_and(
                ~np.isnan(ranges), ranges > 0.01, ranges < self.max_range_meters
            )
        filtered_ranges = ranges[valid_indices]
        filtered_angles = self.laser_angles[valid_indices]

        # Grab expected number of rays
        ray_count = int(self.laser_angles.shape[0] / self.laser_ray_step)
        num_valid = filtered_angles.shape[0]
        sample_indices = np.arange(0, num_valid, float(num_valid) / ray_count).astype(
            np.int
        )
        angles = np.copy(filtered_angles[sample_indices]).astype(np.float32)
        ranges = np.copy(filtered_ranges[sample_indices]).astype(np.float32)
        return ranges, angles
