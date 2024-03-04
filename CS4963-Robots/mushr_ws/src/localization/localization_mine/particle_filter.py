#!/usr/bin/env python
from __future__ import division
from threading import Lock, Thread

import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf_conversions import transformations
from nav_msgs.srv import GetMap

from cse478 import utils
from localization_mine.motion_model import KinematicCarMotionModelROS
from localization_mine.sensor_model import LaserScanSensorModelROS
from localization_mine.resampler import LowVarianceSampler


class ParticleInitializer:
    def __init__(self, x_std=0.1, y_std=0.1, theta_std=0.2):
        self.x_std = x_std
        self.y_std = y_std
        self.theta_std = theta_std

    def reset_click_pose(self, msg: Pose, particles, weights):
        """Initialize the particles and weights in-place.

        The particles should be sampled from a Gaussian distribution around the
        initial pose. Remember to write vectorized code.

        Args:
            msg: a geometry_msgs/Pose message with the initial pose
            particles: the particles to initialize
            weights: the weights associated with particles
        """
        n_particles = particles.shape[0]
        # Hint: use utils.quaternion_to_angle to compute the orientation theta.
        # BEGIN QUESTION 3.1
        # Read Pose
        theta = utils.quaternion_to_angle(msg.orientation)

        particles[:, 0] = np.random.normal(msg.position.x, weights[0], n_particles)
        particles[:, 1] = np.random.normal(msg.position.y, weights[1], n_particles)
        particles[:, 2] = np.random.normal(theta, weights[2], n_particles)

        weights.fill(1/n_particles)
        # END QUESTION 3.1


class ParticleFilter:
    """The particle filtering state estimation algorithm.

    These implementation details can be safely ignored, although you're welcome
    to continue reading to better understand how the entire state estimation
    pipeline is connected.
    """

    def __init__(self, motion_params=None, sensor_params=None, **kwargs):
        """Initialize the particle filter.

        Args:
            motion_params: a dictionary of parameters for the motion model
            sensor_params: a dictionary of parameters for the sensor model
            kwargs: All of the following
                publish_tf (str): To publish the tf or not. Should be false in sim, true on real robot
                n_particles (int): The number of particles
                n_viz_particles (int): The number of particles to visualize
                car_length (float):
                laser_ray_step (int):
                exclude_max_range_rays (bool):
                max_range_meters (float):
                speed_to_erpm_offset (float):
                speed_to_erpm_gain (float):
                steering_to_servo_offset (float):
                steering_to_servo_gain (float):
        """
        required_keyword_args = {
            "publish_tf",
            "tf_prefix",
            "n_particles",
            "n_viz_particles",
            "car_length",
            # Specific to sensor model:
            "laser_ray_step",
            "exclude_max_range_rays",
            "max_range_meters",
            # Specific to motion model:
            "speed_to_erpm_offset",
            "speed_to_erpm_gain",
            "steering_to_servo_offset",
            "steering_to_servo_gain",
        }
        if not required_keyword_args.issubset(set(kwargs)):
            raise ValueError("A required keyword argument is missing")
        self.__dict__.update(kwargs)
        self.n_particles = self.n_particles
        # Cached list of particle indices
        self.particle_indices = np.arange(self.n_particles)
        # Numpy matrix of dimension N_PARTICLES x 3
        self.particles = np.zeros((self.n_particles, 3))
        # Numpy matrix containing weight for each particle
        self.weights = np.full(self.n_particles, 1.0 / self.n_particles, dtype=float)
        self.particle_initializer = ParticleInitializer()

        self.state_lock = Lock()
        self._tf_buffer = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self._tf_buffer)

        if self.use_map_topic:
            # Save info about map
            rospy.loginfo("Waiting for map")
            map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)
        else:
            mapsrv = rospy.ServiceProxy("/static_map", GetMap)
            map_msg = mapsrv().map

        self.map_info = map_msg.info

        # Permissible region
        array_255 = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width)
        )
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        self.permissible_x, self.permissible_y = np.where(self.permissible_region == 1)

        # Publishes the expected pose
        self.pose_pub = rospy.Publisher("~inferred_pose", PoseStamped, queue_size=1)
        # Publishes a subsample of the particles
        self.particle_pub = rospy.Publisher("~particles", PoseArray, queue_size=1)
        # Publishes the path of the car
        self.pub_odom = rospy.Publisher("~odom", Odometry, queue_size=1)


        # Outside caller can use this resampler to resample particles
        self.resampler = LowVarianceSampler(
            self.particles, self.weights, self.state_lock
        )

        # Initialize the motion model subscriber
        motion_model_worker_params = {
            key: kwargs[key]
            for key in (
                "car_length",
                "speed_to_erpm_offset",
                "speed_to_erpm_gain",
                "steering_to_servo_offset",
                "steering_to_servo_gain",
            )
        }
        self.motion_model = KinematicCarMotionModelROS(
            self.particles,
            noise_params=motion_params,
            state_lock=self.state_lock,
            **motion_model_worker_params
        )

        # Initialize the sensor model subscriber
        sensor_model_worker_params = {
            key: kwargs[key]
            for key in (
                "laser_ray_step",
                "exclude_max_range_rays",
                "max_range_meters",
                "car_length",
            )
        }
        sensor_model_worker_params["map_msg"] = map_msg
        self.sensor_model = LaserScanSensorModelROS(
            self.particles,
            self.weights,
            sensor_params=sensor_params,
            state_lock=self.state_lock,
            **sensor_model_worker_params
        )

        self.click_sub = rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self.clicked_pose_cb,
            queue_size=1,
        )

        rospy.loginfo("Waiting for laser scan")
        rospy.wait_for_message("scan", LaserScan)

        self._viz_timer = Thread(target=self.visualize)
        self._viz_timer.start()

        if self.publish_tf:
            rospy.loginfo("Starting TF broadcaster")
            self._tf_pub_timer = Thread(target=self._publish_tf)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self._tf_pub_timer.start()

        rospy.loginfo("Startup complete. Waiting for initial pose estimate")

    def set_pose(self, x, y, theta):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation = utils.angle_to_quaternion(theta)
        self.set_pose_msg(pose)

    def set_pose_msg(self, pose_msg):
        with self.state_lock:
            rospy.loginfo("Setting pose to {}".format(pose_msg))
            self.particle_initializer.reset_click_pose(
                pose_msg, self.particles, self.weights
            )
            # They may be waiting on an initial estimate
            self.sensor_model.start()
            self.motion_model.start()

    def spin(self):
        # We're limited by rate of the sensor and the speed data.
        # Sensor is usually slowest at around 10Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():  # Keep going until we kill it
            rate.sleep()
            # Check if the sensor model says it's time to resample
            if self.sensor_model.do_resample:
                # Reset so that we don't keep resampling
                self.sensor_model.do_resample = False
                self.resampler.resample()

    def expected_pose(self):
        """Compute the expected state, given current particles and weights.

        Use cosine and sine averaging to more accurately compute average theta.

        To get one combined value, use the dot product of position and weight vectors
        https://en.wikipedia.org/wiki/Mean_of_circular_quantities

        Returns:
            np.array of the expected state
        """
        cosines = np.cos(self.particles[:, 2])
        sines = np.sin(self.particles[:, 2])
        theta = np.arctan2(np.dot(sines, self.weights), np.dot(cosines, self.weights))
        position = np.dot(self.particles[:, 0:2].transpose(), self.weights)

        # Offset to car's center of mass
        position[0] += (self.car_length / 2) * np.cos(theta)
        position[1] += (self.car_length / 2) * np.sin(theta)
        return np.array((position[0], position[1], theta), dtype=np.float)

    def clicked_pose_cb(self, msg):
        """Reinitialize particles and weights according to the received initial pose.


        Args:
            msg: a geometry_msgs/Pose message with the initial pose
        """
        self.set_pose_msg(msg.pose.pose)

    def visualize(self):
        """Visualize the current state of the particle filter.

        1. Publishes a tf between the map and the laser. Necessary for
           visualizing the laser scan in the map.
        2. Publishes a PoseStamped message with the expected pose of the car.
        3. Publishes a subsample of the particles, where particles with higher
           weights are more likely to be sampled.
        """
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            inferred_pose = self._infer_pose()
            if inferred_pose is None:
                rate.sleep()
                continue

            if self.pose_pub.get_num_connections() > 0:
                self.pose_pub.publish(inferred_pose)

            if self.pub_odom.get_num_connections() > 0:
                odom = Odometry()
                odom.header = inferred_pose.header
                odom.pose.pose = inferred_pose.pose
                self.pub_odom.publish(odom)

            if self.particle_pub.get_num_connections() > 0:
                with self.state_lock:
                    if self.particles.shape[0] > self.n_viz_particles:
                        # randomly downsample particles
                        proposal_indices = np.random.choice(
                            self.particle_indices, self.n_viz_particles, p=self.weights
                        )
                        self.publish_particles(self.particles[proposal_indices, :])
                    else:
                        self.publish_particles(self.particles)
            rate.sleep()

    def publish_particles(self, particles):
        """Publishes a pose array of particles."""
        pa = PoseArray()
        pa.header = utils.make_header("map")
        pa.poses = utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def _infer_pose(self):
        """Return a geometry_msgs/PoseStamped message with the current expected pose."""
        with self.state_lock:
            inferred_pose = self.expected_pose()
        ps = None
        if isinstance(inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = utils.make_header("map")
            # This estimate is as current as the particles
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = inferred_pose[0]
            ps.pose.position.y = inferred_pose[1]
            ps.pose.orientation = utils.angle_to_quaternion(inferred_pose[2])
        return ps

    def _publish_tf(self):
        """Publish a transform between map and odom frames."""
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.sensor_model.initialized or not self.motion_model.initialized:
                continue
            pose = self._infer_pose()
            if pose is None:
                continue
            pa = utils.pose_to_particle(pose.pose)

            # Future date transform so consumers know that the prediction is
            # good for a little while
            pose.header.stamp += rospy.Duration(0.1)

            try:
                # Look up the transform between laser and odom
                odom_to_laser = self._tf_buffer.lookup_transform(
                    self.tf_prefix + "laser_link", self.tf_prefix + "odom", rospy.Time(0)
                )
            except tf2_ros.LookupException as e:  # Will occur if odom frame does not exist
                rospy.logerr(str(e))
                rospy.logerr("failed to find odom")
                continue

            delta_off, delta_rot = utils.transform_stamped_to_pq(odom_to_laser)

            # Transform offset to be w.r.t the map
            off_x = delta_off[0] * np.cos(pa[2]) - delta_off[1] * np.sin(pa[2])
            off_y = delta_off[0] * np.sin(pa[2]) + delta_off[1] * np.cos(pa[2])

            # Create the transform message
            transform = TransformStamped()
            transform.header.stamp = pose.header.stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = self.tf_prefix + "odom"
            transform.transform.translation.x = pa[0] + off_x
            transform.transform.translation.y = pa[1] + off_y
            transform.transform.translation.z = 0
            nq = transformations.quaternion_from_euler(
                0, 0, pa[2] + transformations.euler_from_quaternion(delta_rot)[2]
            )
            transform.transform.rotation.x = nq[0]
            transform.transform.rotation.y = nq[1]
            transform.transform.rotation.z = nq[2]
            transform.transform.rotation.w = nq[3]

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)

