from __future__ import division
import threading

import matplotlib.cm as cm
import matplotlib.colors as colors
import numpy as np
import rospy
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped
from control.controller import time_parameterize_ramp_up_ramp_down
from control.mpc import ModelPredictiveController
from control.pid import PIDController
from control.purepursuit import PurePursuitController
from control.srv import FollowPath
from cse478 import utils
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import Float64
from std_msgs.msg import Header
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

controllers = {
    "pid": PIDController,
    "pp": PurePursuitController,
    "mpc": ModelPredictiveController,
}


class ControlROS:
    def __init__(self, controller, tf_prefix="", transform_listener=None):
        if transform_listener:
            self.tl = transform_listener
        else:
            self.tl = tf2_ros.TransformListener(tf2_ros.Buffer())
        self.controller = controller
        self.tf_prefix = rospy.get_param("~tf_prefix","")

    def start(self):
        self.setup_pub_sub()
        self.__control_monitor_thread = threading.Thread(target=self.__pose_updater)
        self.__control_monitor_thread.start()
        self.__result_pub_thread = threading.Thread(target=self.__result_listener)
        self.__result_pub_thread.start()
        self.controller.start()
        # NOTE(nickswalker5-1-21): Look, I don't know why rospy Publishers
        # aren't valid for the first part of their life, but they aren't. Without
        # this sleep, visualization won't work for the first few moments, which
        # is enough for test viz messages to now show up.
        rospy.sleep(0.5)

    def shutdown(self):
        # Fire and reset signals to give people a chance to realize we shut down
        self.controller.shutdown()
        self._follow_path_srv.shutdown()
        self._control_pub.unregister()
        self._error_pub.unregister()
        self._reference_state_pub.unregister()
        self._path_pub.unregister()
        self._real_path_pub.unregister()

    def __pose_updater(self):
        # We're limited by the slowest transforms in the
        # map -> base_footprint path. It's usually the real or
        # simulated odom data. Use `rostopic hz` to observe the
        # control frequency, and `tf_tools view_frames` to see
        # the transform frequency.
        pose_rate = rospy.Rate(100)
        while not self.controller.shutdown_event.is_set():
            # Wait for a path if there isn't one
            if self.controller.path is None:
                self._real_poses = []
                with self.controller.path_condition:
                    while (
                        self.controller.path is None
                        and not self.controller.shutdown_event.is_set()
                    ):
                        self.controller.path_condition.wait()
                if self.controller.shutdown_event.is_set():
                    break
            latest_pose = self._get_car_pose()
            if latest_pose is None:
                continue
            with self.controller.state_lock:
                self.controller.current_pose = latest_pose
            self._real_poses.append(utils.particle_to_pose(latest_pose))
            pose_rate.sleep()
        rospy.loginfo("Pose update shutdown")

    def __result_listener(self):
        i = 0
        while not self.controller.shutdown_event.is_set():
            # Keep track of how many loops we go through
            # so we can rate limit some viz topics.
            i += 1
            # Only publish if we've gone through another control loop
            self.controller.looped_event.wait()
            # We've been asleep for a bit. Check again that we're alive
            if self.controller.shutdown_event.is_set():
                continue
            if self.controller.selected_pose is not None:
                pose = self.controller.selected_pose
                p = PoseStamped()
                p.header = Header()
                p.header.stamp = rospy.Time.now()
                p.header.frame_id = "map"
                p.pose.position.x = pose[0]
                p.pose.position.y = pose[1]
                p.pose.orientation = utils.angle_to_quaternion(pose[2])
                self._reference_state_pub.publish(p)

            if self.controller.error:
                self._error_pub.publish(data=self.controller.error)
            if self.controller.next_ctrl is not None:
                ctrl = self.controller.next_ctrl
                assert len(ctrl) == 2
                ctrlmsg = AckermannDriveStamped()
                ctrlmsg.header.stamp = rospy.Time.now()
                ctrlmsg.drive.speed = ctrl[0]
                ctrlmsg.drive.steering_angle = ctrl[1]
                self._control_pub.publish(ctrlmsg)
            if (
                self.controller.costs is not None
                and self.controller.rollouts is not None
            ) and self._rollouts_pub.get_num_connections() > 0:
                markers = rollouts_to_markers_cmap(
                    self.controller.rollouts, self.controller.costs
                )
                self._rollouts_pub.publish(markers)

            if (
                len(self._real_poses) > 0
                and i % 10 == 0
                and self._real_path_pub.get_num_connections() > 0
            ):
                path = PoseArray(header=Header(frame_id="map"), poses=self._real_poses)
                self._real_path_pub.publish(path)

        rospy.loginfo("Result monitor shutdown")

    def setup_pub_sub(self):
        """Initialize ROS service and publishers."""
        self._follow_path_srv = rospy.Service("~follow_path", FollowPath, self.cb_path)

        self._control_pub = rospy.Publisher(
            "mux/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped,
            queue_size=2,
        )

        self._error_pub = rospy.Publisher(
            "~error",
            Float64,
            queue_size=1,
        )

        self._rollouts_pub = rospy.Publisher("~rollouts", MarkerArray, queue_size=100)

        self._reference_state_pub = rospy.Publisher(
            "~path/reference_state",
            PoseStamped,
            queue_size=1,
        )

        self._path_pub = rospy.Publisher(
            "~path/poses", PoseArray, queue_size=1, latch=True
        )

        self._real_poses = []
        self._real_path_pub = rospy.Publisher(
            "~real_path/poses", PoseArray, queue_size=1, latch=True
        )


    def reset_state(self):
        """Reset the controller's internal state (e.g., accumulators in PID control)."""
        rospy.loginfo("Start state reset")
        with self.reset_lock:
            self.controller.reset_state()
        rospy.loginfo("End state reset")
        return []

    def follow_path_with_speed(self, path_xyt, speed):
        """Follow a geometric path of states with a desired speed.

        Args:
            path_xyt: np.array of states with shape L x 3
            speed (double): desired speed
        """
        if not self.controller.is_alive():
            raise RuntimeError("Path command set before controller was started")
        path_xytv = time_parameterize_ramp_up_ramp_down(
            path_xyt, speed, self.controller.min_speed
        )
        self.follow_path(path_xytv)

    def follow_path(self, path_xytv):
        """Follow a geometric path of states and pre-specified speed.

        Args:
            path_xytv: np.array of states and speed with shape L x 4
        """
        if not self.controller.is_alive():
            raise RuntimeError("Path command set before controller was started")
        path_pose_array = configs_to_pose_array(path_xytv[:, :3])
        self._path_pub.publish(path_pose_array)
        self.controller.set_path(path_xytv)
        rospy.loginfo("Path set")

    def cb_path(self, msg):
        """Handle a new geometric path tracking request."""
        # Calling the service interrupts any current path execution
        self.controller.cancel_path()
        speed = msg.speed
        transformed_path = list(
            map(
                lambda pose: self.tl.buffer.transform(
                    pose, "map", timeout=rospy.Duration(1.0)
                ),
                msg.path.poses,
            )
        )
        path_xyt = np.array(
            list(map(lambda pose: utils.pose_to_particle(pose.pose), transformed_path))
        )
        self.follow_path_with_speed(path_xyt, speed)
        finished, error = self.wait_for_finish()
        return finished, error

    def wait_for_finish(self, timeout=None):
        """Wait for the controller to terminate efforts on the current path.

        Args:
            timeout: maximum duration to wait for the controller to finish

        Returns:
            completed: whether the robot reached the end of the path
            errored: whether the robot exceeded its error threshold
        """
        # If the controller has a path, it must fire the finished event
        if self.controller.path is not None:
            self.controller.finished_event.wait(timeout=timeout)
        return bool(self.controller.completed), bool(self.controller.errored)

    def _get_car_pose(self):
        """Return the current vehicle state."""
        try:
            transform = self.tl.buffer.lookup_transform(
                "map", self.tf_prefix + "base_footprint", rospy.Time(0)
            )
            # Drop stamp header
            transform = transform.transform
            return np.array(
                [
                    transform.translation.x,
                    transform.translation.y,
                    utils.quaternion_to_angle(transform.rotation),
                ]
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn_throttle(5, e)
            return None


def override_param(param_store, param_name, param_type, default="UNSPECIFIED"):
    """Set the param value that will override the base param value.

    param_store (dict): overriding parameter keys and values
    param_name (str): parameter name
    param_type: type constructor e.g. float, int
    default: default parameter value
    """
    key_name = param_name.lstrip("~").split("/")[-1]
    if rospy.has_param(param_name):
        param_store[key_name] = param_type(rospy.get_param(param_name))
    elif default != "UNSPECIFIED":
        param_store[key_name] = default


def get_ros_params():
    """Pull controller parameters from the ROS parameter server.

    Assumes that a node has been initialized and that parameters are loaded into
    the node's namespace, so they can be accessed under the "~" prefix.

    Returns:
        controller_type (str): one of "pid", "pp", "mpc"
        controller_params (dict): controller-specific parameters
    """
    base_params = {
        "car_length": float(rospy.get_param("~car_length", 0.33)),
        "frequency": float(rospy.get_param("~frequency", 50)),
        "finish_threshold": float(rospy.get_param("~finish_threshold", 0.3)),
        "exceed_threshold": float(rospy.get_param("~exceed_threshold", 4.00)),
        "distance_lookahead": float(rospy.get_param("~distance_lookahead", 0.6)),
        "min_speed": float(rospy.get_param("~min_speed", 0.5)),
    }

    controller_type = rospy.get_param("~type", "").lower()
    if controller_type == "pid":
        params = {
            "kp": float(rospy.get_param("~pid/kp")),
            "kd": float(rospy.get_param("~pid/kd")),
        }
        del base_params["car_length"]
        override_param(params, "~pid/frequency", float)
        override_param(params, "~pid/finish_threshold", float)
        override_param(params, "~pid/exceed_threshold", float)
        override_param(params, "~pid/distance_lookahead", float)
        override_param(params, "~pid/min_speed", float)
    elif controller_type == "pp":
        params = {}
        override_param(params, "~pp/frequency", float)
        override_param(params, "~pp/finish_threshold", float)
        override_param(params, "~pp/exceed_threshold", float)
        override_param(params, "~pp/distance_lookahead", float)
        override_param(params, "~pp/min_speed", float)
        override_param(params, "~pp/car_length", float)
    elif controller_type == "mpc":
        params = {
            "car_width": float(rospy.get_param("~mpc/car_width", 0.15)),
            "collision_w": float(rospy.get_param("~mpc/collision_w", 1e5)),
            "error_w": float(rospy.get_param("~mpc/error_w", 1.0)),
            "min_delta": float(rospy.get_param("~mpc/min_delta", -0.34)),
            "max_delta": float(rospy.get_param("~mpc/max_delta", 0.34)),
            "K": int(rospy.get_param("~mpc/K")),
            "T": int(rospy.get_param("~mpc/T")),
            "kinematics_params": {
                "vel_std": float(rospy.get_param("~motion_params/vel_std")),
                "delta_std": float(rospy.get_param("~motion_params/delta_std")),
                "x_std": float(rospy.get_param("~motion_params/x_std")),
                "y_std": float(rospy.get_param("~motion_params/y_std")),
                "theta_std": float(rospy.get_param("~motion_params/theta_std")),
            },
        }
        permissible_region, map_info = utils.get_map("/static_map")
        params["permissible_region"] = permissible_region
        params["map_info"] = map_info
        override_param(params, "~mpc/frequency", float)
        override_param(params, "~mpc/finish_threshold", float)
        override_param(params, "~mpc/exceed_threshold", float)
        override_param(params, "~mpc/distance_lookahead", float)
        override_param(params, "~mpc/min_speed", float)
        override_param(params, "~mpc/car_length", float)
    else:
        raise RuntimeError(
            "'{}' is not a controller. You must specify a valid controller type".format(
                controller_type
            )
        )
    merged_params = base_params.copy()
    merged_params.update(params)
    return controller_type, merged_params


def rollouts_to_markers_cmap(poses, costs, ns="paths", cmap="cividis", scale=0.01):
    """
    Visualize poses and costs in rviz.

    """
    max_c = np.max(costs)
    min_c = np.min(costs)
    norm = colors.Normalize(vmin=min_c, vmax=max_c)
    if cmap not in cm.cmaps_listed.keys():
        cmap = "viridis"
    cmap = cm.get_cmap(name=cmap)

    def colorfn(cost):
        r, g, b, a = 0.0, 0.0, 0.0, 1.0
        col = cmap(norm(cost))
        r, g, b = col[0], col[1], col[2]
        if len(col) > 3:
            a = col[3]
        return r, g, b, a

    return rollouts_to_markers(poses, costs, colorfn, ns, scale)


def rollouts_to_markers(poses, costs, colorfn, ns="paths", scale=0.01):
    """
    poses should be an array of trajectories to plot in rviz
    costs should have the same dimensionality as poses.size()[0]
    colorfn maps a point to an rgb tuple of colors
    """
    assert poses.shape[0] == costs.shape[0]

    markers = MarkerArray()

    for i, (traj, cost) in enumerate(zip(poses, costs)):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns + str(i)
        m.id = i
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.color.r, m.color.g, m.color.b, m.color.a = colorfn(cost)

        for t in traj:
            p = Point()
            p.x, p.y = t[0], t[1]
            m.points.append(p)

        markers.markers.append(m)
    return markers


def configs_to_pose_array(path_xyt):
    """Publish path visualization messages."""
    path_as_poses = list(
        map(
            utils.particle_to_pose,
            path_xyt,
        )
    )
    pa = PoseArray()
    pa.header = Header(frame_id="map")
    pa.poses = path_as_poses
    return pa
