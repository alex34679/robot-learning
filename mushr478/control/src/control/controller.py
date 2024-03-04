from __future__ import division
import numpy as np
import threading
import time


def compute_position_in_frame(p, frame):
    """Compute the position in a new coordinate frame.

    Args:
        p: vehicle state [x, y, heading]
        frame: vehicle state [x, y, heading]

    Returns:
        error: p expressed in the new coordinate frame [e_x, e_y]
    """
    # BEGIN QUESTION 1.2
    "*** REPLACE THIS LINE ***"
    raise NotImplementedError
    # END QUESTION 1.2


class BaseController(object):
    def __init__(self, **kwargs):
        self.__properties = {
            "frequency",
            "finish_threshold",
            "exceed_threshold",
            "distance_lookahead",
            "min_speed",
        }
        if not self.__properties == set(kwargs):
            raise ValueError(
                "Invalid keyword argument provided",
                set(kwargs).difference(self.__properties),
            )

        self.__dict__.update(kwargs)
        # Error is typically going to be near this distance lookahead parameter,
        # so if the exceed threshold is lower we'll just immediately error out
        assert self.distance_lookahead < self.exceed_threshold

        self.path = None
        self.ready_event = threading.Event()
        self.path_condition = threading.Condition()
        self.finished_event = threading.Event()
        self.looped_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.state_lock = threading.RLock()
        self.reset_state()

        self.current_pose = None

    def get_reference_index(self, pose, path_xytv, distance_lookahead):
        """Return the index to the next control target on the reference path.

        To compute a reference state that is some lookahead distance away, we
        recommend first finding the state on the path that is closest to the
        current vehicle state. Then, step along the path's waypoints and select
        the index of the first state that is greater than the lookahead distance
        from the current state. (You can also look back one index to see which
        state is closer to the desired lookahead distance.)

        Note that this method must be computationally efficient, since it runs
        directly in the control loop. Vectorize where you can.

        Args:
            pose: current state of the vehicle [x, y, heading]
            path_xytv: np.array of states and speeds with shape L x 4
            distance_lookahead (float): lookahead distance

        Returns:
            index to the reference state on the path

        """
        with self.path_condition:
            # Hint: compute all the distances from the current state to the
            # path's waypoints. You may find the `argmin` method useful.
            # BEGIN QUESTION 1.1
            "*** REPLACE THIS LINE ***"
            raise NotImplementedError
            # END QUESTION 1.1
            return len(path_xytv) - 1

    def get_error(self, pose, reference_xytv):
        """Compute the error vector.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: error vector
        """
        # Subclasses will override this method
        raise NotImplementedError

    def get_control(self, pose, reference_xytv, error):
        """Compute the control action.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: [velocity, steering angle]
        """
        # Subclasses will override this method
        raise NotImplementedError

    def path_complete(self, pose, error, distance_lookahead):
        """Return whether the reference path has been completed.

        The path completes successfully if the reference index refers to the
        final point on the path, and when the error is below finish_threshold.

        The path completes unsuccessfully when the error is above
        exceed_threshold.

        Args:
            pose: current state of the vehicle [x, y, heading]
            error: error vector [e_x, e_y]

        Returns:
            complete: whether the robot has reached the end of the path
            errored: whether the robot has exceeded its error threshold
        """
        ref_is_path_end = self.get_reference_index(
            pose, self.path, distance_lookahead
        ) == (len(self.path) - 1)
        err_l2 = np.linalg.norm(error)
        within_error = err_l2 < self.finish_threshold
        beyond_exceed = err_l2 > self.exceed_threshold
        return ref_is_path_end and within_error, beyond_exceed

    ################
    # Control Loop #
    ################

    def start(self):
        """Start the control loop."""
        self.__control_thread = threading.Thread(target=self._control_loop)
        self.__control_thread.start()

    def _control_loop(self):
        """Implement the control loop."""
        self.ready_event.set()

        interval = 1 / self.frequency
        last_stamp = time.time()

        while not self.shutdown_event.is_set():
            # Sleep long enough to hit our target frequency
            next_stamp = last_stamp + interval
            time.sleep(max(next_stamp - time.time(), 0))
            last_stamp = next_stamp
            # No point in looping until we have path, so wait until it's there.
            # Once we see a path, the condition variable will give us the lock
            # on it for the with context
            with self.path_condition:
                # We hold the lock. Give it up until a path is set
                while self.path is None and not self.shutdown_event.is_set():
                    self.path_condition.wait()
                # We have a path, and we're guaranteed exclusive access now
                # Check to see if we got killed while waited for a path
                if self.shutdown_event.is_set():
                    break

                # Code will crash for degenerate path otherwise
                if len(self.path) == 0:
                    self.completed = True
                    self.errored = False
                    self.path = None
                    self.finished_event.set()
                    self.finished_event.clear()
                    continue

                with self.state_lock:
                    if self.current_pose is None:
                        # No controls calculated, so we won't fire that event.
                        continue
                    index = self.get_reference_index(
                        self.current_pose, self.path, self.distance_lookahead
                    )
                    self.selected_pose = self.get_reference_pose(index)
                    self.error = np.linalg.norm(
                        self.selected_pose[:2] - self.current_pose[:2]
                    )

                    error = self.get_error(self.current_pose, self.selected_pose)
                    self.next_ctrl = self.get_control(
                        self.current_pose, self.selected_pose, error
                    )

                    complete, errored = self.path_complete(
                        self.current_pose, error, self.distance_lookahead
                    )

                    # Maintain some state so we can check if we're stuck
                    if self.prev_pose is None:
                        self.prev_pose = self.current_pose
                        self.prev_pose_stamp = time.time()
                    else:
                        progress = np.linalg.norm(
                            np.array(self.prev_pose[:2]) - self.current_pose[:2]
                        )
                        if progress > 0.5:
                            self.prev_pose = self.current_pose
                            self.prev_pose_stamp = time.time()
                        elif time.time() - self.prev_pose_stamp > 10.0:
                            # We've been within a radius for more than 10 seconds
                            errored = True

                    # Let anyone monitoring know that we've computed controls
                    self.looped_event.set()
                    self.looped_event.clear()
                    # Clear this out so we don't repeat the exact same calculations.
                    self.current_pose = None
                    if complete or errored:
                        self.errored = errored
                        self.completed = complete
                        self.path = None
                        self.finished_event.set()
                        self.finished_event.clear()

        print("Control loop ending")

    def shutdown(self):
        """Shut down the controller."""
        self.shutdown_event.set()
        # We could be waiting for a path to become active. Set the
        # event so the thread can wakeup and see that we've shut down
        with self.path_condition:
            self.path_condition.notify_all()

        # Someone out there could also be waiting for us to loop
        self.looped_event.set()


    def reset_state(self):
        """Reset the controller's internal state."""
        # Override in the child class and ensure that super is called
        with self.state_lock:
            self.selected_pose = None
            self.error = None
            self.next_ctrl = None
            self.rollouts = None
            self.costs = None
            self.errored = None
            self.completed = None
            self.prev_pose = None
            self.prev_pose_stamp = None

    ####################
    # Helper Functions #
    ####################

    def is_alive(self):
        """Return whether controller is ready to begin tracking."""
        return self.ready_event.is_set()

    def get_reference_pose(self, index):
        """Return the reference state from the reference path at the index."""
        with self.path_condition:
            assert len(self.path) > index
            return self.path[index]

    def set_path(self, path):
        """Set the reference path to track.

        This implicitly resets the internal state of the controller.
        """
        with self.path_condition:
            self.path = path
            self.reset_state()
            # We only have one "consumer" by default, but
            # others could be monitoring and want to know
            self.path_condition.notify_all()

    def cancel_path(self):
        """Cancel the current path being tracked, if one exists."""
        if self.path is None:
            # Nothing to cancel
            return False

        with self.path_condition:
            # The control loop will not run after this point, because
            # it'll never acquire the right condition
            self.path = None
            self.path_condition.notify_all()

        # The control loop didn't do the usual "I'm done" ceremony,
        # so let's tell everyone what happened
        with self.state_lock:
            self.completed = False
            self.errored = False
            self.finished_event.set()
            self.finished_event.clear()

        # Listeners don't expect the control to stop coming, so
        # wake them up. It's their responsibility to see that
        # the path is dead at that point
        self.looped_event.set()
        self.looped_event.clear()
        return True


def time_parameterize_ramp_up_ramp_down(path_xyt, speed, min_speed):
    """Parameterize a geometric path of states with a desired speed.

    Vehicles can't instantly reach the desired speed, so we need to ramp up to
    full speed and ramp down to 0 at the end of the path.

    Args:
        path_xyt: np.array of states with shape L x 3
        speed (double): desired speed

    Returns:
        path_xytv: np.array of states and speed with shape L x 4
    """
    # For paths with fewer waypoints than necessary to ramp up and ramp down,
    # just set the desired speed directly (with a non-zero final speed, to
    # guarantee forward movement).
    if path_xyt.shape[0] < 4:
        speeds = speed * np.ones(path_xyt.shape[0])
        speeds = np.maximum(speeds, min_speed)
        return np.hstack([path_xyt, speeds[:, np.newaxis]])

    ramp_distance = 0.5
    displacement_vectors = np.diff(path_xyt[:, [0, 1]], axis=0)
    displacements = np.linalg.norm(displacement_vectors, axis=1)
    cumulative_path_length = np.cumsum(displacements)
    ramp_mask = (cumulative_path_length < ramp_distance) | (
        displacements.sum() - ramp_distance < cumulative_path_length
    )
    # At least one step of slowdown on the front and back
    ramp_mask[[0, -1]] = True
    change_points = np.where(np.diff(ramp_mask))[0]
    speeds = np.interp(
        np.arange(len(path_xyt)) / len(path_xyt),
        [0.0, change_points[0] / len(path_xyt), change_points[1] / len(path_xyt), 1.0],
        [0, speed, speed, 0],
    )
    speeds = np.maximum(speeds, min_speed)
    return np.hstack([path_xyt, speeds[:, np.newaxis]])
