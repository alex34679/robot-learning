#!/usr/bin/env python
from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped

from cse478 import utils
from cse478.collector import SynchronizedMessageCollector


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        pass

    def _test_particle_filter(self):
        reference_pose_topic = "reference_pose"
        plot = bool(rospy.get_param("~plot", False))
        bag_name = rospy.get_param("~bag_name")

        collector = SynchronizedMessageCollector(
            ["/car/particle_filter/inferred_pose", reference_pose_topic],
            [PoseStamped, PoseStamped],
        )
        try:
            rospy.sleep(5)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # We expect time to jump back because the test restarts the bag
            pass
        msgs = collector.start(duration=20)
        self.assertGreaterEqual(
            len(msgs),
            50,
            msg="The test didn't receive enough messages to be able to compare "
            "the particle filter estimate with the ground truth.",
        )

        # Convert synchronized PoseStamped messages to state vectors
        estimates, references = [
            [utils.pose_to_particle(m.pose) for m in ms] for ms in zip(*msgs)
        ]
        estimates = np.array(estimates)
        references = np.array(references)

        # Drop (0, 0, 0) estimates from pre-mature initialization
        premature = (np.abs(estimates) < 2e-1).all(axis=1)
        estimates = estimates[~premature, :]
        references = references[~premature, :]

        position_error, abs_angular_error = utils.estimation_error(
            estimates, references
        )
        # BEGIN SOLUTION NO PROMPT
        rospy.loginfo("Median position error: {}".format(np.median(position_error)))
        rospy.loginfo(
            "Median absolute angular error: {}".format(np.median(abs_angular_error))
        )
        # END SOLUTION
        if plot:
            plt.xlabel("x")
            plt.ylabel("y")
            plt.plot(estimates[:, 0], estimates[:, 1], c="r", label="Estimated State")
            plt.plot(references[:, 0], references[:, 1], c="g", label="Ground Truth")
            plt.legend()
            plt.gca().set_aspect(aspect=1.0)
            plt.show()

        if bag_name != "full":
            return

        # thresholds for full.bag
        pos_threshold, ang_threshold = 0.4, 0.2
        self.assertLess(
            np.median(position_error),
            pos_threshold,
            msg="The median positional error should be less than {} meters".format(
                pos_threshold
            ),
        )
        self.assertLess(
            np.median(abs_angular_error),
            ang_threshold,
            msg="The median angular error should be less than {} radians".format(
                ang_threshold
            ),
        )


if __name__ == "__main__":
    rospy.init_node("test_particle_filter")
    bag_name = rospy.get_param("~bag_name")
    # The xml report will use the method's name, so we have to manually
    # mangle it with the bag name to ensure all entries appear
    setattr(
        TestParticleFilter,
        "test_particle_filter_{}".format(bag_name),
        lambda self: self._test_particle_filter(),
    )
    rostest.rosrun(
        "localization", "test_particle_filter_{}".format(bag_name), TestParticleFilter
    )
