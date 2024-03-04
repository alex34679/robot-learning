# mushr478

## Usage

This repository is meant to live in your ROS workspace's `src` directory. It expects to be overlaid on another workspace containing MuSHR dependency packages.

## Running Tests

Each package contains a `test` directory with unit and integration tests built with the [rosunit](https://wiki.ros.org/rosunit) framework. To run all of these tests:

    catkin test

If you just want to run tests for a particular package, add the package name onto the command as in this example:

    catkin test introduction

<details>
<summary>Advanced Test Usage</summary>

`catkin test` provides a summary view of all test results. You may need to see more detailed logs if you are, for instance, diagnosing why test isn't being run. To run tests for a package and see more log output:

    roscd introduction; catkin run_tests --no-deps --this

It is possible to run tests by individual file. The command differs by the types of tests; for tests that use ROS (they start a node, usually to publish or subscribe to topics from the code under test), use `rostest` to run the launch file for the test:

    rostest introduction pose_listener.test --text

For plain Python unit tests, simply run the file:

    python3 $(rospack find introduction)/test/norms.py

</details>

