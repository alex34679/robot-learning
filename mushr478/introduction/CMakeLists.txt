cmake_minimum_required(VERSION 3.0.2)
project(introduction)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

install(DIRECTORY launch plans
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
  scripts/fibonacci
  scripts/compare_norm
  scripts/path_publisher
  scripts/pose_listener
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin_add_nosetests tests Python code directly
# add_rostest tests the ROS integrations
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(test/path_publisher.test)

  # Q1 tests
  catkin_add_nosetests(test/fibonacci.py)
  add_rostest(test/fibonacci_small.test)
  add_rostest(test/fibonacci_launch.test)

  # Q2 tests
  catkin_add_nosetests(test/norms.py)
  add_rostest(test/pose_listener.test)

  # Q3 tests
  catkin_add_nosetests(test/indexing.py)

endif()
