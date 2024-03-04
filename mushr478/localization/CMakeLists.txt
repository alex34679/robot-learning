cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS
  scripts/download_bags.sh
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY bags config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
  scripts/make_motion_model_plot
  scripts/make_sensor_model_single_plot
  scripts/make_sensor_model_likelihood_plot
  scripts/make_particle_filter_plot
  scripts/particle_filter_node
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)

  # catkin_add_nosetests tests Python code directly

  # Q1, Q2, Q3 tests
  catkin_add_nosetests(test/motion_model.py)
  catkin_add_nosetests(test/sensor_model.py)
  catkin_add_nosetests(test/particle_initializer.py)
  catkin_add_nosetests(test/resample.py)

  # add_rostest tests the ROS integrations

  # Q3 tests
  add_rostest(test/particle_filter.test ARGS bag_name:=circle)
  add_rostest(test/particle_filter.test ARGS bag_name:=full)

endif()
