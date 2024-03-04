cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rospy
  std_msgs
)

add_service_files(
  FILES
  FollowPath.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  nav_msgs
  std_msgs
)

install(
  DIRECTORY
  config
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/control_node
  scripts/path_sender
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)

  # catkin_add_nosetests tests Python code directly

  catkin_add_nosetests(test/controller.py)
  catkin_add_nosetests(test/pid.py)
  catkin_add_nosetests(test/purepursuit.py)
  catkin_add_nosetests(test/mpc.py)

  # add_rostest tests the ROS integrations

  add_rostest(test/controller_performance.test ARGS type:=pid)
  add_rostest(test/controller_performance.test ARGS type:=pp)
  add_rostest(test/controller_performance.test ARGS type:=mpc)

endif()
