cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/roadmap
  scripts/run_search
  scripts/planner
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  # find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)

  # catkin_add_nosetests tests Python code directly

  catkin_add_nosetests(test/samplers.py)
  catkin_add_nosetests(test/roadmap.py)
  catkin_add_nosetests(test/astar.py)
  catkin_add_nosetests(test/shortcut.py)

  # add_rostest tests the ROS integrations

endif()
