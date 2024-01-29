# project(l2c)
# cmake_minimum_required(VERSION 3.1.3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  GetEEState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/robot.py
  scripts/main.py
  scripts/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
