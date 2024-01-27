# project(l2o_interface_mock)
# cmake_minimum_required(VERSION 3.1.3)

catkin_install_python(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  script/robot.py
  script/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
