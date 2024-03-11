project(l2c)
cmake_minimum_required(VERSION 3.1.3)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  moveit_servo
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
  cfg/ControllerConfig.cfg
)

add_service_files(
  FILES
  GetEEState.srv
)

add_message_files(
  FILES
  Plan.msg
  Optimization.msg
  NamedPose.msg
  NamedPoseArrayStamped.msg
  PerceptionScore.msg
  PerceptionScoreDict.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  scripts/gui.py
  scripts/llm.py
  scripts/perception.py
  scripts/controller.py
  scripts/state_estimation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
)

# ## Declare a cpp library
# add_library(${PROJECT_NAME}_core
# )
# target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(pose_tracker
  scripts/pose_tracker.cpp
)
target_compile_features(pose_tracker INTERFACE cxx_std_11)

# ## Add dependencies to exported targets, like ROS msgs or srvs
# add_dependencies(${PROJECT_NAME}_core
#   ${catkin_EXPORTED_TARGETS}
# )

add_dependencies(pose_tracker
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

target_link_libraries(pose_tracker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS pose_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
