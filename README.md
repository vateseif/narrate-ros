# l2c_sim
# Installation
- Install [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- Install [Franka libraries](https://frankaemika.github.io/docs/installation_linux.html)
- Set up [real-time kernel for Franka](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel) on the workstation PC
- Install [MoveIt](https://moveit.ros.org/install/) binary (apt)
- Install [MoveIt](https://moveit.ros.org/install/source/) from source and `catkin build panda_moveit_config`

- (not sure) Install deps: `sudo rosdep init` (if you don't have it yet), `rosdep update`, `rosdep install --from-paths src --ignore-src -r -y`
- Install packages in this catkin workspace with `catkin_make`
- Source with `source devel/setup.bash`

# Run
- Launch with `roslaunch l2c moveit_panda.launch` launches the panda robot, moveit, and a custom world
- `rosrun l2c state_estimation.py`
- `rosrun l2c controller.py`

# Useful tools
- [Gazebo models](https://github.com/osrf/gazebo_models?tab=readme-ov-file)

# Real robot connection
Check [this](https://frankaemika.github.io/docs/getting_started.html).

# Useful resources
- [Moveit 1 Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
- [Moveit 1 Python Interface Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
