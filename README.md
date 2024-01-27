# l2c_sim
# Installation
- Install [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- Install [Franka libraries](https://frankaemika.github.io/docs/installation_linux.html)
- Set up [real-time kernel for Franka](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel) on the workstation PC
- Install [MoveIt](https://moveit.ros.org/install/) binary (apt)
- Install [MoveIt](https://moveit.ros.org/install/source/) from source and `catkin build panda_moveit_config`

- Set up a catkin workspace with `catkin_make` in the root folder of this repo (might not be necessary)
- Install deps: `sudo rosdep init` (if you don't have it yet), `rosdep update`, `rosdep install --from-paths src --ignore-src -r -y`
- Install packages in this catkin workspace with `catkin_make`
- Source with `source devel/setup.bash`
- Launch with `roslaunch moveit_servoing pose_tracking.launch`

# Useful tools
- [Gazebo models](https://github.com/osrf/gazebo_models?tab=readme-ov-file)

# Real robot connection
Check [this](https://frankaemika.github.io/docs/getting_started.html).

# Run an example
To launch moveit and panda (franka robot) on gazebo
```
roslaunch panda_moveit_config demo_gazebo.launch
```

Also launch this (integration not working yet...)
```
roslaunch moveit_servoing pose_tracking.launch
```

# Useful resources
- [Moveit 1 Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
