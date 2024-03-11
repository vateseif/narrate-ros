# NARRATE (ROS) - Control and Optimization Language Architecture for Robotics
This repo contains a reference implementation of the paper [NARRATE](https://narrate-mpc.github.io) for ROS-1.

We open source the TP, OD, TG, and TT modules at the core of NARRATE. We don't release the perception module used in the paper's experiments as it is entirely dependent on the Aruco-based cubes used in the benchmarks.

The implementation assumes that the robot used is the Franka Emika Panda with the Panda Hand as gripper.

# Dependencies
- Install [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (untested on other ROS distros)
- Install [MoveIt](https://moveit.ros.org/install/) binary (apt)
- Install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). Make sure to specify the right python version when changing PATH in the `.bashrc`
- Install [Franka libraries](https://frankaemika.github.io/docs/installation_linux.html)

# Installation
- Install packages in this catkin workspace with `catkin build l2c`
- Source with `source devel/setup.bash`

# Usage
Launch the pipeline with the following command, and optionally record rosbags
```
roslaunch l2c pipeline_franka.launch record:=true record_camera:=false bag_prefix:=task_name
```
To interact with the TP and OD, launch the terminal-based GUI
```
python3 scripts/gui.py
```
Once the GUI is launched, prepend every command with `OD` or `TP` depending on the system you'd like to interact with. For example:
```
TP clean the pan
```
If the plan is longer than a single sub-task (if you called the TP), step through tasks clicking Enter once each sub-task is complete.
