#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import rospy
import moveit_commander
import tf.transformations as tfm
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped, Pose
 
from l2c.srv import GetEEState, GetEEStateResponse

import numpy as np

class Robot(object):
    """Robot"""

    def __init__(self):
        super(Robot, self).__init__()

        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot", anonymous=True)

        # init subs
        self.target_pose_sub = rospy.Subscriber('/target_pose', PoseStamped, self.go_to_pose)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.store_joint_velocities)

        # init pubs
        #self.pose_publisher = rospy.Publisher('/pose', Pose, queue_size=1)
        self.x0_pub = rospy.Service('get_x0', GetEEState, self.get_ee_state)

        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.robot = moveit_commander.RobotCommander("robot_description")

        #print(self.robot.get_current_state())

    def get_ee_state(self, req):
        """ Return the end-effector state as (pose: Pose, ee_velocity:Twist)"""
        # ee pose
        pose = self.get_pose()

        # compute ee velocity
        jacobian = self.move_group.get_jacobian_matrix(self.move_group.get_current_joint_values())
        
        ee_velocity = TwistStamped() # dim = 6
        vel = np.dot(jacobian, self.joint_velocities)
        ee_velocity.header.stamp = rospy.Time.now()
        ee_velocity.twist.linear.x = vel[0]
        ee_velocity.twist.linear.y = vel[1]
        ee_velocity.twist.linear.z = vel[2]
        ee_velocity.twist.angular.x = vel[3]
        ee_velocity.twist.angular.y = vel[4]
        ee_velocity.twist.angular.z = vel[5] 

        rospy.loginfo(pose)
        rospy.loginfo(ee_velocity)

        return GetEEStateResponse(pose, ee_velocity)

    def store_joint_velocities(self, msg):
        """ Store the velocities of the joints for later use"""
        joint_velocities = msg.velocity
        self.joint_velocities = np.array(joint_velocities)[:7]

    def go_to_pose(self, pose_displacement):
        """
        ee_pose_displacement: [x, y, z, qx, qy, qz, qw]
        """
        #print(f"{pose_displacement=}")
        pose_displacement = pose_displacement.pose
        
        target_pose = Pose()
        current_pose = self.get_pose()
        
        target_pose.position.x = current_pose.position.x + pose_displacement.position.x
        target_pose.position.y = current_pose.position.y + pose_displacement.position.y
        target_pose.position.z = current_pose.position.z + pose_displacement.position.z

        # Convert quaternions to numpy arrays
        current_pose_quat = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        pose_displacement_quat = [pose_displacement.orientation.x, pose_displacement.orientation.y, pose_displacement.orientation.z, pose_displacement.orientation.w]

        # Multiply the quaternions
        target_pose.orientation = Quaternion(*tfm.quaternion_multiply(current_pose_quat, pose_displacement_quat))

        # set target pose
        self.move_group.set_pose_target(target_pose)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

        return True
    
    def get_pose(self):
        #self.move_group.
        return self.move_group.get_current_pose()
    
    def run(self):
        rospy.spin() 

if __name__ == "__main__":
    robot = Robot()
    robot.run()