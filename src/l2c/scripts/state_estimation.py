#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped

import numpy as np
import pinocchio as pin
#from pinocchio.utils import se3ToXYZQUAT

class FrankaEEFInfoNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('state_estimation')

        # Load the URDF of the Franka robot (ensure the correct path)
        self.robot_model = pin.buildModelFromUrdf(os.path.join(os.path.dirname(__file__), '..', 'robots/panda.urdf'))
        self.robot_data = self.robot_model.createData()

        # subs
        self.robot_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        # pubs
        self.ee_pose_publisher = rospy.Publisher('/ee_pose', PoseStamped, queue_size=1)
        self.ee_velocity_publisher = rospy.Publisher('/ee_twist', TwistStamped, queue_size=1)

    def joint_state_callback(self, msg):
        # Extract joint positions from the message
        q = np.array(msg.position)[:7]
        # Assuming the velocities are also available
        v = np.array(msg.velocity)[:7]

        # Compute the forward kinematics to get the end effector's pose
        pin.forwardKinematics(self.robot_model, self.robot_data, q)
        #eef_frame_id = self.robot_model.getFrameId('panda_link7') # Check your URDF for the correct end effector link name
        eef_pose = self.robot_data.oMi[7] # TODO: set 7 (last link index) into the config
        #print(eef_pose)

        # Convert the pose to a ROS Pose message
        pose = Pose()
        xyz_quat = pin.SE3ToXYZQUAT(eef_pose)
        pose.position.x, pose.position.y, pose.position.z = (xyz_quat[0], xyz_quat[1], xyz_quat[2])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (xyz_quat[3], xyz_quat[4], xyz_quat[5], xyz_quat[6])

        # Compute the Jacobian and use it to calculate the end effector's velocity
        J = pin.computeFrameJacobian(self.robot_model, self.robot_data, q, 7) # TODO: set 7 (last link index) into the config
        v_eef = J.dot(v)  # End effector velocity in the local frame

        # Convert the velocity to a ROS Twist message
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = v_eef[:3]
        twist.angular.x, twist.angular.y, twist.angular.z = v_eef[3:]

        # add timestamps
        pose_msg = PoseStamped()
        pose_msg.pose = pose
        pose_msg.header.stamp = rospy.Time.now()

        twist_msg = TwistStamped()
        twist_msg.twist = twist
        twist_msg.header.stamp = rospy.Time.now()
        
        # publish
        self.ee_pose_publisher.publish(pose_msg)
        self.ee_velocity_publisher.publish(twist_msg)
        rospy.loginfo("OK")

if __name__ == '__main__':
    node = FrankaEEFInfoNode()
    rospy.spin()
