#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped

import numpy as np
import pinocchio as pin
from pinocchio.utils import se3ToXYZQUAT

class FrankaEEFInfoNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('state_estimation')

        # Load the URDF of the Franka robot (ensure the correct path)
        self.robot_model = pin.buildModelFromUrdf('/home/seif/projects/l2c_sim/src/l2c/robots/panda.urdf')
        self.robot_data = self.robot_model.createData()
        self.robot_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.eef_pose_publisher = rospy.Publisher('/ee_pose', Pose, queue_size=1)
        self.eef_velocity_publisher = rospy.Publisher('/ee_twist', Twist, queue_size=1)

    def joint_state_callback(self, msg):
        # Extract joint positions from the message
        q = np.array(msg.position)[:7]
        # Assuming the velocities are also available
        v = np.array(msg.velocity)[:7]

        # Compute the forward kinematics to get the end effector's pose
        pin.forwardKinematics(self.robot_model, self.robot_data, q)
        #eef_frame_id = self.robot_model.getFrameId('panda_link7') # Check your URDF for the correct end effector link name
        eef_pose = self.robot_data.oMi[7]
        print(eef_pose)

        # Convert the pose to a ROS Pose message
        pose_msg = Pose()
        xyz_quat = se3ToXYZQUAT(eef_pose)
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = (xyz_quat[0], xyz_quat[1], xyz_quat[2])
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = (xyz_quat[3], xyz_quat[4], xyz_quat[5], xyz_quat[6])

        # Compute the Jacobian and use it to calculate the end effector's velocity
        J = pin.computeFrameJacobian(self.robot_model, self.robot_data, q, 7)
        v_eef = J.dot(v)  # End effector velocity in the local frame

        # Convert the velocity to a ROS Twist message
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = v_eef[:3]
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = v_eef[3:]

        # Publish the messages
        self.eef_pose_publisher.publish(pose_msg)
        self.eef_velocity_publisher.publish(twist_msg)

        rospy.loginfo(pose_msg)
        rospy.loginfo(twist_msg)

if __name__ == '__main__':
    node = FrankaEEFInfoNode()
    rospy.spin()
