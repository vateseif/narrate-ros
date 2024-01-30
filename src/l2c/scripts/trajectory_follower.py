#!/usr/bin/env python3
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory


class TrajectoryFollower:

    def __init__(self) -> None:

        self.trajectory: MultiDOFJointTrajectory = None

        # init node
        rospy.init_node('trajectory_follower', anonymous=True)

        # subs
        self.robot_state_subscriber = rospy.Subscriber('/ee_trajectory', MultiDOFJointTrajectory, self.trajectory_callback, queue_size=1)

        # pubs
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)

    
    def trajectory_callback(self, trajectory:MultiDOFJointTrajectory):
        # TODO
        self.trajectory = trajectory
        pass


    def step(self):
        # TODO
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.pose.position.x = 0.300
        pose_msg.pose.position.y = 0.000
        pose_msg.pose.position.z = 0.300
        pose_msg.pose.orientation.x = 0.924, 
        pose_msg.pose.orientation.y = -0.383 
        pose_msg.pose.orientation.z = 0.
        pose_msg.pose.orientation.w = 0.
        
        
        self.target_pose_pub.publish(pose_msg)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("uploading pose")
            self.step()
            rate.sleep()


if __name__ == "__main__":
    trf = TrajectoryFollower()
    trf.run()