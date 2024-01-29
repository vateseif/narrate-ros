#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion


class Robot(object):
    """Robot"""

    def __init__(self):
        super(Robot, self).__init__()

        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot", anonymous=True)

        # init pubs and subs
        self.pose_publisher = rospy.Publisher('/pose', Pose, queue_size=1)
        self.target_pose_subscriber = rospy.Subscriber('/target_pose', Pose, self.go_to_pose)


        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def go_to_pose(self, pose_displacement):
        """
        ee_pose_displacement: [x, y, z, qx, qy, qz, qw]
        """
        print(f"{pose_displacement=}")

        
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
        return self.move_group.get_current_pose().pose
    
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # publish current pose
            self.pose_publisher.publish(self.get_pose())
            rate.sleep()

if __name__ == "__main__":
    robot = Robot()
    robot.run()