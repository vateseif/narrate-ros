#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory


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

    def go_to_pose(self, ee_pose):
        """
        ee_pose: [x, y, z, qx, qy, qz, qw]
        """
        print(f"{ee_pose=}")

        self.move_group.set_pose_target(ee_pose)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

        return True
    
    def get_pose(self, req):
        return self.move_group.get_current_pose.pose
    
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # publish current pose
            self.pose_publisher.publish(self.get_pose())
            rate.sleep()

if __name__ == "__main__":
    robot = Robot()
    robot.run()