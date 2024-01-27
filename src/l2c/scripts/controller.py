#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#import tf.transformations


def talker():
    rospy.init_node('mpc', anonymous=True)
    pub = rospy.Publisher('/pose', Pose, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Example pose: position (x, y, z) and orientation in Euler angles (roll, pitch, yaw)
    position = Point(0.3, 0.0, 0.6)  # Substitute with actual values
    quaternion = Quaternion(0., 0., 0., 1.)  # Substitute with actual values

    # Wait for the user to press Enter
    input("Press Enter to publish the robot's pose...")
    # Convert Euler angles to quaternion for orientation
    #quaternion = Quaternion(*tf.transformations.quaternion_from_euler(*euler))
    # Create Pose message
    
    pose = Pose(position, quaternion)
    rospy.loginfo(pose)
    pub.publish(pose)
    rospy.loginfo("Pose published. Node will now shutdown.")
    rospy.signal_shutdown("Pose published")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
