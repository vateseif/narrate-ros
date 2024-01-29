#!/usr/bin/env python3
import os
import sys
print(os.getcwd())
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'config'))


import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import tf.transformations

import numpy as np
from controller import Controller
from config import ControllerConfig




print(f"CONTROLLER CONFIG STATUS = {ControllerConfig.status}")

def talker():
    rospy.init_node('mpc', anonymous=True)
    pub = rospy.Publisher('/target_pose', Pose, queue_size=5)
    rate = rospy.Rate(5)  # 10 Hz

    # Example pose: position (x, y, z) and orientation in Euler angles (roll, pitch, yaw)
    position = Point(0.3, 0.0, 0.3)  # Substitute with actual values
    euler = (pi, 0., -pi/4)
    #quaternion = Quaternion(0.924, -0.383, 0., 0.)  # Substitute with actual values
    #target_euler = tf.transformations.euler_from_quaternion((0.924, -0.383, 0., 0.))
    #print(f"TARGET EULER = {target_euler}")
    # Wait for the user to press Enter
    input("Press Enter to publish the robot's pose...")
    # Convert Euler angles to quaternion for orientation
    quaternion = Quaternion(*tf.transformations.quaternion_from_euler(*euler))
    # Create Pose message
    
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        position = Point(0.3, 0.0, 0.4 + 0.1*sin(t))

        pose = Pose(position, quaternion)

        rospy.loginfo(pose)
        pub.publish(pose)
        rospy.loginfo("Pose published. Node will now shutdown.")
        #rospy.signal_shutdown("Pose published")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
