#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def broadcast_transforms_panda():
    transform = TransformStamped()
    transform.header.frame_id = "world"
    transform.child_frame_id = "robot_base"
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    return [transform]

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    br = tf2_ros.StaticTransformBroadcaster()
    t_panda = broadcast_transforms_panda()
    br.sendTransform(t_panda)
    print("[BROADCAST_STATIC_TF] Broadcasting static transforms")
    rospy.spin()
