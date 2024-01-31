#!/usr/bin/env python
from __future__ import annotations
"""
Description: This node publishes the raw images from the gelsight sensors.
"""


from geometry_msgs.msg import PoseStamped
import rospy
from std_srvs.srv import Empty, SetBool
import numpy as np
from geometry_msgs.msg import Point, Transform
import tf.transformations as tft
import tf
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

def quaternion_to_axis_angle(q: np.ndarray):
    # Normalize quaternion
    q_norm = q / np.linalg.norm(q)
    
    # Extract angle
    angle = 2 * np.arccos(q_norm[-1])
    
    # Calculate axis
    sin_theta = np.sqrt(1 - q_norm[-1]**2)  # sin(theta)
    if sin_theta < 1e-6:
        # If sin(theta) is close to zero, set axis to [1, 0, 0] (arbitrary axis)
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = q_norm[:-1] / sin_theta
    
    angle = min(angle, 2 * np.pi - angle)
    return axis, angle

class TrackingNode:

    def __init__(self):
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)
        self._ee_frame = rospy.get_param("~ee_frame", "panda_hand")
        self._base_frame = rospy.get_param("~base_frame", "panda_link0")
        # subs
        self.robot_state_subscriber = rospy.Subscriber('/ee_trajectory', MultiDOFJointTrajectory, self.trajectory_callback, queue_size=1)
        self._tf_listener = tf.TransformListener()

        self._trajectory = []
        self._distance_resolution = rospy.get_param("~distance_resolution", 0.02)
        self._angle_resolution = rospy.get_param("~angle_resolution", 0.2)
        self._max_time_delay = rospy.get_param("~max_time_delay", 0.05)
        self._vis_traj_pub = rospy.Publisher("/planned_trajectory", MarkerArray, queue_size=1)

    def trajectory_callback(self, trajectory:MultiDOFJointTrajectory):
        rospy.loginfo("Received trajectory with %d points.", len(trajectory.points))
        if len(trajectory.points) > 1:
            rospy.logwarn("Received trajectory with more than one point. Only the first point will be used.")
            return
        
        self._trajectory = []
        for pose in trajectory.points[0].transforms:
            pos = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
            ori = np.array([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
            self._trajectory.append((pos, ori))


    def _get_ee_pose(self):
        t = rospy.Time.now()
        self._tf_listener.waitForTransform(self._base_frame, self._ee_frame, t, rospy.Duration(self._max_time_delay))
        curr_ee_positon, curr_ee_orientation = self._tf_listener.lookupTransform(self._base_frame, self._ee_frame, t)
        return curr_ee_positon, curr_ee_orientation
    
    def _tracking_cb(self, event):
        if len(self._trajectory) == 0:
            rospy.loginfo_throttle(1.0, "No trajectory to follow. Waiting for trajectory messages.")
            return
        print("Following trajectory with %d points." % len(self._trajectory))
        # get current ee pose
        curr_ee_positon, curr_ee_orientation = self._get_ee_pose()
        # find closest point on trajectory
        min_dist = np.inf
        min_ori_dist = np.inf
        min_idx = None

        for i, pose in enumerate(self._trajectory):
            dist = np.linalg.norm(curr_ee_positon - pose[0])
            _, ori_dist = quaternion_to_axis_angle(tft.quaternion_multiply(curr_ee_orientation, tft.quaternion_inverse(pose[1])))

            if dist < min_dist:
                min_dist = dist
                min_ori_dist = ori_dist
                min_idx = i

        print("Min dist: %f, min ori dist: %f" % (min_dist, min_ori_dist))

        if min_dist < self._distance_resolution and min_ori_dist < self._angle_resolution:
            # Point reached, remove everything up to this point
            self._trajectory = self._trajectory[(1+min_idx):]
            print("Point reached, %d points left." % len(self._trajectory))            
        if len(self._trajectory) == 0:
            return
        
        ref_pose = self._trajectory[0]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._base_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = ref_pose[0][0]
        pose_msg.pose.position.y = ref_pose[0][1]
        pose_msg.pose.position.z = ref_pose[0][2]
        pose_msg.pose.orientation.x = ref_pose[1][0]
        pose_msg.pose.orientation.y = ref_pose[1][1]
        pose_msg.pose.orientation.z = ref_pose[1][2]
        pose_msg.pose.orientation.w = ref_pose[1][3]
        self.target_pose_pub.publish(pose_msg)

        
    def _vis_trajectory(self, event):
        # publish waypoints as message to visualize in rviz
        all_points = MarkerArray()
        for i, pose in enumerate(self._trajectory):
            point = Marker()
            point.header.frame_id = self._base_frame
            point.header.stamp = rospy.Time.now()
            point.id = i
            point.type = Marker.ARROW
            point.action = Marker.ADD
            point.pose.position.x = pose[0][0]
            point.pose.position.y = pose[0][1]
            point.pose.position.z = pose[0][2]
            point.pose.orientation.x = pose[1][0]
            point.pose.orientation.y = pose[1][1]
            point.pose.orientation.z = pose[1][2]
            point.pose.orientation.w = pose[1][3]
            point.scale.x = 0.05
            point.scale.y = 0.01
            point.scale.z = 0.01
            point.color.a = 1.0
            point.color.r = 1.0
            point.color.g = 0.0
            point.color.b = 0.0
            all_points.markers.append(point)

        self._vis_traj_pub.publish(all_points)

if __name__ == "__main__":
    rospy.init_node("controller_node_sm", anonymous=True)
    node = TrackingNode()
    freq = rospy.get_param("~frequency", 50)
    rospy.Timer(rospy.Duration(1.0 / freq), node._tracking_cb)
    vis_freq = rospy.get_param("~vis_frequency", 10)
    rospy.Timer(rospy.Duration(1.0 / vis_freq), node._vis_trajectory)

    if rospy.get_param("~dummy_trajectory", False):
        published = False
        pub = rospy.Publisher("/ee_trajectory", MultiDOFJointTrajectory, queue_size=10)

        def pub_dummy_trajectory(event):
            global published
            if not published:
                rospy.loginfo("Publishing dummy trajectory")
                published = True
            else:
                return
            positions = np.linspace(0.5, 0.2, 10)
            msg = MultiDOFJointTrajectory()
            point = MultiDOFJointTrajectoryPoint()
            for pos in positions:
                pose = Transform()
                pose.translation.x = 0.5
                pose.translation.y = 0
                pose.translation.z = pos
                pose.rotation.x = 1.0
                point.transforms.append(pose)
            msg.points.append(point)
            pub.publish(msg)
        rospy.Timer(rospy.Duration(0.1), pub_dummy_trajectory)

    rospy.spin()