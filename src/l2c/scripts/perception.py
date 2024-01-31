#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from l2c.msg import NamedPoseArrayStamped, NamedPose
from geometry_msgs.msg import Pose
import tf

import cv2
import os
import yaml
import numpy as np

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []

    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

class PerceptionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('perception')

        self.flip_image = True  # TODO adjust if markers are detected and rejected

        # Load the URDF of the Franka robot (ensure the correct path)
        self.camera_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
        self.cubes_pose_publisher = rospy.Publisher('/perception/cubes/pose', NamedPoseArrayStamped, queue_size=1)

        dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dict_aruco, parameters)

        local_path = os.getcwd()
        dir_config = os.path.join(local_path, 'src/l2c/assets/calib_data_test/camera_calibration')
        with open(dir_config) as file:
            cam_calib_dist = yaml.load(file, Loader=yaml.Loader)
            print('load yaml')
            print(cam_calib_dist.keys())
            self.mtx = cam_calib_dist['mtx']
            self.dist = cam_calib_dist['dist']

    def camera_callback(self, msg):
        """
        Take camera image and return cube poses in camera frame (assumes 1 aruco marker per cube on top face)

        TODO: this currently outputs the aurco marker pose, not the cube pose
        """
        # convert msg.data into a numpy array
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        if self.flip_image:
            frame = cv2.flip(frame, 1)  # Pure magic, if you skip this line the markersa are rejected

        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('/home/antonio/Downloads/ros_tmp/image_raw.png', frame)
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))

        dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]


        # cv2.imwrite('/home/antonio/Downloads/ros_tmp/image_input.png', dst)
        
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(dst)
        # print(f"rejectedCandidates: {rejectedCandidates}")
        # print(f"markerCorners: {markerCorners}")

        if not markerIds is None:

            rvecs, tvecs, trash = my_estimatePoseSingleMarkers(markerCorners, 5.3, newcameramtx, self.dist)
            # rvecs can be converted to R with Rodrigues
            #cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
            for idx in range(len(markerIds)):

                cv2.drawFrameAxes(dst,self.mtx,self.dist,rvecs[idx],tvecs[idx],5)
                # print('marker id:%d, pos_x = %f,pos_y = %f, pos_z = %f' % (markerIds[idx],tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]))
                # print('marker id:%d, rot_x = %f,rot_y = %f, rot_z = %f' % (markerIds[idx],rvecs[idx][0],rvecs[idx][1],rvecs[idx][2]))

        cv2.aruco.drawDetectedMarkers(dst, markerCorners, markerIds)
        # print(markerIds)
        # cv2.imwrite('/home/antonio/Downloads/ros_tmp/markers.png', dst)

        if markerIds is None:
            print('no marker detected')
            return
        # Publish the messages
        pose_msg = NamedPoseArrayStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'camera'
        for idx, marker_id in enumerate(markerIds):
            named_pose = NamedPose()
            named_pose.name = str(marker_id[0])
            pose = Pose()
            pose.position.x = tvecs[idx][0]
            pose.position.y = tvecs[idx][1]
            pose.position.z = tvecs[idx][2]

            # r_matrix = cv2.Rodrigues(rvecs[idx])[0]
            # TODO check if euler angle to quaternion is correct
            quaternions = tf.transformations.quaternion_from_euler(rvecs[idx][0], rvecs[idx][1], rvecs[idx][2])

            pose.orientation.x = quaternions[0]
            pose.orientation.y = quaternions[1]
            pose.orientation.z = quaternions[2]
            pose.orientation.w = quaternions[3]

            named_pose.pose = pose
            pose_msg.named_poses.append(named_pose)
        self.cubes_pose_publisher.publish(pose_msg)

if __name__ == '__main__':
    node = PerceptionNode()
    rospy.spin()