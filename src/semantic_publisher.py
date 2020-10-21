#!/usr/bin/env python
import os
from cv_bridge import CvBridge, CvBridgeError
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import imutils
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import rospy
from sensor_msgs.msg import Image, PointField, PointCloud2, CompressedImage
#from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
import ros_numpy

class SemanticPublisher():
    rospy.init_node('semantic_publisher')
    rate = rospy.Rate(10)
    
 
    bridge = CvBridge()
    veh_pose = PoseStamped()
    x_origin = 1369.04968262
    y_origin = 562.848144531
    resolution = 0.2
    print("Loading Map...")
    #bev_map = cv2.imread("/home/dfpazr/Documents/CogRob/avl/semantic_navigation/bev.jpg")
    bev_map = cv2.imread("/home/dfpazr/Documents/CogRob/avl/TritonNet/iros_psm_ws/src/vision_semantic_segmentation/outputs/cfn_mtx_with_intensity/version_10/global_map.png")
    print("Map Loaded")
    local_map_pub = rospy.Publisher('/semantic_pose', Image, queue_size=10)
    def __init__(self):
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
    

        while not rospy.is_shutdown():
            self.rate.sleep()
    def get_yaw(self, quat):
        r = R.from_quat(quat)
        #rot_vec = r.as_rotvec()
        rot_vec = r.as_euler('zyx', degrees=True)
        #print("Current heading: " + str(rot_vec[0]))
        return rot_vec[0]       
         
    def pose_callback(self, msg):
        self.veh_pose = msg.pose
        # Extract vehicle heading
        quat = np.array([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])
        current_yaw = self.get_yaw(quat)

        pose_u_pix = int((msg.pose.position.x + self.x_origin) /self.resolution)
        pose_v_pix = int((msg.pose.position.y + self.y_origin) /self.resolution)
        #print(self.bev_map.shape)
        local_map = self.bev_map[pose_u_pix-200:pose_u_pix+200, pose_v_pix-200:pose_v_pix+200,:]
        local_map = np.copy(local_map)
        print("Yaw: " + str(180-current_yaw))
        # positive angle represents a CC rotation 
        local_map = imutils.rotate(local_map, angle=180-current_yaw)
        local_map[195:205, 195:205] = np.array([0,0,255])
        local_map_msg = self.bridge.cv2_to_imgmsg(local_map)
        self.local_map_pub.publish(local_map_msg)


if __name__ == "__main__":
    SemanticPublisher() 
