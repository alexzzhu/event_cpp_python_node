#!/usr/bin/env python
import numpy as np
import math
import os
from ruamel.yaml import YAML

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

class EventCppPython:
    """
    ROS node that reads in events and poses, and returns events and velocities on demand.
    """
    def __init__(self,
                 bag_folder_path,
                 calib_file,
                 n_events_per_window,
                 pub_event_arr=True,
                 image_size=[260, 346]):
        self._n_events_per_window = n_events_per_window
        self._image_size = image_size

        print("Loading calibration")
        self._load_calib(bag_folder_path, calib_file)

        self._t_start = None
        
        self._bridge = CvBridge()
        if pub_event_arr:
            rospy.Subscriber("left/event_arr",
                             Float64MultiArray,
                             self._event_arr_callback,
                             buff_size=10000000)
        else:
            rospy.Subscriber("left/event_time_image",
                             Image,
                             self._event_time_image_callback,
                             buff_size=10000000)
        
        print ("Setup done, waiting for events.")

    def _ros_array_to_np(self, ros_msg):
        n_rows = ros_msg.layout.dim[0].size
        n_cols = ros_msg.layout.dim[1].size
        arr_np = np.reshape(np.array(ros_msg.data),
                            (n_rows, n_cols))
        return arr_np

    def _event_arr_callback(self, event_arr_msg):
        events_np = np.reshape(np.array(event_arr_msg.data),
                               (4, event_arr_msg.layout.dim[1].size))
        print("Received {} events".format(events_np.shape[1]))
        return
    
    def _event_time_image_callback(self, event_time_image_msg):       
        try:
            event_time_image = self._bridge.imgmsg_to_cv2(event_time_image_msg, "32FC4")
        except CvBridgeError as e:
            print(e)
            
        cv2.imshow("Event time image", np.amax(event_time_image[:, :, :2], axis=2))
        cv2.waitKey(1)
        # Do something with event_time_image
        return
        
    def _load_calib(self, bag_folder, calib_file):
        self._left_x_rect_map = np.loadtxt(os.path.join(bag_folder,
                                                       "{}_left_x_map.txt".format(calib_file)))
        self._left_y_rect_map = np.loadtxt(os.path.join(bag_folder,
                                                       "{}_left_y_map.txt".format(calib_file)))
        self._right_x_rect_map = np.loadtxt(os.path.join(bag_folder,
                                                       "{}_right_x_map.txt".format(calib_file)))
        self._right_y_rect_map = np.loadtxt(os.path.join(bag_folder,
                                                       "{}_right_y_map.txt".format(calib_file)))

        yaml = YAML()
        
        with open(os.path.join(bag_folder,"camchain-imucam-{}.yaml".format(calib_file)), 'r') as f:
            calib = yaml.load(f)
        left_P = calib['cam0']['projection_matrix']
        right_P = calib['cam1']['projection_matrix']

        self._left_intrinsics = np.array([left_P[0][0], 
                                          left_P[1][1], 
                                          left_P[0][2], 
                                          left_P[1][2]])
        self._right_intrinsics = np.array([right_P[0][0], 
                                           right_P[1][1], 
                                           right_P[0][2], 
                                           right_P[1][2]])
        self._baseline = right_P[0][3]
    
    
