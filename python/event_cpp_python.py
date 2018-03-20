#!/usr/bin/env python
import numpy as np
import math
import os
from ruamel.yaml import YAML

import rospy
from std_msgs.msg import Float64MultiArray
from event_cpp_python_node.msg import EventTimeImage

class EventCppPython:
    """
    ROS node that reads in events and poses, and returns events and velocities on demand.
    """
    def __init__(self,
                 bag_folder_path,
                 calib_file,
                 n_events_per_window,
                 image_size=[260, 346]):
        self._n_events_per_window = n_events_per_window
        self._image_size = image_size

        print("Loading calibration")
        self._load_calib(bag_folder_path, calib_file)

        self._t_start = None
    
        rospy.Subscriber("event_time_image",
                         EventTimeImage,
                         self._event_time_image_callback,
                         buff_size=10000000)
        
        print ("Setup done, waiting for events.")

    def _ros_array_to_np(self, ros_msg):
        n_rows = ros_msg.layout.dim[0].size
        n_cols = ros_msg.layout.dim[1].size
        arr_np = np.reshape(np.array(ros_msg.data),
                            (n_rows, n_cols))
        return arr_np
        
    def _event_time_image_callback(self, event_time_image_msg):
        time = rospy.Time.now()
        pos_count_img_msg = event_time_image_msg.pos_count_img
        neg_count_img_msg = event_time_image_msg.neg_count_img
        pos_time_img_msg = event_time_image_msg.pos_time_img
        neg_time_img_msg = event_time_image_msg.neg_time_img
        
        pos_count_img = self._ros_array_to_np(pos_count_img_msg)
        neg_count_img = self._ros_array_to_np(neg_count_img_msg)
        pos_time_img = self._ros_array_to_np(pos_time_img_msg)
        neg_time_img = self._ros_array_to_np(neg_time_img_msg)

        max_time = max(pos_time_img.max(), neg_time_img.max())
        min_time = min(pos_time_img[pos_time_img > 0].min(), neg_time_img[neg_time_img > 0].min())

        pos_time_img[pos_time_img > 0] = (pos_time_img[pos_time_img > 0] - min_time) \
                                         / (max_time - min_time)
        neg_time_img[neg_time_img > 0] = (neg_time_img[neg_time_img > 0] - min_time) \
                                         / (max_time - min_time)

        event_time_img = np.stack([pos_count_img, neg_count_img, pos_time_img, neg_time_img],
                                  axis=0)
        
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
    
    
