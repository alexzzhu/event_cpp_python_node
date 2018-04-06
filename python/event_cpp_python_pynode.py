#!/usr/bin/env python
import numpy as np
import math
import os
import cv2
import time
import rospy
from event_cpp_python import EventCppPython

def main():
    rospy.init_node("event_cpp_python")
    
    n_events_per_window = rospy.get_param("~n_events_per_window")   
    bag_folder_path = rospy.get_param("~bag_folder_path")
    sequence = rospy.get_param("~sequence")
    pub_event_arr = rospy.get_param("~pub_event_arr")
    
    event_cpp_python = EventCppPython(bag_folder_path,
                                      sequence,
                                      n_events_per_window,
                                      pub_event_arr)

    print("Setup done, waiting for events.")
    rospy.spin()

if __name__ == "__main__":
    main()
