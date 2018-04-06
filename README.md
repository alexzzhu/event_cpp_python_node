# Event C++ Python Node
This repository contains the basic ROS nodes to read events and other messages from the MVSEC dataset, convert them into image form, and publish to a python node, where further processing can occur. The separation between C++ and python exists as we observed a very slow processing time for python to process individual events (~20ms from events to an image). However, python is still necessary for GPU based operations such as Tensorflow, as well as for simpler prototyping. This package aims to resolve these issues, with hopefully minimal overhead from the ROS message transfer.

## Usage
The main C++ processing happens in ```src/event_cpp_python.cpp```, where the ```playFromBag``` function iterates over a pair of data and ground truth bags, and calls the corresponding callback function for each message. The file is organized as callback functions for simple integration into a subscriber based ROS node. 

Currently, there are two options for what the C++ side sends to the python side. If the param ```pub_event_arr``` in ```launch/event_cpp_python.launch``` is set to ```true```, a 4xN matrix will be sent. If it is set to ```false```, a rows x cols x 4 image will be sent (see our EV-FlowNet paper for the structure of this image).

The python node is mostly in ```python/event_cpp_python.py```. In particular, in ```_event_arr_callback``` if ```pub_event_arr``` is set to true, and ```_event_time_image_callback```. Any python processing of the input event message can occur there. Currently, it either prints the size of the event matrix or displays the image of the latest timestamp at each pixel using OpenCV.

Note that, while the C++ side is set up for stereo events, the python side only has structure for handling events from the left camera.
