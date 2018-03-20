# Event C++ Python Node
This repository contains the basic ROS nodes to read events and other messages from the MVSEC dataset, convert them into image form, and publish to a python node, where further processing can occur. The separation between C++ and python exists as we observed a very slow processing time for python to process individual events (~20ms from events to an image). However, python is still necessary for GPU based operations such as Tensorflow, as well as for simpler prototyping. This package aims to resolve these issues, with hopefully minimal overhead from the ROS message transfer.

## Usage
The main C++ processing happens in ```src/event_cpp_python.cpp```, where the ```playFromBag``` function iterates over a pair of data and ground truth bags, and calls the corresponding callback function for each message. The file is organized as callback functions for simple integration into a subscriber based ROS node. 

To do any processing of the events in C++, you only need to edit the functions ```eventCallback``` and ```generateAndSendRequest```. ```eventCallback``` currently converts events into a time image, but can be modified to store them in a ```std::vector``` of ```Eigen::Vector4f```s. Any processing can then occur in ```generateAndSendRequest```. If you are publishing a different data format, then the publisher ```event_time_image_pub_```'s message type will also need be modified.

The python node is mostly in ```python/event_cpp_python.py```. In particular, in ```_event_time_image_callback```. Any python processing of the input event message can occur there. Currently, it displays the image of the latest timestamp at each pixel using OpenCV.
