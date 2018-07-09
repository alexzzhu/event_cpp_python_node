# Event C++ Python Node
This repository contains the basic ROS nodes to read events and other messages from a live DAVIS camera or the MVSEC dataset, convert them into image form, and publish to a python node, where further processing can occur. The separation between C++ and python exists as we observed a very slow processing time for python to process individual events (~20ms from events to an image). However, python is still necessary for GPU based operations such as Tensorflow, as well as for simpler prototyping. This package aims to resolve these issues, with hopefully minimal overhead from the ROS message transfer.

To use this package, you must have the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) package compiled in your ROS workspace, and either run the davis_ros_driver package, or play or read a bag from the [MVSEC](https://daniilidis-group.github.io/mvsec/) dataset.

Note that this package is still in development, and will improve in terms of quality and documentation over time.

## Usage
This package supports two different methods of operation: reading directly from a ROS bag, or subscribing to events from a topic in real time. To use this package, you will need a calibration for the camera, in the yaml format defined by Kalibr: https://github.com/ethz-asl/kalibr/wiki/yaml-formats. To subscribe to events in real time, run: 

```roslaunch event_cpp_python_node event_cpp_python_node.launch read_from_bag:=false calibration_file:=${PATH_TO_CALIB_FILE}```

where ${PATH_TO_CALIB_FILE} points to your calibration yaml file. To read from a bag, run: 

```roslaunch event_cpp_python_node event_cpp_python_node.launch read_from_bag:=true calibration_file:=${PATH_TO_CALIB_FILE} data_bag_path:=${PATH_TO_DATA_BAG} gt_bag_path:=${PATH_TO_GT_BAG}```

where the arguments are once again the path to your data and gt bag from MVSEC.

Running these commands should succeed and print whenever events are received. Your code can then be entered in two places, depending on whether you are writing in C++ or python. For C++ code, [your code can be placed here for live streaming](https://github.com/alexzzhu/event_cpp_python_node/blob/a0acc31fa996368dce74b7aac3061fd90245520b/src/event_cpp_python_live.cpp#L95), [and here for reading from a bag.](https://github.com/alexzzhu/event_cpp_python_node/blob/a0acc31fa996368dce74b7aac3061fd90245520b/src/event_cpp_python.cpp#L169)

Note that, when reading from a bag, you also have access to other topics such as the gt pose, depth etc in the callback functions, e.g. depthImageCallback.

[To write code in python, place your code here.](https://github.com/alexzzhu/event_cpp_python_node/blob/a0acc31fa996368dce74b7aac3061fd90245520b/python/event_cpp_python.py#L56)
