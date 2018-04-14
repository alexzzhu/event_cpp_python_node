#include <ros/ros.h>
#include "event_cpp_python_node/event_cpp_python.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "event_based_stereo");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  event_cpp_python_node::EventCppPython event_cpp_python(nh, nh_priv);
  
  ros::spin();
}
