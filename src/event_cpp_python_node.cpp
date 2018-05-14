#include <ros/ros.h>
#include "event_cpp_python_node/event_cpp_python.h"
#include "event_cpp_python_node/event_cpp_python_live.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "event_based_stereo");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  bool read_from_bag = false;
  nh_priv.getParam("read_from_bag", read_from_bag);

  if (read_from_bag) {
      event_cpp_python_node::EventCppPython event_cpp_python(nh, nh_priv);
      
      std::string data_bag_path, gt_bag_path;
      nh_priv.getParam("data_bag_path", data_bag_path);
      nh_priv.getParam("gt_bag_path", gt_bag_path);
      
      event_cpp_python.playFromBag(data_bag_path, gt_bag_path);
  } else {
      event_cpp_python_node::EventCppPythonLive event_cpp_python(nh, nh_priv);
      ros::spin();
  }
}
