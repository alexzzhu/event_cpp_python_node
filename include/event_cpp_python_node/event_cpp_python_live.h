#pragma once

// Standard
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>

// Eigen
#include <Eigen/Dense>
// For aligned_allocators.
#include <Eigen/StdVector>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>

// Message filter subscribers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/eigen.hpp>

// Custom
#include "event_cpp_python_node/undistorter.h"

namespace event_cpp_python_node {
class EventCppPythonLive{
public:
    EventCppPythonLive(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~EventCppPythonLive();
private:
    // ROS stuff.
    ros::NodeHandle nh_;
    ros::Publisher left_event_arr_pub_, right_event_arr_pub_;
    ros::Subscriber left_event_sub_, right_event_sub_;

    image_transport::Publisher left_event_time_image_pub_, right_event_time_image_pub_;
    image_transport::ImageTransport it_;

    // For live subscribers.
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            sensor_msgs::Image> ApproxSyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<ApproxSyncPolicy> sync_;
    
    // Objects.
    // Stores camera info, rectifies images.
    Undistorter left_undistorter_, right_undistorter_;
    // Variables.
    // Size of the image.
    int rows_, cols_;
    int iter = 1;
    float baseline_;
    float start_time_;
    int n_ima_ = 0;
    int n_events_per_window_;
    int n_events_ = 0;
    // Time of first event.
    ros::Time t_start_ = ros::Time(0);
    bool pub_event_arr_ = true;
    bool started_ = false;
    Eigen::Vector4f intrinsics_;
    cv::Mat left_timestamp_image_, right_timestamp_image_;
    
    std::vector<Eigen::Vector4f, 
                Eigen::aligned_allocator<Eigen::Vector4f>> left_event_vec_, right_event_vec_;
    
    void generateEventMatrix(const std::vector<Eigen::Vector4f, 
                             Eigen::aligned_allocator<Eigen::Vector4f>>& event_vec,
                             Eigen::MatrixXf& events);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_img_msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void eventCallback(const dvs_msgs::EventArray::ConstPtr& event_msg,
		       const Undistorter& undistorter,
                       const ros::Publisher& event_arr_pub,
                       const image_transport::Publisher& event_time_image_pub,
		       std::vector<Eigen::Vector4f, 
		       Eigen::aligned_allocator<Eigen::Vector4f>>& event_vec,
                       cv::Mat& event_timestamp_image);
    void generateAndSendRequest(const ros::Publisher& event_arr_pub,
                                const std::vector<Eigen::Vector4f,
                                Eigen::aligned_allocator<Eigen::Vector4f>> event_vec);
    void waitAndProcessResponse(const cv::Mat& gt_depth, const Eigen::MatrixXf& left_events);
    void disparityResultsCallback(const sensor_msgs::ImageConstPtr disparity_msg,
                                  const sensor_msgs::ImageConstPtr deblurred_image_msg);
    void poseDepthCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                           const sensor_msgs::Image::ConstPtr& depth_img_msg);
};
} // namespace
