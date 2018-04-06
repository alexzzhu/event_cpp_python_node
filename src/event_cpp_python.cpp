#include "event_cpp_python_node/event_cpp_python.h"

namespace event_cpp_python_node {
EventCppPython::~EventCppPython()
{
    ROS_INFO("event_cpp_python shutting down");
} // End EventCppPython destructor
  
EventCppPython::EventCppPython(ros::NodeHandle nh,
                               ros::NodeHandle nh_priv)
    : nh_(nh),
      it_(nh),
      left_undistorter_(nh, nh_priv, "cam0"),
      right_undistorter_(nh, nh_priv, "cam1")
{ 
    nh_priv.getParam("n_events_per_window", n_events_per_window_);
    nh_priv.getParam("start_time", start_time_);
    nh_priv.getParam("pub_event_arr", pub_event_arr_);
    
    XmlRpc::XmlRpcValue lines;
    nh_priv.getParam("cam1/projection_matrix", lines);
    baseline_ = static_cast<double>(lines[0][3]);
  
    left_undistorter_.resolution(&rows_, &cols_);

    if (pub_event_arr_) {
        left_event_arr_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("left/event_arr", 1000);
        right_event_arr_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("right/event_arr", 1000);
    } else {
        left_timestamp_image_ = cv::Mat::zeros(rows_, cols_, CV_32FC4);
        left_event_time_image_pub_ = it_.advertise("left/event_time_image", 1000);
        right_timestamp_image_ = cv::Mat::zeros(rows_, cols_, CV_32FC4);
        right_event_time_image_pub_ = it_.advertise("right/event_time_image", 1000);
    }
  
    ROS_INFO("event_cpp_python initialized.");
} // End EventCppPython constructor.
  
void EventCppPython::playFromBag(const std::string& data_bag_path,
                                 const std::string& gt_bag_path) 
{
    ROS_INFO("Reading from bags %s, %s\n", data_bag_path.c_str(), gt_bag_path.c_str());
  
    // Open bags.
    rosbag::Bag data_bag, gt_bag;
    try {
        data_bag.open(data_bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        ROS_ERROR_STREAM("cannot open bag: " << data_bag_path << " due to: " << e.what());
        return;
    }
  
    try {
        gt_bag.open(gt_bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        ROS_ERROR_STREAM("cannot open bag: " << gt_bag_path << " due to: " << e.what());
        return;
    }
  
    // Specify desired topics.
    std::string left_events("/davis/left/events"),
        right_events("/davis/right/events"),
        pose("/davis/left/odometry"),
        depth_image("/davis/left/depth_image_rect"),
        imu("/davis/left/imu");
    std::vector<std::string> data_topics = { left_events,
                                             right_events,
                                             imu };
    std::vector<std::string> gt_topics = { pose, depth_image };
  
    // Create a view that accesses both bags.
    rosbag::View view(data_bag, rosbag::TopicQuery(data_topics));
    view.addQuery(gt_bag, rosbag::TopicQuery(gt_topics));
  
    ros::Time bag_start_time = view.getBeginTime();
    ros::Rate rate(20);
    
    // Loop over bags in each topic sequentially in time.
    for (rosbag::MessageInstance const m: view) {
        if (!ros::ok()) break;
    
        if (m.getTime() < bag_start_time + ros::Duration(start_time_)) {
            continue;
        }
    
        const std::string &topic = m.getTopic();
      
        if (topic.find("events") != std::string::npos) {
            dvs_msgs::EventArray::ConstPtr event_ptr = 
                m.instantiate<dvs_msgs::EventArray>();
            if (event_ptr) {
                if (topic == left_events) {
                    eventCallback(event_ptr,
                                  left_undistorter_,
                                  left_event_arr_pub_,
                                  left_event_time_image_pub_,
                                  left_event_vec_,
                                  left_timestamp_image_);
                }
                // Uncomment this if you want stereo event processing.
                /*
                  else {
                  eventCallback(event_ptr,
                  right_undistorter_,
                  right_event_arr_pub_,
                  right_event_time_image_pub_,
                  right_event_vec_,
                  right_timestamp_image_
                  );
                  }
                */
            }
        }

        if (topic == imu) {
            sensor_msgs::Imu::ConstPtr imu_ptr =
                m.instantiate<sensor_msgs::Imu>();
            if (imu_ptr) {
                imuCallback(imu_ptr);
            }
        }
    
        if (topic == pose) {
            geometry_msgs::PoseStamped::ConstPtr pose_ptr = 
                m.instantiate<geometry_msgs::PoseStamped>();
            if (pose_ptr) {
                poseCallback(pose_ptr);
            }
        }
    
        if (topic == depth_image) {
            sensor_msgs::Image::ConstPtr depth_img_ptr =
                m.instantiate<sensor_msgs::Image>();
            if (depth_img_ptr) {
                depthImageCallback(depth_img_ptr);
                rate.sleep();
            }
        }
    }
}

// Grabs the last n_events_per_window_ events from a vector and puts it in an Eigen MatrixXf.
void EventCppPython::generateEventMatrix(const std::vector<Eigen::Vector4f, 
                                         Eigen::aligned_allocator<Eigen::Vector4f>>& event_vec,
                                         Eigen::MatrixXf& events)
{
    if (events.cols() != n_events_per_window_) {
        ROS_ERROR("Error: You need to set the size of the input events correctly before calling generateEventMatrix");
        ros::shutdown();
    }

    for (int i=0; i < n_events_per_window_; ++i) {
        events.col(i) = event_vec[i];
    }

    return;
}

// Converts an Eigen::MatrixXf into a std_msgs::Float64MultiArray message and publishes it.
void EventCppPython::generateAndSendRequest(const ros::Publisher& event_arr_pub,
                                            const std::vector<Eigen::Vector4f,
                                            Eigen::aligned_allocator<Eigen::Vector4f>> event_vec)
{
    Eigen::MatrixXf event_mat(4, n_events_per_window_);;
    generateEventMatrix(event_vec, event_mat);
    std_msgs::Float64MultiArray event_arr_msg;
    tf::matrixEigenToMsg(event_mat.cast<double>().eval(), event_arr_msg);
    event_arr_pub.publish(event_arr_msg);
}
  
// Receives event messages, rectifies them, and then stores them in the appropriate vector.
void EventCppPython::eventCallback(const dvs_msgs::EventArray::ConstPtr& event_msg,
                                   const Undistorter& undistorter,
                                   const ros::Publisher& event_arr_pub,
                                   const image_transport::Publisher& event_time_image_pub,
                                   std::vector<Eigen::Vector4f, 
                                   Eigen::aligned_allocator<Eigen::Vector4f>>& event_vec,
                                   cv::Mat& event_timestamp_image)    
{
    // If no events, just return.
    if (event_msg->events.size() == 0) return;

    const std::vector<dvs_msgs::Event>& events = event_msg->events;
    
    ros::Time t_init = events[0].ts;
    if (!started_) {
        t_start_ = t_init;
        started_ = true;
        ROS_INFO("Events received at time %f", t_start_.toSec());
    }
    
    for (const auto& event : events) {
        // Rectify the event.
        float x_rect, y_rect;
        undistorter.lookup(event.x, event.y, &x_rect, &y_rect);

        x_rect = round(x_rect);
        y_rect = round(y_rect);

        // Only keep events inside the image after rectification.
        if (x_rect < 0 || x_rect >= cols_ || y_rect < 0 || y_rect >= rows_) {
            continue;
        }

        float t = static_cast<float>((event.ts - t_start_).toSec());
        int pol = event.polarity ? 1:-1;
        
        // Store the event in event_vec.
        if (pub_event_arr_) {
            event_vec.push_back(Eigen::Vector4f(x_rect, y_rect, t, pol));
        } else {
            cv::Vec4f& pixel = event_timestamp_image.at<cv::Vec4f>(y_rect, x_rect);
            if (pol == 1) {
                pixel.val[2] = t;
                ++pixel.val[0];
            } else {
                pixel.val[3] = t;
                ++pixel.val[1];
            }
        }
    }

    n_events_ += events.size();

    // If we have enough events, publish and reset event images.
    if (n_events_ > n_events_per_window_) {
        if (pub_event_arr_) {
            generateAndSendRequest(event_arr_pub, event_vec);
            event_vec.erase(event_vec.begin(), event_vec.begin() + n_events_per_window_);
        } else {
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                                 "32FC4",
                                                                 event_timestamp_image)
                .toImageMsg();
            image_msg->header.stamp = events.back().ts;
            event_time_image_pub.publish(image_msg);
            event_timestamp_image = cv::Mat::zeros(rows_, cols_, CV_32FC4);
        }
        n_events_ = 0;
    }
  
    return;
} // End eventCallback

// Ground truth depth image message callback. Put whatever you want here.
void EventCppPython::depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_img_msg)
{ 
    return;
}

// IMU message callback. Put whatever you want here.
void EventCppPython::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    return;
}

// GT pose message callback. Put whatever you want here.
void EventCppPython::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    return;
}

} // End namespace
