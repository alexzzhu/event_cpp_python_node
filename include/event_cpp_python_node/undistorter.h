#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace event_cpp_python_node {
  class Undistorter {
  public:
    Undistorter(const ros::NodeHandle& nh, 
		const ros::NodeHandle& nh_priv,
		const std::string& cam);
    ~Undistorter();
    Undistorter(const Undistorter&) = delete;
    Undistorter& operator=(const Undistorter&) = delete;

    inline bool lookupDistorted(const float& x_rect, 
				      const float& y_rect, 
				      float *x, 
				      float *y) const {
      float x_rect_rounded = std::round(x_rect);
      float y_rect_rounded = std::round(y_rect);
      if (y_rect_rounded < 0 || y_rect_rounded < 0 || 
	  x_rect_rounded >= cols_ || y_rect_rounded >= rows_) 
	{
	return false;
	}
      int idx = (std::round(y_rect) * cols_ + std::round(x_rect)) * 2;
      *x = lookup_table_distorted_[idx];
      *y = lookup_table_distorted_[idx + 1];
      return true;
    }

    inline void lookup(int x, int y, float *xUndist, float *yUndist) const {
      int idx = (y * cols_ + x) * 2;
      *xUndist = lookup_table_[idx];
      *yUndist = lookup_table_[idx + 1];
    }

    inline void caminfo(float *fx, float *fy, float *px, float *py, float *Tx) const {
      *fx = fx_;
      *fy = fy_;
      *px = px_;
      *py = py_;
      *Tx = Tx_;
    }

    inline void resolution(int *rows, int *cols) const {
      *rows = rows_;
      *cols = cols_;
    }

   private:
    void initLookupTable();
    void loadCamInfo(const std::string& cam);
    std::string distortion_model_;
    ros::NodeHandle nh_, nh_priv_;
    float *lookup_table_{0};
    float *lookup_table_distorted_{0};
    int rows_, cols_;
    float fx_, fy_, px_, py_, Tx_;
    cv::Mat K_, P_, R_, distortion_coeffs_;
  };
} // End namespace.
