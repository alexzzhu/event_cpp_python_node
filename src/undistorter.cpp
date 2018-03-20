#include "event_cpp_python_node/undistorter.h"

#include <fstream>

//#define DEBUG

namespace event_cpp_python_node {
  Undistorter::Undistorter(const ros::NodeHandle& nh,
			   const ros::NodeHandle& nh_priv,
			   const std::string& cam)
    : nh_(nh),
      nh_priv_(nh_priv),
      K_(cv::Mat::eye(3, 3, CV_32F)),
      P_(cv::Mat::zeros(3, 4, CV_32F)),
      R_(cv::Mat::zeros(3, 3, CV_32F)) {
    loadCamInfo(cam);
    initLookupTable();
  } // End Undistorter constructor.

  Undistorter::~Undistorter() {
    delete[] lookup_table_;
    delete[] lookup_table_distorted_;
  } // End Undistorter destructor.

  void Undistorter::loadCamInfo(const std::string& cam) {
    if (nh_priv_.hasParam(cam)) {
      nh_priv_.getParam(cam+"/distortion_model", distortion_model_);
      std::vector<float> resolution;
      nh_priv_.getParam(cam+"/resolution", resolution);
      cols_ = resolution[0];
      rows_ = resolution[1];
      std::vector<float> distortion_coeffs, intrinsics;
      nh_priv_.getParam(cam+"/distortion_coeffs", distortion_coeffs);
      nh_priv_.getParam(cam+"/intrinsics", intrinsics);
      cv::transpose(cv::Mat(distortion_coeffs), distortion_coeffs_);
      K_.at<float>(0, 0) = intrinsics[0];
      K_.at<float>(1, 1) = intrinsics[1];
      K_.at<float>(0, 2) = intrinsics[2];
      K_.at<float>(1, 2) = intrinsics[3];

      XmlRpc::XmlRpcValue lines;
      nh_priv_.getParam(cam+"/projection_matrix", lines);
      if (lines.size() != 3 ||
	  lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
	ROS_ERROR("invalid data in calib file projection matrix");
	return;
      }

      for (int i = 0; i < lines.size(); i++) {
	for (int j = 0; j < lines[i].size(); j++) {
	  if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
	    ROS_ERROR_STREAM("bad value in calib file projection");
	  } else {
	    P_.at<float>(i, j) =
              static_cast<float>(static_cast<double>(lines[i][j]));
	  }
	}
      }

      nh_priv_.getParam(cam+"/rectification_matrix", lines);
      if (lines.size() != 3 ||
	  lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
	ROS_ERROR("invalid data in calib file rectification matrix");
	return;
      }

      for (int i = 0; i < lines.size(); i++) {
	for (int j = 0; j < lines[i].size(); j++) {
	  if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
	    ROS_ERROR_STREAM("bad value in calib file rectification");
	  } else {
	    R_.at<float>(i, j) =
              static_cast<float>(static_cast<double>(lines[i][j]));
	  }
	}
      }

      fx_ = P_.at<float>(0, 0);
      fy_ = P_.at<float>(1, 1);
      px_ = P_.at<float>(0, 2);
      py_ = P_.at<float>(1, 2);
      Tx_ = P_.at<float>(0, 3);
    } else {
      ROS_ERROR("Camera calibration params for camera %s not loaded into params.", cam.c_str());
    }
  }

#ifdef DEBUG	
  static void dumpTable(float *table, int w, int h) {
    std::ofstream filex("table_x.txt");
    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
	int idx = 2 * (i * w + j);
	filex << " " << table[idx];
      }
      filex << std::endl;
    }
    std::ofstream filey("table_y.txt");
    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
	int idx = 2 * (i * w + j) + 1;
	filey << " " << table[idx];
      }
      filey << std::endl;
    }
  } // End dumpTable.
#endif

  void Undistorter::initLookupTable() {
    if (lookup_table_ != NULL) {
      ROS_WARN("%s: duplicate initialization of undistort map!!!",
	       nh_priv_.getNamespace().c_str());
      delete[] lookup_table_;
    }
    if (lookup_table_distorted_ != NULL) {
      ROS_WARN("%s: duplicate initialization of distort map!!!",
               nh_priv_.getNamespace().c_str());
      delete[] lookup_table_distorted_;
    }

    int npoints = rows_ * cols_;
    lookup_table_ = new float[npoints * 2];  // one for x, one for y

    std::vector<cv::Point2f> dist_points, undist_points;
    for (int row = 0; row < rows_; row++) {
      for (int col = 0; col < cols_; col++) {
        cv::Point2f distPt((float)col, (float)row);
        dist_points.push_back(distPt);
      }
    }

    if (distortion_model_ == "plumb_bob" || distortion_model_ == "radtan") {
      cv::undistortPoints(dist_points, undist_points, K_, distortion_coeffs_,
                          R_, P_);
    } else if (distortion_model_ == "equidistant") {
      cv::fisheye::undistortPoints(dist_points, undist_points, K_,
                                   distortion_coeffs_, R_, P_);
    } else {
      ROS_ERROR("Currently, only equidistant and plumb_bob distortion models are supported");
    }
    
    for (int iter = 0; iter < dist_points.size(); iter++) {
      int idx = iter * 2;
      int row = (int)dist_points[iter].y;
      int col = (int)dist_points[iter].x;
      float x = undist_points[iter].x;
      float y = undist_points[iter].y;
      lookup_table_[idx] = (float)x;
      lookup_table_[idx + 1] = (float)y;
    }

    // Build mapping from undistorted to distorted.
    int npoints_dist = rows_ * cols_;

    lookup_table_distorted_ = new float[npoints_dist * 2];

    std::vector<cv::Point3f> norm_points;
    dist_points.clear();
    undist_points.clear();

    cv::Mat R_transpose;
    cv::transpose(R_, R_transpose);

    for (int y = 0; y < cols_; y++) {
      for (int x = 0; x < rows_; x++) {
        float x_norm = (x - px_) / fx_;
        float y_norm = (y - py_) / fy_;

        cv::Mat normPtRot(3, 1, cv::DataType<float>::type);
        normPtRot.at<float>(0) = x_norm;
        normPtRot.at<float>(1) = y_norm;
        normPtRot.at<float>(2) = 1.0;
        cv::Mat normPt = R_transpose * normPtRot;

        cv::Point3f undistPt(normPt.at<float>(0) / normPt.at<float>(2),
                             normPt.at<float>(1) / normPt.at<float>(2), 1.0);

        norm_points.push_back(undistPt);
        undist_points.push_back(cv::Point2f(x, y));
      }
    }

    cv::Mat rvec = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
    cv::Mat tvec = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);

    if (distortion_model_ == "plumb_bob" || distortion_model_ == "radtan") {
      cv::projectPoints(norm_points, rvec, tvec, K_, distortion_coeffs_,
                        dist_points);
    } else if (distortion_model_ == "equidistant") {
      cv::fisheye::projectPoints(norm_points, dist_points, rvec, tvec, K_,
                                 distortion_coeffs_);
    } else {
      ROS_ERROR("Currently, only equidistant and plumb_bob distortion models are supported");
    }

    for (int iter = 0; iter < undist_points.size(); iter++) {
      // int idx = iter * 2;
      int y = std::round(undist_points[iter].y);
      int x = std::round(undist_points[iter].x);
      if (x < 0 || y < 0 || x >= cols_ || y >= rows_) continue;

      int idx = (y * cols_ + x) * 2;

      float x_dist = dist_points[iter].x;
      float y_dist = dist_points[iter].y;

      lookup_table_distorted_[idx] = x_dist;
      lookup_table_distorted_[idx + 1] = y_dist;
    }
#ifdef DEBUG
    dumpTable(lookup_table_, width_, height_);
#endif
  } // End initLookupTable.
} // End namespace.
