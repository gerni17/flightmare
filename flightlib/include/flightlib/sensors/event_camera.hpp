#pragma once

#include <yaml-cpp/yaml.h>

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flightlib/common/logger.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/sensors/sensor_base.hpp"

namespace flightlib {

struct Event {
  int coord_x;
  int coord_y;
  int polarity;
  float time;
};


class EventCamera : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EventCamera();
  ~EventCamera();

  // public set functions
  bool setRelPose(const Ref<Vector<3>> B_r_BC, const Ref<Matrix<3, 3>> R_BC);
  bool setWidth(const int width);
  bool setHeight(const int height);
  bool setFOV(const Scalar fov);
  bool changeTime(float timestep);
  double getSimTime();
  // bool setDepthScale(const Scalar depth_scale);
  // bool setPostProcesscing(const std::vector<bool>& enabled_layers);
  bool feedImageQueue(const cv::Mat& image_mat);
  bool feedEventImageQueue(const cv::Mat& image_mat);
  bool feedEventQueue(const std::vector<Event_t>& events);


  // public get functions
  // std::vector<bool> getEnabledLayers(void) const;
  Matrix<4, 4> getRelPose(void) const;
  int getChannels(void) const;
  int getWidth(void) const;
  int getHeight(void) const;
  Scalar getFOV(void) const;
  // Scalar getDepthScale(void) const;
  bool getRGBImage(cv::Mat& rgb_img);
  // bool getDepthMap(cv::Mat& depth_map);
  // bool getSegmentation(cv::Mat& segmentation);
  // bool getOpticalFlow(cv::Mat& opticalflow);
  bool getEventImages(cv::Mat& image_mat);
  std::vector<Event_t> getEvents(); 
  cv::Mat createEventimages();
  // bool getEvents(std::vector<Event>& events);


  // auxiliary functions
  // void enableDepth(const bool on);
  // void enableOpticalFlow(const bool on);
  // void enableSegmentation(const bool on);

 private:
  Logger logger_{"RBGCamera"};

  // camera parameters
  int channels_;
  int width_;
  int height_;
  Scalar fov_;
  // Scalar depth_scale_;
  double sim_time = 0.0;

  // Camera relative
  Vector<3> B_r_BC_;
  Matrix<4, 4> T_BC_;

  // image data buffer
  std::mutex queue_mutex_;
  const int queue_size_ = 1;

  std::deque<cv::Mat> rgb_queue_;
  std::deque<cv::Mat> event_image_queue_;
  std::deque<std::vector<Event_t>> event_queue_;
  std::vector<Event_t> event_queue_for_img;
  std::vector<Event_t> event_queue_for_test;


  // std::deque<cv::Mat> opticalflow_queue_;
  // std::deque<flightlib::Event> event_queue_;

  // [depth, segmentation, optical flow]
  // std::vector<bool> enabled_layers_;
};

}  // namespace flightlib
