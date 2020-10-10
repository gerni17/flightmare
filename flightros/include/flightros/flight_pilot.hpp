
#pragma once

#include <chrono>
#include <ctime>
#include <fstream>
#include <memory>
#include <vector>

#define ImageFloatType float

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/event_camera.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;
using OpticFlow = cv::Mat_<cv::Vec<ImageFloatType, 2>>;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightPilot();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);
  void writeCSV(std::string filename, cv::Mat m);
  void calcopticalFlow();
  void saveCSV();
  void saveImages();
  void calculateHist(cv::Mat bgr_planes[3]);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  image_transport::ImageTransport my_image_transport;

  // publisher
  image_transport::Publisher rgb_pub_;
  image_transport::Publisher of_pub_;
  image_transport::Publisher cv_pub_;

  // subscriber
  ros::Subscriber sub_state_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  std::shared_ptr<EventCamera> event_camera_;
    std::shared_ptr<EventCamera> event_camera_2;
  
  QuadState quad_state_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  cv::Mat prev, curr, bgr_, rgb_img;
  cv::Mat bgr[3];
  cv::Mat optical_flow_image;
  cv::Mat event_image;

  // auxiliary variables
  Scalar main_loop_freq_{50.0};
  int counter = 0;
};
}  // namespace flightros