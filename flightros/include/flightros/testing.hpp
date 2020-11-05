#pragma once

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// standard libraries
#include <assert.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/event_camera.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>

using namespace flightlib;

namespace testing {

class manual_timer {
  std::chrono::high_resolution_clock::time_point t0;
  double timestamp{0.0};

 public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }
  void stop() {
    timestamp = std::chrono::duration<double>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count() *
                1000.0;
  }
  const double &get() { return timestamp; }
};

// void setupQuad();
bool setUnity(const bool render);
bool connectUnity(void);
std::string type2str(int type);


// publisher
image_transport::Publisher rgb_pub_;
image_transport::Publisher diff_pub_;

cv::Mat rgb_image;


// unity quadrotor
std::shared_ptr<Quadrotor> quad_ptr_;
std::shared_ptr<RGBCamera> rgb_camera_;
std::shared_ptr<EventCamera> event_camera_;
QuadState quad_state_;

// Flightmare(Unity3D)
std::shared_ptr<UnityBridge> unity_bridge_ptr_;
SceneID scene_id_{UnityScene::WAREHOUSE};
bool unity_ready_{false};
bool unity_render_{true};
RenderMessage_t unity_output_;
uint16_t receive_id_{0};
}  // namespace testing