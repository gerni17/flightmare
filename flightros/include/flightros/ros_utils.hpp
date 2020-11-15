#pragma once

#include <flightlib/common/types.hpp>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>

// #include <esim_msgs/OpticFlow.h>
using namespace flightlib;

namespace flightros {

inline std::string getTopicName(int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << "cam" << i << "/" << suffix;
  return ss.str();
}

inline std::string getTopicName(const std::string& prefix, int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << prefix << "/" << getTopicName(i, suffix);
  return ss.str();
}

inline ros::Time toRosTime(int64_t t)
{
  ros::Time ros_time;
  ros_time.fromNSec(t);
  return ros_time;
}


void imageToMsg(const cv::Mat_<ImageFloatType>& image, int64_t t, sensor_msgs::ImagePtr& msg);


void eventsToMsg(const EventsVector& events, int width, int height, dvs_msgs::EventArrayPtr& msg);

// sensor_msgs::Imu imuToMsg(const Vector3& acc, const Vector3& gyr, Time t);

// geometry_msgs::TwistStamped twistToMsg(const AngularVelocity& w, const LinearVelocity& v, int64_t t);

// void cameraToMsg(const ze::Camera::Ptr& camera, Time t, sensor_msgs::CameraInfoPtr& msg);


} // namespace event_camera_simulator
