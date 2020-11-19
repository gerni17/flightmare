#pragma once

#include <chrono>
#include <flightlib/common/types.hpp>
// #include <esim/visualization/publisher_interface.hpp>
#include <rosbag/bag.h>

#include <flightros/ros_utils.hpp>
// #include <esim/common/utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/transformation.hpp>
// #include <esim/visualization/ros_utils.hpp>
// #include <minkindr_conversions/kindr_msg.h>
// #include <minkindr_conversions/kindr_tf.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tf/tfMessage.h>


namespace flightros {

class RosbagWriter {
 public:
  RosbagWriter(const std::string& path_to_output_bag);
  RosbagWriter(const std::string& path_to_output_bag, int64_t stime);

  ~RosbagWriter();

  // virtual void imageCallback(const ImagePtrVector& images, int64_t t)
  // override;
  void eventsCallback(const EventsVector& events);
  void poseCallback(const ze::Transformation& T_W_C, int64_t t);
  void poseCallback(const flightlib::Quaternion& T_W_C, int64_t t);

  // virtual void twistCallback(const AngularVelocityVector& ws,
  //                            const LinearVelocityVector& vs,
  //                            int64_t t) override;
  // virtual void imuCallback(const Vector3& acc, const Vector3& gyr,
  //                          int64_t t) override;
  // virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig,
  //                                 int64_t t) override;


  // static RosbagWriter::Ptr createBagWriterFromGflags(size_t num_cameras);

 private:
  size_t num_cameras_;
  cv::Size sensor_size_;
  rosbag::Bag bag_;
  int64_t starting_time;
  const std::string topic_name_prefix_ = "";

  int64_t last_published_camera_info_time_;
  int64_t last_published_image_time_;
};

}  // namespace flightros
