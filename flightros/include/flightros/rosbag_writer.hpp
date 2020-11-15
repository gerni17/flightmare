#pragma once

#include <flightlib/common/types.hpp>

// #include <esim/visualization/publisher_interface.hpp>
#include <rosbag/bag.h>
#include <flightros/ros_utils.hpp>


namespace flightros {

class RosbagWriter 
{
public:
  RosbagWriter(const std::string& path_to_output_bag);
  ~RosbagWriter();

  // virtual void imageCallback(const ImagePtrVector& images, int64_t t) override;
  void eventsCallback(const EventsVector& events) ;
  // virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, int64_t t) override;
  // virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, int64_t t) override;
  // virtual void imuCallback(const Vector3& acc, const Vector3& gyr, int64_t t) override;
  // virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, int64_t t) override;


  // static RosbagWriter::Ptr createBagWriterFromGflags(size_t num_cameras);

private:
  size_t num_cameras_;
  cv::Size sensor_size_;
  rosbag::Bag bag_;

  const std::string topic_name_prefix_ = "";

  int64_t last_published_camera_info_time_;
  int64_t last_published_image_time_;

};

} // namespace event_camera_simulator
