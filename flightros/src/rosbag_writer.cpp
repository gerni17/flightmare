#include <flightros/rosbag_writer.hpp>
// #include <esim/common/utils.hpp>
// #include <ze/common/time_conversions.hpp>
// #include <esim/visualization/ros_utils.hpp>
// #include <minkindr_conversions/kindr_msg.h>
// #include <minkindr_conversions/kindr_tf.h>
#include <tf/tfMessage.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DECLARE_double(ros_publisher_camera_info_rate);
DECLARE_double(ros_publisher_frame_rate);
DECLARE_double(ros_publisher_depth_rate);
DECLARE_double(ros_publisher_pointcloud_rate);
DECLARE_double(ros_publisher_optic_flow_rate);

DEFINE_string(path_to_output_bag, "",
"Path to which save the output bag file.");

namespace flightros {
  

RosbagWriter::RosbagWriter(const std::string& path_to_output_bag)
{
  // CHECK_GE(num_cameras, 1);
  num_cameras_ = 1;
  sensor_size_ = cv::Size(100,100);
  // cv::MatSize size()=100;
  // std::string path_to_output_bag= "pat/h";
  starting_time = 1;
  try
  {
    bag_.open(path_to_output_bag, rosbag::bagmode::Write);
  }
  catch(rosbag::BagIOException e)
  {
    LOG(FATAL) << "Error: could not open rosbag: " << FLAGS_path_to_output_bag << std::endl;
    return;
  }

  // LOG(INFO) << "Will write to bag: " << path_to_output_bag;

  last_published_camera_info_time_ = 0;
  last_published_image_time_ = 0;
}

// Publisher::Ptr RosbagWriter::createBagWriterFromGflags(size_t num_cameras)
// {
//   if(FLAGS_path_to_output_bag == "")
//   {
//     LOG(INFO) << "Empty output bag string: will not write to rosbag";
//     return nullptr;
//   }

//   return std::make_shared<RosbagWriter>(FLAGS_path_to_output_bag, num_cameras);
// }

RosbagWriter::~RosbagWriter()
{
  LOG(INFO) << "Finalizing the bag...";
  bag_.close();
  LOG(INFO) << "Finished writing to bag: " << FLAGS_path_to_output_bag;
}



// void RosbagWriter::imageCallback(const ImagePtrVector& images, int64_t t)
// {
//   for(size_t i=0; i<num_cameras_; ++i)
//   {
//     sensor_sizes_[i] = images[i]->size();

//     static const Duration min_time_interval_between_published_images_
//         = ze::secToNanosec(1.0 / FLAGS_ros_publisher_frame_rate);
//     if(last_published_image_time_ > 0 && t - last_published_image_time_ < min_time_interval_between_published_images_)
//     {
//       return;
//     }

//     if(images[i])
//     {
//       sensor_msgs::ImagePtr msg;
//       imageToMsg(*images[i], t, msg);
//       bag_.write(getTopicName(topic_name_prefix_, i, "image_raw"),
//                  msg->header.stamp, msg);
//     }
//   }
//   last_published_image_time_ = t;
// }





void RosbagWriter::eventsCallback(const EventsVector& events)
{

    if(sensor_size_.width == 0 || sensor_size_.height == 0)
    {
      ROS_ERROR_STREAM("width to small");
      return;
    }

    if(events.empty())
    {
      ROS_ERROR_STREAM("empty ");
      return;
    }
    ROS_ERROR_STREAM("event callback");
    dvs_msgs::EventArrayPtr msg;
    msg.reset(new dvs_msgs::EventArray);
    eventsToMsg(events, sensor_size_.width, sensor_size_.height, msg,starting_time);

    bag_.write(getTopicName(topic_name_prefix_, 0, "events"),
               msg->header.stamp, msg);
}

// void RosbagWriter::poseCallback(const Transformation& T_W_B,
//                                 const TransformationVector& T_W_Cs,
//                                 cv::Mat_<ImageFloatType> t)
// {
//   if(T_W_Cs.size() != num_cameras_)
//   {
//     LOG(WARNING) << "Number of poses is different than number of cameras."
//                  << "Will not output poses.";
//     return;
//   }
//   geometry_msgs::PoseStamped pose_stamped_msg;
//   geometry_msgs::TransformStamped transform_stamped_msg;
//   transform_stamped_msg.header.frame_id = "map";
//   transform_stamped_msg.header.stamp = toRosTime(t);
//   tf::tfMessage tf_msg;

//   for(size_t i=0; i<num_cameras_; ++i)
//   {
//     // Write pose to bag
//     tf::poseStampedKindrToMsg(T_W_Cs[i], toRosTime(t), "map", &pose_stamped_msg);
//     bag_.write(getTopicName(topic_name_prefix_, i, "pose"),
//                toRosTime(t), pose_stamped_msg);

//     // Write tf transform to bag
//     std::stringstream ss; ss << "cam" << i;
//     transform_stamped_msg.child_frame_id = ss.str();
//     tf::transformKindrToMsg(T_W_Cs[i], &transform_stamped_msg.transform);
//     tf_msg.transforms.push_back(transform_stamped_msg);
//   }

//   transform_stamped_msg.child_frame_id = "body";
//   tf::transformKindrToMsg(T_W_B, &transform_stamped_msg.transform);
//   tf_msg.transforms.push_back(transform_stamped_msg);

//   bag_.write("/tf", toRosTime(t), tf_msg);
// }

// void RosbagWriter::twistCallback(const AngularVelocityVector &ws, const LinearVelocityVector &vs, Time t)
// {
//   if(ws.size() != num_cameras_
//      || vs.size() != num_cameras_)
//   {
//     LOG(WARNING) << "Number of twists is different than number of cameras."
//                  << "Will not output twists.";
//     return;
//   }
//   CHECK_EQ(ws.size(), num_cameras_);
//   CHECK_EQ(vs.size(), num_cameras_);

//   for(size_t i=0; i<num_cameras_; ++i)
//   {
//     const geometry_msgs::TwistStamped msg = twistToMsg(ws[i], vs[i], t);
//     bag_.write(getTopicName(topic_name_prefix_, i, "twist"),
//                msg.header.stamp, msg);
//   }
// }

// void RosbagWriter::imuCallback(const Vector3& acc, const Vector3& gyr, Time t)
// {
//   VLOG_EVERY_N(1, 500) << "t = " << ze::nanosecToSecTrunc(t) << " s";

//   const sensor_msgs::Imu msg = imuToMsg(acc, gyr, t);
//   const std::string imu_topic = "/imu";
//   bag_.write(imu_topic,
//              msg.header.stamp, msg);
// }

// void RosbagWriter::cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t)
// {
//   CHECK(camera_rig);
//   CHECK_EQ(camera_rig->size(), num_cameras_);

//   static const Duration min_time_interval_between_published_camera_info_
//       = ze::secToNanosec(1.0 / FLAGS_ros_publisher_camera_info_rate);
//   if(last_published_camera_info_time_ > 0 && t - last_published_camera_info_time_ < min_time_interval_between_published_camera_info_)
//   {
//     return;
//   }

//   for(size_t i=0; i<num_cameras_; ++i)
//   {
//     sensor_msgs::CameraInfoPtr msg;
//     msg.reset(new sensor_msgs::CameraInfo);
//     cameraToMsg(camera_rig->atShared(i), t, msg);
//     bag_.write(getTopicName(topic_name_prefix_, i, "camera_info"),
//                msg->header.stamp, msg);
//   }

//   last_published_camera_info_time_ = t;

// }

} // namespace event_camera_simulator