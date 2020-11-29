#include <flightros/ros_utils.hpp>
// #include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

namespace flightros {

void imageToMsg(const cv::Mat_<ImageFloatType>& image, int64_t t,
                sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImage cv_image;
  // image.convertTo(cv_image.image, CV_8UC3, 255.0);
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}
void imageToMsg(const cv::Mat& image, int64_t t, sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImage cv_image;
  // image.convertTo(cv_image.image, CV_8UC3, 255.0);
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}


void eventsToMsg(const EventsVector& events, int width, int height,
                 dvs_msgs::EventArrayPtr& msg, int64_t starting_time) {
  // CHECK(msg);
  std::vector<dvs_msgs::Event> events_list;
  for (const Event_t& e : events) {
    dvs_msgs::Event ev;
    ev.x = e.coord_x;
    ev.y = e.coord_y;
    ev.ts = toRosTime((e.time*1000 + starting_time) );
    ev.polarity = e.polarity;
    if(e.time>0){
    events_list.push_back(ev);
    }
  }


  msg->events = events_list;
  msg->height = height;
  msg->width = width;
  msg->header.stamp = events_list.back().ts;
}

// sensor_msgs::Imu imuToMsg(const Vector3& acc, const Vector3& gyr, Time t)
// {
//   sensor_msgs::Imu imu;
//   imu.header.stamp = toRosTime(t);

//   imu.linear_acceleration.x = acc(0);
//   imu.linear_acceleration.y = acc(1);
//   imu.linear_acceleration.z = acc(2);

//   imu.angular_velocity.x = gyr(0);
//   imu.angular_velocity.y = gyr(1);
//   imu.angular_velocity.z = gyr(2);

//   return imu;
// }

// geometry_msgs::TwistStamped twistToMsg(const AngularVelocity& w, const
// LinearVelocity& v, Time t)
// {
//   geometry_msgs::TwistStamped twist;
//   twist.header.stamp = toRosTime(t);

//   twist.twist.angular.x = w(0);
//   twist.twist.angular.y = w(1);
//   twist.twist.angular.z = w(2);

//   twist.twist.linear.x = v(0);
//   twist.twist.linear.y = v(1);
//   twist.twist.linear.z = v(2);

//   return twist;
// }

// void cameraToMsg(const ze::Camera::Ptr& camera, Time t,
// sensor_msgs::CameraInfoPtr& msg)
// {
//   CHECK(msg);
//   msg->width = camera->width();
//   msg->height = camera->height();
//   msg->header.stamp = toRosTime(t);

//   CalibrationMatrix K = calibrationMatrixFromCamera(camera);
//   boost::array<double, 9> K_vec;
//   std::vector<double> D_vec;
//   for(int i=0; i<3; ++i)
//   {
//     for(int j=0; j<3; ++j)
//     {
//       K_vec[j+i*3] = static_cast<double>(K(i,j));
//     }
//   }

//   switch(camera->type())
//   {
//     case ze::CameraType::PinholeRadialTangential:
//     case ze::CameraType::Pinhole:
//       msg->distortion_model = "plumb_bob";
//       break;
//     case ze::CameraType::PinholeEquidistant:
//       msg->distortion_model = "equidistant";
//       break;
//     case ze::CameraType::PinholeFov:
//       msg->distortion_model = "fov";
//       break;
//     default:
//       LOG(WARNING) << "Unknown camera distortion model";
//       msg->distortion_model = "";
//       break;
//   }

//   for(int j=0; j<camera->distortionParameters().rows(); ++j)
//   {
//     D_vec.push_back(static_cast<double>(camera->distortionParameters()(j)));
//     // @TODO: use the distortion params from the camera
//   }

//   msg->K = K_vec;
//   msg->D = D_vec;
//   msg->P = {K(0,0), 0,      K(0,2), 0,
//             0,      K(1,1), K(1,2), 0,
//             0,      0,      1,      0};
//   msg->R = {1, 0, 0,
//             0, 1, 0,
//             0, 0, 1};
// }


}  // namespace flightros
