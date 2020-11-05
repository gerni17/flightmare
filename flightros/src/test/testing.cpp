#include "flightros/testing.hpp"

bool testing::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool testing::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

std::string testing::type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_gates");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);
  image_transport::ImageTransport my_image_transport(nh);

  // quad initialization
  testing::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  testing::rgb_camera_ = std::make_unique<RGBCamera>();
  testing::event_camera_ = std::make_unique<EventCamera>();


  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // std::cout << R_BC << std::endl;
  testing::rgb_camera_->setFOV(90);
  testing::rgb_camera_->setWidth(720);
  testing::rgb_camera_->setHeight(480);
  testing::rgb_camera_->setRelPose(B_r_BC, R_BC);
  testing::rgb_camera_->setPostProcesscing(std::vector<bool>{
    false, false, false});  // depth, segmentation, optical flow
  testing::quad_ptr_->addRGBCamera(testing::rgb_camera_);
  testing::event_camera_->setFOV(90);
  testing::event_camera_->setWidth(346);
  testing::event_camera_->setHeight(260);
  testing::event_camera_->setRelPose(B_r_BC, R_BC);
  testing::quad_ptr_->addEventCamera(testing::event_camera_);

  double cp = 0.5;
  double cm = 0.5;


  // // initialization
  testing::quad_state_.setZero();
  testing::quad_ptr_->reset(testing::quad_state_);

  // Initialize gates
  std::string object_id = "unity_gate";
  std::string prefab_id = "rpg_gate";
  std::shared_ptr<StaticGate> gate_1 =
    std::make_shared<StaticGate>(object_id, prefab_id);
  gate_1->setPosition(Eigen::Vector3f(0, 10, 2.5));
  gate_1->setRotation(
    Quaternion(std::cos(0.5 * M_PI_2), 0.0, 0.0, std::sin(0.5 * M_PI_2)));

  std::string object_id_2 = "unity_gate_2";
  std::shared_ptr<StaticGate> gate_2 =
    std::make_unique<StaticGate>(object_id_2, prefab_id);
  gate_2->setPosition(Eigen::Vector3f(0, -10, 2.5));
  gate_2->setRotation(
    Quaternion(std::cos(0.5 * M_PI_2), 0.0, 0.0, std::sin(0.5 * M_PI_2)));

  // ROS
  testing::rgb_pub_ = my_image_transport.advertise("camera/rgb", 1);
  testing::diff_pub_ = my_image_transport.advertise("camera/diff", 1);

  // Set unity bridge
  testing::setUnity(testing::unity_render_);

  // Add gates
  testing::unity_bridge_ptr_->addStaticObject(gate_1);
  testing::unity_bridge_ptr_->addStaticObject(gate_2);

  // connect unity
  testing::connectUnity();

  // Define path through gates
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 10, 2.5));
  way_points.push_back(Eigen::Vector3d(5, 0, 2.5));
  way_points.push_back(Eigen::Vector3d(0, -10, 2.5));
  way_points.push_back(Eigen::Vector3d(-5, 0, 2.5));

  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(num_waypoints);
  minimization_weights << 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);

  polynomial_trajectories::PolynomialTrajectory trajectory =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                        20.0, 20.0, 6.0);

  // Start testing
  testing::manual_timer timer;
  timer.start();
  bool is_first_image = true;
  cv::Mat new_image;
  cv::Mat reconstructed_image;
  reconstructed_image.convertTo(reconstructed_image, CV_64FC1);

  while (ros::ok() && testing::unity_render_ && testing::unity_ready_) {
    timer.stop();

    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(
        // trajectory, ros::Duration(timer.get()/1000));
        trajectory, ros::Duration(testing::event_camera_->getSimTime()));

    testing::quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
    testing::quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
    testing::quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
    testing::quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
    testing::quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
    testing::quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
    testing::quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();

    // std::cout << desired_pose.position;


    testing::quad_ptr_->setState(testing::quad_state_);

    testing::unity_bridge_ptr_->getRender(0);
    testing::unity_bridge_ptr_->handleOutput();

    // add image to addin events


    testing::event_camera_->getRGBImage(new_image);

    cv::Mat planes[3];
    split(new_image, planes);

    // transform to greyscale
    cv::Mat image_gray;
    // // // // // cvtColor(new_image, image_gray, CV_BGR2GRAY);
    cv::Mat log_img;
    planes[0].convertTo(log_img, CV_64FC1);
    cv::Mat dummie = cv::Mat::zeros(new_image.rows, new_image.cols, CV_64FC1);
    ROS_INFO_STREAM("breakpoint");
    dummie += 0.0001;
    cv::log(dummie + 0.0001, dummie);
    ROS_INFO_STREAM("breakpoint_");

    cv::log(log_img + 0.0001, log_img);
    ROS_INFO_STREAM("breakpoint");
    ROS_WARN_STREAM("type new_image: " << testing::type2str(new_image.type())
                                       << " number of channels "
                                       << new_image.channels());
    ROS_WARN_STREAM("type log_img: " << testing::type2str(log_img.type())
                                     << " number of channels "
                                     << log_img.channels());
    ROS_WARN_STREAM("type dummie: " << testing::type2str(dummie.type())
                                    << " number of channels "
                                    << dummie.channels());
    cv::Mat image = log_img - dummie;
    ROS_WARN_STREAM("type image: " << testing::type2str(image.type())
                                   << " number of channels "
                                   << image.channels());

    // Initialize m
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);
    ROS_INFO_STREAM("max of image:" << maxVal << " " << minVal);

    // notsure that this is needed
    // image.convertTo(image_gray, CV_8U);
    // ROS_WARN_STREAM("type image gray: " <<
    // testing::type2str(image_gray.type())
    //                                     << " number of channels "
    //                                     << image_gray.channels());
    if (is_first_image) {
      reconstructed_image = image;
      is_first_image = false;
      ROS_INFO_STREAM("first image");
    }
    cv::Mat last_image = (reconstructed_image);
    // cvtColor(new_image, cv::COLOR_RGB2GRAY);
    // reconstruct new image with event

    ROS_WARN_STREAM("reconstructed type: "
                    << testing::type2str(reconstructed_image.type())
                    << " number of channels "
                    << reconstructed_image.channels());
    int count = 0;
    for (const Event_t& e : testing::event_camera_->getEvents()) {
      if (e.time != 0) {
        int pol;
        // const double C = e.polarity ? cp : cm;
        if (e.polarity > 0)
          pol = 1;
        else
          pol = -1;
        double C;
        if (e.polarity)
          C = cp;
        else
          C = cm;
        float u = reconstructed_image.at<float>(e.coord_y, e.coord_x);
        // reconstructed_image.at<float>(e.coord_y, e.coord_x) += pol * 0.1f;
        float v = reconstructed_image.at<float>(e.coord_y, e.coord_x);
        // ROS_INFO_STREAM("changed from: " << u << " to " << v << " at "
        //                                  << e.coord_x << " and " <<
        //                                  e.coord_y);
      }
      count++;
    }
    double u = reconstructed_image.at<float>(maxLoc);
    double d = u + 0.1f;
    reconstructed_image.at<float>(maxLoc) = d;
    float v = reconstructed_image.at<float>(maxLoc);
    ROS_INFO_STREAM("changed from: " << maxVal << " to " << v <<"and u "<<u<<
                    "d is " << d << " at " << maxLoc);
    ROS_INFO_STREAM("count " << count);
    double minVal_;
    double maxVal_;
    cv::Point minLoc_;
    cv::Point maxLoc_;
    cv::Mat events_image;
    // cv::absdiff(reconstructed_image, last_image, events_image);
    ROS_WARN_STREAM("event type: " << testing::type2str(events_image.type())
                                   << " number of channels "
                                   << events_image.channels());

    cv::minMaxLoc(reconstructed_image, &minVal_, &maxVal_, &minLoc_, &maxLoc_);
    ROS_INFO_STREAM("max of event_image: " << maxVal_ << " " << minVal_
                                           << " and loc " << maxLoc_);

    // sensor_msgs::ImagePtr rgb_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "8UC1", reconstructed_image)
    //     .toImageMsg();
    // testing::rgb_pub_.publish(rgb_msg);
    // // chec that the two images are equal
    // cv::Mat error;
    // // cv::absdiff(reconstructed_image,image_gray,error);

    // sensor_msgs::ImagePtr diff_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "8U_C1", image).toImageMsg();

    // testing::diff_pub_.publish(diff_msg);

    // assign new image to last image for next loop
    reconstructed_image = image;
  }

  return 0;
}
