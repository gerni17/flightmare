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
  std::cout << R_BC << std::endl;
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

  double cp = 0.1;
  double cm = 0.1;


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
  testing::rgb_pub_ = my_image_transport.advertise("camera/rgb_image", 1);
  testing::diff_pub_ = my_image_transport.advertise("camera/diff_image", 1);

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

    std::cout << desired_pose.position;


    testing::quad_ptr_->setState(testing::quad_state_);

    testing::unity_bridge_ptr_->getRender(0);
    testing::unity_bridge_ptr_->handleOutput();

    // add image to addin events


    testing::event_camera_->getRGBImage(new_image);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_image).toImageMsg();
    testing::rgb_pub_.publish(rgb_msg);

    // transform to greyscale
    cv::Mat image_gray;
    cvtColor(new_image, image_gray, CV_BGR2GRAY);
    if (is_first_image) {
      reconstructed_image = image_gray;
      is_first_image = false;
    }
    // cvtColor(new_image, cv::COLOR_RGB2GRAY);
    // reconstruct new image with event
    for (const Event_t& e : testing::event_camera_->getEvents()) {
      if (e.time != 0) {
        // double pol = e.polarity ? 1. : -1.;
        // const double C = e.polarity ? cp : cm;
        double C;
        if (e.polarity)
          C = cp;
        else
          C = cm;

        reconstructed_image.at<uchar>(e.coord_y, e.coord_x) += e.polarity * C;
      }
    }

    // chec that the two images are equal
    cv::Mat difference = (reconstructed_image - image_gray);

    sensor_msgs::ImagePtr diff_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", difference).toImageMsg();

    testing::diff_pub_.publish(diff_msg);

    // assign new image to last image for next loop
    reconstructed_image = image_gray;
  }

  return 0;
}
