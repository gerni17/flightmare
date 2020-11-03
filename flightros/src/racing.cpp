#include "flightros/racing.hpp"

bool racing::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool racing::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}


int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_gates");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);
  image_transport::ImageTransport my_image_transport(nh);

  // quad initialization
  racing::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  racing::rgb_camera_ = std::make_unique<RGBCamera>();
  racing::event_camera_ = std::make_unique<EventCamera>();

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  racing::rgb_camera_->setFOV(90);
  racing::rgb_camera_->setWidth(720);
  racing::rgb_camera_->setHeight(480);
  racing::rgb_camera_->setRelPose(B_r_BC, R_BC);
  racing::rgb_camera_->setPostProcesscing(std::vector<bool>{
    false, false, false});  // depth, segmentation, optical flow
  racing::quad_ptr_->addRGBCamera(racing::rgb_camera_);
  racing::event_camera_->setFOV(90);
  racing::event_camera_->setWidth(346);
  racing::event_camera_->setHeight(260);
  racing::event_camera_->setRelPose(B_r_BC, R_BC);
  racing::quad_ptr_->addEventCamera(racing::event_camera_);

  // // initialization
  racing::quad_state_.setZero();
  racing::quad_ptr_->reset(racing::quad_state_);

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
  racing::rgb_pub_ = my_image_transport.advertise("camera/rgb_image", 1);


  // Set unity bridge
  racing::setUnity(racing::unity_render_);

  // Add gates
  racing::unity_bridge_ptr_->addStaticObject(gate_1);
  racing::unity_bridge_ptr_->addStaticObject(gate_2);

  // connect unity
  racing::connectUnity();

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

  // Start racing
  racing::manual_timer timer;
  timer.start();

  while (ros::ok() && racing::unity_render_ && racing::unity_ready_) {
    timer.stop();

    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(
        // trajectory, ros::Duration(timer.get()/1000));
    trajectory, ros::Duration(racing::event_camera_->getSimTime()));

    racing::quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
    racing::quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
    racing::quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
    racing::quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
    racing::quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
    racing::quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
    racing::quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();

    std::cout << desired_pose.position;


    racing::quad_ptr_->setState(racing::quad_state_);

    racing::unity_bridge_ptr_->getRender(0);
    racing::unity_bridge_ptr_->handleOutput();

    racing::event_camera_->getRGBImage(racing::rgb_image);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8",
      racing::rgb_image).toImageMsg();
    racing::rgb_pub_.publish(rgb_msg);
  }

  return 0;
}
