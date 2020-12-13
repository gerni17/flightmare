#include "flightros/record.hpp"

// DEFINE_string(path_to_output_bag, "",
// "Path to which save the output bag file.");

bool record::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool record::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

std::string record::type2str(int type) {
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
void record::saveToFile(std::vector<Event_t> events) {
  // CHECK_EQ(events.size(), 1);
  for (const Event_t& e : events) {
    // rearrange?
    if (e.polarity != 0) {
      record::events_text_file_ << e.time << " " << e.coord_x << " "
                                << e.coord_y << " " << e.polarity << std::endl;
    }
  }
}

int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_gates");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
    image_transport::ImageTransport my_image_transport(nh);

  ros::Rate(50.0);

  // quad initialization
  record::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  record::rgb_camera_ = std::make_unique<RGBCamera>();
  record::event_camera_ = std::make_unique<EventCamera>();

  record::scene_id_ = 4;
  record::rgb_pub_ = my_image_transport.advertise("camera/rgb", 1);


  int frame = 0;
  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // std::cout << R_BC << std::endl;
  record::rgb_camera_->setFOV(90);
  record::rgb_camera_->setWidth(352);
  record::rgb_camera_->setHeight(264);
  record::rgb_camera_->setRelPose(B_r_BC, R_BC);
  record::rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, true});  // depth, segmentation, optical flow
  record::quad_ptr_->addRGBCamera(record::rgb_camera_);
  record::event_camera_->setFOV(90);
  record::event_camera_->setWidth(352);
  record::event_camera_->setHeight(264);
  record::event_camera_->setRelPose(B_r_BC, R_BC);
  record::event_camera_->setCp(0.1);
  record::event_camera_->setCm(0.1);
  record::event_camera_->setsigmaCm(0.0);
  record::event_camera_->setsigmaCp(0.0);
  record::event_camera_->setRefractory(1);
  record::event_camera_->setLogEps(0.0001);

  record::quad_ptr_->addEventCamera(record::event_camera_);


  double cp = record::event_camera_->getCp();
  double cm = record::event_camera_->getCm();
  bool record = false;
  // // initialization
  record::quad_state_.setZero();
  record::quad_ptr_->reset(record::quad_state_);

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


  record::writer_ = std::make_shared<RosbagWriter>(record::path_to_output_bag);

  // Set unity bridge
  record::setUnity(record::unity_render_);

  // Add gates
  // record::unity_bridge_ptr_->addStaticObject(gate_1);
  // record::unity_bridge_ptr_->addStaticObject(gate_2);

  // connect unity
  record::connectUnity();


  quadrotor_common::TrajectoryPoint start_pt= quadrotor_common::TrajectoryPoint();
  // Define path through gates
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 20, 2.5));
  way_points.push_back(Eigen::Vector3d(2, 0, 2.5));
  way_points.push_back(Eigen::Vector3d(0, -20, 2.5));
  way_points.push_back(Eigen::Vector3d(-2, 0, 2.5));
  // way_points.push_back(Eigen::Vector3d(-16, -13, 2.5));
  // way_points.push_back(Eigen::Vector3d(-14, -14, 2.5));
  // way_points.push_back(Eigen::Vector3d(-2, -13, 2.5));
  // way_points.push_back(Eigen::Vector3d(0, -1, 2.5));
  // way_points.push_back(Eigen::Vector3d(0, 80, 2.5));
  // way_points.push_back(Eigen::Vector3d(0, -10, 2));

  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(num_waypoints);
  minimization_weights << 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);//these numbers here represent the order and the continuity...

  polynomial_trajectories::PolynomialTrajectory trajectory =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                        20.0, 20.0, 6.0);//theselast three are max veloci and acc

  // record::events_text_file_.open("/home/gian/Desktop/events");
  // Start record

  cv::Mat new_image, of_image, depth_image,ev_image;
  Image I;
  ROS_INFO_STREAM("Cp value " << cp);

  while (ros::ok() && record::unity_render_ && record::unity_ready_) {
    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(
        trajectory, ros::Duration(record::event_camera_->getSecSimTime()));

    record::quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
    record::quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
    record::quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
    record::quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
    record::quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
    record::quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
    record::quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();

    ze::Transformation twc;
    record::quad_state_.qx.normalize();
    twc.getRotation() = ze::Quaternion(
      record::quad_state_.x[QS::ATTW], record::quad_state_.x[QS::ATTX],
      record::quad_state_.x[QS::ATTY], record::quad_state_.x[QS::ATTZ]);

    twc.getPosition() = ze::Position((Scalar)desired_pose.position.x(),
                                     (Scalar)desired_pose.position.y(),
                                     (Scalar)desired_pose.position.z());
    record::writer_->poseCallback(twc,
                                  record::event_camera_->getNanoSimTime());

    ROS_INFO_STREAM("pose " <<(Scalar)desired_pose.position.x() <<"/"<<(Scalar)desired_pose.position.y()<<"/"<<(Scalar)desired_pose.position.z()<<"/"<<
     record::event_camera_->getSecSimTime());

    record::quad_ptr_->setState(record::quad_state_);

    record::unity_bridge_ptr_->getRender(0);
    record::unity_bridge_ptr_->handleOutput();


    record::event_camera_->getRGBImage(ev_image);
    record::rgb_camera_->getRGBImage(new_image);
    record::rgb_camera_->getOpticalFlow(of_image);
    record::rgb_camera_->getDepthMap(depth_image);

        sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8",depth_image).toImageMsg();
    record::rgb_pub_.publish(rgb_msg);

    // cv::cvtColor(new_image, new_image, CV_BGR2GRAY);

    // new_image.convertTo(I, cv::DataType<ImageFloatType>::type, 1. / 255.);

    // auto img_ptr = std::make_shared<Image>(I);
    RGBImagePtr rgb_img_ptr = std::make_shared<RGBImage>(ev_image);
    RGBImagePtr of_img_ptr = std::make_shared<RGBImage>(of_image);
    RGBImagePtr depth_img_ptr = std::make_shared<RGBImage>(depth_image);

    if (frame > 10 && record::event_camera_->getImgStore()) {
      ROS_INFO_STREAM("starting to record");
      record = true;
    }

    if (record) {
      // add image to addin events
      if (record::event_camera_->getImgStore()) {
        record::writer_->imageRGBCallback(
          rgb_img_ptr, record::event_camera_->getNanoSimTime());
        record::writer_->imageOFCallback(
          of_img_ptr, record::event_camera_->getNanoSimTime());
        record::writer_->imageDepthCallback(depth_img_ptr, record::event_camera_->getNanoSimTime());

        ROS_INFO_STREAM("image");
      }

      const EventsVector& events = record::event_camera_->getEvents();
      record::writer_->eventsCallback(events,
                                      record::event_camera_->getNanoSimTime());
    }
    // clear the buffer
    record::event_camera_->deleteEventQueue();


    frame++;
  }
  record::events_text_file_.close();
  return 0;
}
