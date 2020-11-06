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
  testing::rgb_pub_ = my_image_transport.advertise("camera/rgb", 1);
  testing::diff_pub_ = my_image_transport.advertise("camera/diff", 1);
  testing::event_pub_ = my_image_transport.advertise("camera/event", 1);

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
  Image I, L, L_reconstructed, L_last,L_second;
  int64_t stamp;
  // reconstructed_image.convertTo(reconstructed_image, CV_64FC1);

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

    cv::Mat new_image;
    testing::event_camera_->getRGBImage(new_image);

    cv::Mat ev_img = testing::event_camera_->createEventimages();
    sensor_msgs::ImagePtr ev_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", ev_img).toImageMsg();
    testing::event_pub_.publish(ev_msg);
    ROS_INFO_STREAM("Type " << testing::type2str(new_image.type()));

    cv::cvtColor(new_image, new_image, CV_BGR2GRAY);

    ROS_INFO_STREAM("Type " << testing::type2str(new_image.type()));

    // cv::Mat planes[3];
    // split(new_image, planes);
    // I = planes[0];

    new_image.convertTo(I, cv::DataType<ImageFloatType>::type);

    Image dummie =
      cv::Mat::zeros(I.rows, I.cols, cv::DataType<ImageFloatType>::type);

    Image log_img;
    cv::log(I + 0.0001, log_img);

    cv::log(dummie + 0.0001, dummie);
    L = log_img - dummie;

    // cv::log(0.00001 + I, L);

    if (is_first_image) {
      // Initialize reconstructed image from the ground truth image
      L_second = L_last.clone();
      L_reconstructed = L.clone();
      L_last = L.clone();
      is_first_image = false;
    }
    int counter = 0;
    int counter_ = 0;
    int count = 0;
    for (const Event_t& e : testing::event_camera_->getEvents()) {
      if (e.time != 0) {
        int pol;
        if (e.polarity == 1) {
          pol = 1;
          count++;
        } else if (e.polarity == -1) {
          pol = -1;
          counter++;
        } else
          counter_++;
        // ROS_INFO_STREAM("Polarity  " << pol);
        const ImageFloatType C = e.polarity ? cp : cm;
        // ROS_INFO_STREAM(
        //   "Values before: " << L_reconstructed(e.coord_y, e.coord_x));

        L_reconstructed(e.coord_y, e.coord_x) += pol * 0.1;
        // ROS_INFO_STREAM(
        // "Values after: " << L_reconstructed(e.coord_y, e.coord_x))
      }
    }
    ROS_INFO_STREAM("Amount of pos events  " << count << " of neg " << counter
                                             << " else " << counter_);
    // Here all informations are gathered and we only need to evaluate it

    // clculate the total error of the reconsstruction
    ImageFloatType total_error = 0;
    for (int y = 0; y < I.rows; ++y) {
      for (int x = 0; x < I.cols; ++x) {
        const ImageFloatType reconstruction_error =
          std::fabs(L_reconstructed(y, x) - L(y, x));
        total_error += reconstruction_error;
      }
    }
    ROS_INFO_STREAM("Total error " << total_error);

    // calculate total difference of two consecutive images

    total_error = 0;
    for (int y = 0; y < I.rows; ++y) {
      for (int x = 0; x < I.cols; ++x) {
        total_error += std::fabs(L_last(y, x) - L(y, x));
      }
    }
    ROS_INFO_STREAM("Total diference between two images " << total_error);

    // clculate std deviation and mean of the error
    Image error, real_diff;
    cv::absdiff(L, L_reconstructed, error);
    cv::absdiff(L, L_last, real_diff);

    cv::Scalar mean_error, stddev_error, mean, stddevv;
    cv::meanStdDev(L, mean, stddevv);
    cv::meanStdDev(error, mean_error, stddev_error);
    ROS_INFO_STREAM("Mean of img: " << mean << ", Stddev: " << stddevv);
    ROS_INFO_STREAM("Mean error: " << mean_error
                                   << ", Stddev: " << stddev_error);
    ROS_INFO_STREAM("Type " << testing::type2str(error.type()));
    double minVal, maxVal, minVal_, maxVal_;
    cv::minMaxLoc(error, &minVal, &maxVal);
    cv::minMaxLoc(real_diff, &minVal_, &maxVal_);
    // publish the two iamge
    cv::Mat draw;
    error.convertTo(draw, CV_8U, 255.0 / (maxVal - minVal), -minVal);
    ROS_INFO_STREAM("Type " << testing::type2str(error.type()));

    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", draw).toImageMsg();
    testing::rgb_pub_.publish(rgb_msg);

    cv::Mat draw_;
    real_diff.convertTo(draw_, CV_8U, 255.0 / (maxVal_ - minVal_), -minVal_);
    ROS_INFO_STREAM("The biggest val of last " << maxVal_ << " and "
                                               << minVal_);
    ROS_INFO_STREAM("The biggest val of reconstructed " << maxVal << " and "
                                                        << minVal);


    sensor_msgs::ImagePtr diff_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_).toImageMsg();
    testing::diff_pub_.publish(diff_msg);

    // reset initial conditions
    L_second = L_last.clone();
    L_reconstructed = L.clone();
    L_last = L.clone();
  }

  return 0;
}
